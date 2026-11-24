#
# Copyright (C) 2022 DroneCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
'''
 driver for CAN over MAVLink, using MAV_CMD_CAN_FORWARD and CAN_FRAME messages

 Parent process death detection is most reliable on Python 3.8+ because
 multiprocessing.parent_process() is used when available. On Windows with
 Python 3.7, this API is unavailable and os.getppid() is not used as fallback,
 so parent death detection is effectively disabled.
'''

import os
import sys
import time
import multiprocessing
from logging import getLogger
from .common import DriverError, CANFrame, AbstractDriver
from pymavlink import mavutil

try:
    import queue
except ImportError:
    # noinspection PyPep8Naming,PyUnresolvedReferences
    import Queue as queue

if 'darwin' in sys.platform:
    RX_QUEUE_SIZE = 32767   # http://stackoverflow.com/questions/5900985/multiprocessing-queue-maxsize-limit-is-32767
else:
    RX_QUEUE_SIZE = 1000000
TX_QUEUE_SIZE = 1000
TX_PRIORITY_QUEUE_SIZE = 500

logger = getLogger(__name__)
kill_process = False


MAVCAN_RECV_TIMEOUT_IDLE_SEC = 0.001
MAVCAN_RECV_TIMEOUT_BUSY_SEC = 0.000
MAVCAN_DEFAULT_BAUDRATE = 921600
MAVCAN_HIGH_PRIORITY_MAX = 10


class ControlMessage(object):
    def __init__(self, command, data):
        self.command = command
        self.data = data

def io_process(url, bus, target_system, baudrate, tx_queue, tx_priority_queue, rx_queue, exit_queue, parent_pid):
    os.environ['MAVLINK20'] = '1'

    if os.name == 'nt':
        try:
            import ctypes
            ABOVE_NORMAL_PRIORITY_CLASS = 0x00008000
            ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(),
                                                    ABOVE_NORMAL_PRIORITY_CLASS)
        except Exception:
            logger.debug('Could not raise MAVCAN IO process priority on Windows', exc_info=True)

    # If the parent process dies unexpectedly, stop this IO process to avoid leaving a stale
    # MAVLink connection running that can interfere with new instances.
    parent_sentinel = None
    mp_wait = None
    has_parent_process_api = hasattr(multiprocessing, 'parent_process')
    try:
        parent = multiprocessing.parent_process() if has_parent_process_api else None
        if parent is not None:
            parent_sentinel = parent.sentinel
            from multiprocessing.connection import wait as mp_wait  # type: ignore
    except Exception:
        parent_sentinel = None
        mp_wait = None
    target_component = 0
    last_enable = time.time()
    conn = None
    filter_list = None
    signing_key = None
    firmware_update_mode = False
    last_loss_print_t = time.time()
    exit_proc = False
    readonly = False

    def connect():
        nonlocal conn, baudrate, readonly
        conn = mavutil.mavlink_connection(url, baud=baudrate, source_system=250,
                                          source_component=mavutil.mavlink.MAV_COMP_ID_MAVCAN,
                                          dialect='ardupilotmega')
        if conn is None:
            raise DriverError('unable to connect to %s' % url)
        nonlocal signing_key
        if signing_key is not None:
            conn.setup_signing(signing_key, sign_outgoing=True)

        readonly = isinstance(conn, mavutil.mavlogfile)

    def reconnect():
        nonlocal exit_proc
        while True and not exit_proc:
            if (not exit_queue.empty() and exit_queue.get() == "QUIT"):
                exit_proc = True
                return
            try:
                time.sleep(1)
                logger.info('reconnecting to %s' % url)
                connect()
                return
            except Exception:
                continue

    def enable_can_forward():
        '''re-enable CAN forwarding. Called at 1Hz'''
        if readonly:
            return
        nonlocal last_enable, bus, target_system
        last_enable = time.time()
        conn.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_CAN_FORWARD,
            0,
            bus+1,
            0,
            0,
            0,
            0,
            0,
            0)
        if filter_list is not None:
            ids = sorted(filter_list[:16])
            num_ids = len(ids)
            if len(ids) < 16:
                ids += [0]*(16-num_ids)
            try:
                conn.mav.can_filter_modify_send(
                    target_system,
                    target_component,
                    bus+1,
                    mavutil.mavlink.CAN_FILTER_REPLACE,
                    num_ids,
                    ids)
            except Exception as ex:
                print(ex)

    def handle_control_message(m):
        '''handle a ControlMessage'''
        if m.command == "BusNum":
            nonlocal bus
            bus = int(m.data)
        elif m.command == "FilterList":
            nonlocal filter_list
            filter_list = m.data
        elif m.command == "SigningKey":
            nonlocal signing_key
            signing_key = m.data
            conn.setup_signing(signing_key, sign_outgoing=True)
        elif m.command == 'FirmwareUpdateMode':
            nonlocal firmware_update_mode
            firmware_update_mode = bool(m.data)

    connect()
    enable_can_forward()

    if os.name == 'nt' and not has_parent_process_api:
        logger.warning('Python 3.8+ is recommended on Windows for parent process death detection in MAVCAN IO process')

    while True:
        drained_count = 0
        pending_control_message = None
        try:
            if mp_wait is not None and parent_sentinel is not None and mp_wait([parent_sentinel], timeout=0):
                # Parent process is gone.
                conn.close()
                return
        except Exception:
            pass
        if (not exit_queue.empty() and exit_queue.get() == "QUIT") or exit_proc:
            conn.close()
            return
        # Keep the old PID check only as a last-resort fallback on POSIX.
        # On Windows with Python < 3.8, parent_process() is unavailable and
        # this fallback is intentionally disabled, so parent death detection
        # is effectively unavailable.
        if parent_sentinel is None and os.name != 'nt':
            try:
                if os.getppid() != parent_pid:
                    conn.close()
                    return
            except Exception:
                pass
        while True:
            if (not exit_queue.empty() and exit_queue.get() == "QUIT") or exit_proc:
                conn.close()
                return
            try:
                frame = tx_priority_queue.get_nowait()
            except queue.Empty:
                break
            drained_count += 1
            if readonly:
                continue
            if isinstance(frame, ControlMessage):
                # Controls are queued separately for responsiveness, but must
                # not overtake data already waiting in the normal FIFO queue.
                pending_control_message = frame
                break
            message_id = frame.id
            if frame.extended:
                message_id |= 1<<31
            message = frame.data
            mlen = len(message)
            if mlen < frame.MAX_DATA_LENGTH:
                message += bytearray([0]*(frame.MAX_DATA_LENGTH-mlen))
            try:
                send_started_at = time.monotonic()
                if frame.canfd:
                    conn.mav.canfd_frame_send(
                        target_system,
                        target_component,
                        bus,
                        mlen,
                        message_id,
                        message)
                else:
                    conn.mav.can_frame_send(
                        target_system,
                        target_component,
                        bus,
                        mlen,
                        message_id,
                        message)
            except Exception as ex:
                print(ex)
            if time.time() - last_enable > 1:
                enable_can_forward()

        while True:
            if (not exit_queue.empty() and exit_queue.get() == "QUIT") or exit_proc:
                conn.close()
                return
            try:
                frame = tx_queue.get_nowait()
            except queue.Empty:
                break
            drained_count += 1
            if readonly:
                continue
            if isinstance(frame, ControlMessage):
                handle_control_message(frame)
                continue
            message_id = frame.id
            if frame.extended:
                message_id |= 1<<31
            message = frame.data
            mlen = len(message)
            if mlen < frame.MAX_DATA_LENGTH:
                message += bytearray([0]*(frame.MAX_DATA_LENGTH-mlen))
            try:
                send_started_at = time.monotonic()
                if frame.canfd:
                    conn.mav.canfd_frame_send(
                        target_system,
                        target_component,
                        bus,
                        mlen,
                        message_id,
                        message)
                else:
                    conn.mav.can_frame_send(
                        target_system,
                        target_component,
                        bus,
                        mlen,
                        message_id,
                        message)
            except Exception as ex:
                print(ex)
            if time.time() - last_enable > 1:
                enable_can_forward()

        if pending_control_message is not None:
            handle_control_message(pending_control_message)

        recv_timeout_sec = MAVCAN_RECV_TIMEOUT_BUSY_SEC if drained_count > 0 else MAVCAN_RECV_TIMEOUT_IDLE_SEC
        if recv_timeout_sec > 0.0 and (not tx_priority_queue.empty() or not tx_queue.empty()):
            recv_timeout_sec = MAVCAN_RECV_TIMEOUT_BUSY_SEC
        try:
            m = conn.recv_match(type=['CAN_FRAME', 'CANFD_FRAME'], blocking=True, timeout=recv_timeout_sec)
        except Exception as ex:
            reconnect()
            continue
        now = time.time()
        if m is None:
            if now - last_enable > 1:
                enable_can_forward()
            if now - last_loss_print_t > 5:
                last_loss_print_t = now
                print("MAVLink packet loss %.2f%%" % conn.packet_loss())
            continue
        if target_system == 0:
            target_system = m.get_srcSystem()
        is_extended = (m.id & (1<<31)) != 0
        is_canfd = m.get_type() == 'CANFD_FRAME'
        canid = m.id & 0x1FFFFFFF
        frame = CANFrame(canid, m.data[:m.len], is_extended, canfd=is_canfd)
        rx_queue.put_nowait(frame)


# MAVLink CAN driver
#
class MAVCAN(AbstractDriver):
    """
    Driver for MAVLink CAN bus adapters, using CAN_FRAME MAVLink packets
    """

    def __init__(self, url, **kwargs):
        super(MAVCAN, self).__init__()
        self.bus = kwargs.get('bus_number', 1) - 1
        self.target_system = kwargs.get('mavlink_target_system', 0)
        self.filter_list = None
        baudrate = kwargs.get('baudrate', MAVCAN_DEFAULT_BAUDRATE)

        self.rx_queue = multiprocessing.Queue(maxsize=RX_QUEUE_SIZE)
        self.tx_queue = multiprocessing.Queue(maxsize=TX_QUEUE_SIZE)
        self.tx_priority_queue = multiprocessing.Queue(maxsize=TX_PRIORITY_QUEUE_SIZE)
        self.exit_queue = multiprocessing.Queue(maxsize=1)
        self.firmware_update_mode = False

        self.proc = multiprocessing.Process(target=io_process, name='mavcan_io_process',
                                            args=(url, self.bus, self.target_system, baudrate,
                            self.tx_queue, self.tx_priority_queue,
                            self.rx_queue, self.exit_queue, os.getpid()))
        self.proc.daemon = True
        self.proc.start()

        # allow signing pass phrase in environment
        pass_phrase = os.environ.get("DRONECAN_SIGNING_KEY",None)
        if pass_phrase:
            self.set_signing_passphrase(pass_phrase)

    def close(self):
        if self.proc is not None:
            self.exit_queue.put_nowait("QUIT")
            self.proc.join()

    def __del__(self):
        self.close()

    @staticmethod
    def _extract_uavcan_priority(frame):
        if not getattr(frame, 'extended', False):
            return None
        return (frame.id >> 24) & 0x1F

    def _put_control_message(self, message):
        # Control messages should be applied ASAP to keep runtime mode transitions responsive.
        self.tx_priority_queue.put_nowait(message)

    def _is_high_priority_frame(self, frame):
        if not self.firmware_update_mode:
            return False
        priority = self._extract_uavcan_priority(frame)
        return priority is not None and priority <= MAVCAN_HIGH_PRIORITY_MAX

    def receive(self, timeout=None):
        tstart = time.time()
        while True:
            try:
                frame = self.rx_queue.get(block=0)
            except queue.Empty:
                frame = None
            if frame is not None:
                self._rx_hook(frame)
                return frame
            if timeout is not None:
                timeout = max(timeout, 0.001)
                if time.time() >= tstart + timeout:
                    return

    def send_frame(self, frame):
        self._tx_hook(frame)
        if self._is_high_priority_frame(frame):
            self.tx_priority_queue.put_nowait(frame)
        else:
            self.tx_queue.put_nowait(frame)

    def detect_mavlink_baud(device_name, baudrate):
        '''return the baudrate of a MAVLink device, or None if none is detected'''
        os.environ['MAVLINK20'] = '1'
        baud_candidates = [baudrate, MAVCAN_DEFAULT_BAUDRATE, 115200]
        seen = set()
        ordered_bauds = []
        for b in baud_candidates:
            if b in seen:
                continue
            seen.add(b)
            ordered_bauds.append(b)

        for baud in ordered_bauds:
            conn = None
            try:
                conn = mavutil.mavlink_connection(device_name,
                                                  baud=baud,
                                                  source_system=250,
                                                  source_component=mavutil.mavlink.MAV_COMP_ID_MAVCAN)
                if not conn:
                    continue
                m = conn.recv_match(blocking=True, type=['HEARTBEAT', 'ATTITUDE', 'SYS_STATUS'], timeout=1.1)
                if m is not None:
                    return baud
            except Exception:
                continue
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass

        return None

    def is_mavlink_port(device_name, baudrate):
        '''check if a device is sending mavlink'''
        return MAVCAN.detect_mavlink_baud(device_name, baudrate) is not None

    def set_filter_list(self, ids):
        '''set list of message IDs to accept, sent to the remote capture node with mavcan'''
        self.filter_list = ids
        self._put_control_message(ControlMessage('FilterList', self.filter_list))

    def get_filter_list(self, ids):
        '''set list of message IDs to accept, sent to the remote capture node with mavcan'''
        return self.filter_list

    def set_bus(self, busnum):
        '''set the remote bus number to attach to'''
        if busnum <= 0:
            raise DriverError('invalid bus %s' % busnum)
        self.bus = busnum - 1
        self._put_control_message(ControlMessage('BusNum', self.bus))

    def get_bus(self):
        '''get the remote bus number we are attached to'''
        return self.bus+1

    def get_filter_list(self):
        '''get the current filter list'''
        return self.filter_list

    def passphrase_to_key(self, passphrase):
        '''convert a passphrase to a 32 byte key'''
        import hashlib
        h = hashlib.new('sha256')
        if sys.version_info[0] >= 3:
            passphrase = passphrase.encode('ascii')
        h.update(passphrase)
        return h.digest()

    def set_signing_passphrase(self, passphrase):
        '''set MAVLink2 signing passphrase'''
        signing_key = self.passphrase_to_key(passphrase)
        self._put_control_message(ControlMessage('SigningKey', signing_key))

    def set_firmware_update_mode(self, enabled):
        self.firmware_update_mode = bool(enabled)
        self._put_control_message(ControlMessage('FirmwareUpdateMode', self.firmware_update_mode))
