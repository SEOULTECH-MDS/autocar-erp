#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

try:
    import hid  # from hidapi
except Exception:
    hid = None

try:
    import usb.core
    import usb.util
except Exception:
    usb = None


class TowerLampTest(Node):

    def __init__(self) -> None:
        super().__init__('tower_lamp_test')

        # Parameters
        self.declare_parameter('vid', 0x04d8)
        self.declare_parameter('pid', 0xe73c)
        self.declare_parameter('red_index', 0)
        self.declare_parameter('green_index', 2)
        self.declare_parameter('sound_on_value', 3)
        self.declare_parameter('sound_off_value', 0)
        self.declare_parameter('hid_report_id', 0)
        self.declare_parameter('pad_len', 16)  # total bytes to write (including report id if used)
        self.declare_parameter('use_feature', False)  # use send_feature_report instead of write
        self.declare_parameter('prepend_index_type', True)  # prepend Q_index, Q_type
        self.declare_parameter('q_index', 0)
        self.declare_parameter('q_type', 0)
        self.declare_parameter('hid_path', '')  # optional explicit path
        self.declare_parameter('use_pyusb', True)
        self.declare_parameter('pyusb_interface', -1)
        self.declare_parameter('pyusb_endpoint_out', -1)
        self.declare_parameter('pyusb_use_ctrl', False)  # use HID SET_REPORT(Output) control transfer

        # What to do: mode can be 'off', 'red', 'green', 'all_on', 'estop', 'auto'
        #  - 'estop': red ON, buzzer OFF
        #  - 'auto' : green ON, buzzer 1Hz toggle
        # Additionally, allow direct overrides
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('toggle_buzzer', True)
        self.declare_parameter('use_qlight_hex_map', True)  # send vendor-captured 16B payloads

        self.vid = int(self.get_parameter('vid').get_parameter_value().integer_value)
        self.pid = int(self.get_parameter('pid').get_parameter_value().integer_value)
        self.red_index = int(self.get_parameter('red_index').get_parameter_value().integer_value)
        self.green_index = int(self.get_parameter('green_index').get_parameter_value().integer_value)
        self.sound_on_value = int(self.get_parameter('sound_on_value').get_parameter_value().integer_value)
        self.sound_off_value = int(self.get_parameter('sound_off_value').get_parameter_value().integer_value)
        self.hid_report_id = int(self.get_parameter('hid_report_id').get_parameter_value().integer_value)
        self.hid_path = str(self.get_parameter('hid_path').get_parameter_value().string_value)
        self.pad_len = int(self.get_parameter('pad_len').get_parameter_value().integer_value)
        self.use_feature = bool(self.get_parameter('use_feature').get_parameter_value().bool_value)
        self.prepend_index_type = bool(self.get_parameter('prepend_index_type').get_parameter_value().bool_value)
        self.q_index = int(self.get_parameter('q_index').get_parameter_value().integer_value)
        self.q_type = int(self.get_parameter('q_type').get_parameter_value().integer_value)
        self.use_pyusb = bool(self.get_parameter('use_pyusb').get_parameter_value().bool_value)
        self.pyusb_interface = int(self.get_parameter('pyusb_interface').get_parameter_value().integer_value)
        self.pyusb_endpoint_out = int(self.get_parameter('pyusb_endpoint_out').get_parameter_value().integer_value)
        self.pyusb_use_ctrl = bool(self.get_parameter('pyusb_use_ctrl').get_parameter_value().bool_value)

        self.mode = str(self.get_parameter('mode').get_parameter_value().string_value)
        self.toggle_buzzer = bool(self.get_parameter('toggle_buzzer').get_parameter_value().bool_value)
        self.use_qlight_hex_map = bool(self.get_parameter('use_qlight_hex_map').get_parameter_value().bool_value)

        # Choose backend (pyusb or hidapi)
        self.dev = None
        self.usb_dev = None
        self.usb_ep_out = None
        self.usb_iface_num = None

        # Simplified: force pyusb backend only
        self.use_pyusb = True
        self._usb_open()

        self.prev_payload = None
        self.buzzer_on = False

        # Immediate apply once
        self._apply_once()

        # If mode requires buzzer toggle, set a 1s timer
        if self.mode == 'auto' and self.toggle_buzzer:
            self.timer = self.create_timer(1.0, self._toggle_timer)

    def _apply_once(self) -> None:
        if self.use_qlight_hex_map:
            # Use vendor-captured 16B payloads (persist across sends due to D_not semantics)
            if self.mode == 'off':
                self._send_known('RED_OFF')
                self._send_known('GREEN_OFF')
                self._send_known('BUZZER_OFF')
            elif self.mode == 'red' or self.mode == 'estop':
                self._send_known('RED_ON')
                self._send_known('GREEN_OFF')
                self._send_known('BUZZER_OFF')
            elif self.mode == 'green':
                self._send_known('GREEN_ON')
            elif self.mode == 'auto':
                # Keep green on; buzzer handled by timer
                self._send_known('GREEN_ON')
                if self.toggle_buzzer:
                    self._send_known('BUZZER_A' if self.buzzer_on else 'BUZZER_OFF')
            elif self.mode == 'all_on':
                self._send_known('RED_ON')
                self._send_known('GREEN_ON')
                self._send_known('BUZZER_A')
            else:
                self._send_known('RED_OFF')
                self._send_known('GREEN_OFF')
                self._send_known('BUZZER_OFF')
            return
        else:
            lamp = [0, 0, 0, 0, 0]
            sound = self.sound_off_value

            if self.mode == 'off':
                lamp = [0, 0, 0, 0, 0]
                sound = self.sound_off_value
            elif self.mode == 'red' or self.mode == 'estop':
                lamp[self.red_index] = 1
                if 0 <= self.green_index < 5:
                    lamp[self.green_index] = 0
                sound = self.sound_off_value
            elif self.mode == 'green' or self.mode == 'auto':
                if 0 <= self.red_index < 5:
                    lamp[self.red_index] = 0
                lamp[self.green_index] = 1
                sound = self.sound_on_value if (self.mode == 'auto' and self.toggle_buzzer and self.buzzer_on) else self.sound_off_value
            elif self.mode == 'all_on':
                lamp = [1, 1, 1, 1, 1]
                sound = self.sound_on_value
            else:
                lamp = [0, 0, 0, 0, 0]
                sound = self.sound_off_value

            payload = bytes(lamp + [sound])
            if payload != self.prev_payload:
                self.prev_payload = payload
                self._send_output(payload)

    def _toggle_timer(self) -> None:
        # Toggle buzzer and resend in 'auto' mode (green ON)
        if self.mode != 'auto':
            return
        self.buzzer_on = not self.buzzer_on
        if self.use_qlight_hex_map:
            self._send_known('BUZZER_A' if self.buzzer_on else 'BUZZER_OFF')
        else:
            self._apply_once()

    # ==== Q-Light known payloads (16 bytes) ====
    def _send_known(self, key: str) -> None:
        table = {
            'RED_ON': bytes.fromhex('57050164646464644000b0f000000000'),
            'RED_BLINK': bytes.fromhex('57050264646464644000b0f000000000'),
            'RED_OFF': bytes.fromhex('57050064646464644000b0f000000000'),
            'GREEN_ON': bytes.fromhex('57056464016464644000b0f000000000'),
            'GREEN_BLINK': bytes.fromhex('57056464026464644000b0f000000000'),
            'GREEN_OFF': bytes.fromhex('57056464006464644000b0f000000000'),
            'BUZZER_A': bytes.fromhex('57056464646464014000b0f000000000'),
            'BUZZER_OFF': bytes.fromhex('57056464646464004000f0f200000000'),
        }
        data = table.get(key)
        if not data:
            self.get_logger().warn(f"unknown key: {key}")
            return
        # Send as raw 16 bytes via chosen backend
        try:
            if self.use_pyusb:
                n = self.usb_dev.write(self.usb_ep_out, data, timeout=200)
                self.get_logger().info(f"usb.write bytes={n} data={data.hex()}")
            else:
                # HID path without report id
                n = self.dev.write(data)
                self.get_logger().info(f"hid.write bytes={n} data={data.hex()}")
        except Exception as e:
            self.get_logger().warn(f"known payload send 실패: {e}")

    def _send_output(self, payload: bytes) -> None:
        if self.use_pyusb:
            self._usb_write(payload)
        else:
            self._hid_write(payload)

    def _hid_write(self, payload: bytes) -> None:
        # Build buffer with optional report id and optional padding
        def build_out(prefixed: bool):
            body = payload
            # According to vendor DLL prototype Usb_Qu_write(Q_index, Q_type, pQ_data[6])
            # many devices expect [index, type] then 6-byte data in report
            if self.prepend_index_type:
                body = bytes([self.q_index & 0xFF, self.q_type & 0xFF]) + body
            out = (bytes([self.hid_report_id]) + body) if (prefixed and self.hid_report_id >= 0) else body
            if self.pad_len and len(out) < self.pad_len:
                out = out + bytes([0x00] * (self.pad_len - len(out)))
            return out

        out1 = build_out(prefixed=True)
        out2 = build_out(prefixed=False)

        try:
            if self.use_feature and hasattr(self.dev, 'send_feature_report'):
                n = self.dev.send_feature_report(out1)
                self.get_logger().info(f"send_feature_report bytes={n} data={out1.hex()}")
                return
        except Exception as e:
            self.get_logger().warn(f"feature_report 실패: {e}")

        # Try write with report-id first
        try:
            n = self.dev.write(out1)
            self.get_logger().info(f"write bytes={n} data={out1.hex()}")
            return
        except Exception as e1:
            self.get_logger().warn(f"write(prefixed) 실패: {e1}")
        # Try write without report-id
        try:
            n = self.dev.write(out2)
            self.get_logger().info(f"write bytes={n} data={out2.hex()}")
        except Exception as e2:
            self.get_logger().warn(f"write(raw) 실패: {e2}")

    # ===== pyusb backend =====
    def _usb_open(self) -> None:
        if usb is None:
            self.get_logger().error("pyusb가 설치되어 있지 않습니다. 'pip install pyusb' 또는 'sudo apt install python3-usb' 후 재시도")
            raise RuntimeError('pyusb missing')
        dev = usb.core.find(idVendor=self.vid, idProduct=self.pid)
        if dev is None:
            raise RuntimeError('USB device not found')
        try:
            if dev.is_kernel_driver_active(0):
                try:
                    dev.detach_kernel_driver(0)
                except Exception:
                    pass
        except Exception:
            pass
        try:
            dev.set_configuration()
        except Exception:
            pass

        ep_out = None
        iface_num = None
        for cfg in dev:
            for intf in cfg:
                if self.pyusb_interface != -1 and intf.bInterfaceNumber != self.pyusb_interface:
                    continue
                if intf.bInterfaceClass == 3 or self.pyusb_interface != -1:
                    for ep in intf:
                        if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_OUT:
                            if self.pyusb_endpoint_out == -1 or ep.bEndpointAddress == self.pyusb_endpoint_out:
                                ep_out = ep.bEndpointAddress
                                iface_num = intf.bInterfaceNumber
                                break
                if ep_out is not None:
                    break
            if ep_out is not None:
                break
        if ep_out is None:
            raise RuntimeError('OUT endpoint not found')

        usb.util.claim_interface(dev, iface_num)
        self.usb_dev = dev
        self.usb_ep_out = ep_out
        self.usb_iface_num = iface_num
        self.get_logger().info(f"pyusb open OK iface={iface_num} ep_out=0x{ep_out:02x}")

    def _usb_write(self, payload: bytes) -> None:
        body = payload
        if self.prepend_index_type:
            body = bytes([self.q_index & 0xFF, self.q_type & 0xFF]) + body
        out = body
        total = self.pad_len if self.pad_len else 16
        if len(out) < total:
            out = out + bytes([0x00] * (total - len(out)))
        try:
            if self.pyusb_use_ctrl:
                # HID SET_REPORT(Output): bmRequestType=0x21, bRequest=0x09, wValue=(0x02<<8)|reportId
                report_id = 0 if self.hid_report_id < 0 else self.hid_report_id
                n = self.usb_dev.ctrl_transfer(0x21, 0x09, (0x02 << 8) | report_id, self.usb_iface_num, out, timeout=200)
                self.get_logger().info(f"usb.ctrl_transfer bytes={n} data={out.hex()}")
            else:
                n = self.usb_dev.write(self.usb_ep_out, out, timeout=200)
                self.get_logger().info(f"usb.write bytes={n} data={out.hex()}")
        except Exception as e:
            self.get_logger().warn(f"usb write 실패: {e}")

    # ===== helpers =====
    def _open_by_vid_pid(self) -> None:
        if hasattr(hid, 'Device'):
            self.dev = hid.Device(vid=self.vid, pid=self.pid)
        else:
            d = hid.device()
            d.open(self.vid, self.pid)
            self.dev = d

    def _open_by_path(self, path: str) -> None:
        # open using raw path
        if hasattr(hid, 'Device'):
            d = hid.Device(path=path.encode() if isinstance(path, str) else path)
            self.dev = d
        else:
            d = hid.device()
            d.open_path(path.encode() if isinstance(path, str) else path)
            self.dev = d

    def _enumerate_paths(self, vid: int, pid: int):
        try:
            infos = hid.enumerate(vid, pid)
            paths = []
            for info in infos:
                p = info.get('path') if isinstance(info, dict) else getattr(info, 'path', None)
                if p:
                    if isinstance(p, bytes):
                        p = p.decode(errors='ignore')
                    paths.append(p)
            return paths
        except Exception:
            return []


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TowerLampTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


