#include "OptidriveE3Spindle.h"

/*
    Grbl is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

namespace Spindles {
    OptidriveE3::OptidriveE3() : VFD(), _enabled(false), _reverse(false), _rpm(0) {
        _baudrate = 115200;
        _parity   = Uart::Parity::None;
    }

    void OptidriveE3::direction_command(SpindleState mode, ModbusCommand& data) {
        bool const enabled = (mode != SpindleState::Disable);
        bool const reverse = (mode == SpindleState::Ccw);
        setStateAndSpeed(data, enabled, reverse, _rpm);
    }

    void OptidriveE3::set_speed_command(uint32_t rpm, ModbusCommand& data) { setStateAndSpeed(data, _enabled, _reverse, rpm); }

    OptidriveE3::response_parser OptidriveE3::get_current_rpm(ModbusCommand& data) {
        // write multiple holding registers
        data.msg[1] = 0x03;
        // first register address is 0x0006, because registeres are 0 indexed
        data.msg[2] = 0x00;
        data.msg[3] = 0x06;
        // number of registers - 1
        data.msg[4] = 0x00;
        data.msg[5] = 0x01;

        data.tx_length = 6;
        data.rx_length = 5;

        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
            if (response[1] != 0x03)
                return false;
            if (response[2] != 2)
                return false;
            int16_t speed = response[3] << 8 | response[4];
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Warning, "GET SPEED: %d", (int)speed);
            auto rpm       = speed * 60 / 10;
            vfd->_sync_rpm = rpm;
            return true;
        };
    }

    OptidriveE3::response_parser OptidriveE3::get_status_ok(ModbusCommand& data) {
        // write multiple holding registers
        data.msg[1] = 0x03;
        // first register address is 0x0006, because registeres are 0 indexed
        data.msg[2] = 0x00;
        data.msg[3] = 0x05;
        // number of registers - 1
        data.msg[4] = 0x00;
        data.msg[5] = 0x01;

        data.tx_length = 6;
        data.rx_length = 5;

        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Warning, "GET OK: %d", (int)response[4]);

            if (response[1] != 0x03)
                return false;
            if (response[2] != 2)
                return false;
            auto statusCode = response[4];
            return statusCode != 2;
        };
    }

    bool OptidriveE3::supports_actual_rpm() const { return true; }

    bool OptidriveE3::safety_polling() const { return true; }

    void OptidriveE3::setStateAndSpeed(ModbusCommand& data, bool enabled, bool reverse, uint32_t rpm) {
        _enabled = enabled;
        _rpm     = rpm;
        _reverse = reverse;

        int32_t speedValue = rpm;
        if (reverse) {
            speedValue = -speedValue;
        }

        // convert to tenths of Hz, e.g. 12000 rpm = 200Hz => 2000
        speedValue = speedValue * 10 / 60;

        // write multiple holding registers
        data.msg[1] = 0x10;
        // first register address is 0x0000, because registeres are 0 indexed
        data.msg[2] = 0x00;
        data.msg[3] = 0x00;
        // number of registers - 2
        data.msg[4] = 0x00;
        data.msg[5] = 0x02;
        // number of bytes
        data.msg[6] = 0x04;

        // 1 - Drive Control Command
        data.msg[7] = 0x00;                    // high byte
        data.msg[8] = _enabled ? 0x01 : 0x00;  // low byte

        // 2 - Modbus Speed reference setpoint
        data.msg[9]  = (speedValue >> 8) & 0xFF;  // high byte
        data.msg[10] = (speedValue)&0xFF;         // low byte

        data.tx_length = 11;
        data.rx_length = 6;
    }
}
