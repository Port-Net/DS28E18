
# Arduino Library for DS28E18 Integrated Circuits

## Usage

Please include OneWire from Paul Stoffregen in the library manager before you begin.

## Credits

The structure has been derived from
https://github.com/milesburton/Arduino-Temperature-Control-Library developt by Miles Burton <miles@mnetcs.com> 

# Basic function

on begin() the lib checks if there are DS28E18 on the bus that need to get initialized.
After initializing status is read to clear PowerOnReset flag.
- to use the IC you have to provide a sequence of I2C or SPI commands and load them into the SRAM. This has to be done every time after power loss.
- Please refer to datasheet at https://www.analog.com/en/products/ds28e18.html
Sequence is loaded with load_sequence(), executed with run_sequence() and results are fetched with read_sequence().

There is support for honeywell preasure sensor MPR series built in. Functions are:
- MPR_sensor_init()
- MPR_sensor_measure_result()

currently only tested on ESP32 with PSU connected (no strong pullup)

# License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
