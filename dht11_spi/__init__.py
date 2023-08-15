'''
PyPi Package: dht11_spi
Homepage: https://www.learningtopi.com/python-modules-applications/dht11_spi/
Git: https://github.com/LearningToPi/dht11_spi

Description:
DHT11/DHT22 python3 driver that uses the SPI to receive binary data from the
temp/humidity sensor.  This driver uses a GPIO pin attached to a NPN transistor
to send the low signal required to start the communication.

General GPIO Pin (NOT an SPI CS) used to pull the signal pin low.  The DHT11/DHT22
will then send its binary data by pulling the data pin low.  SPI bus reads the 
high and low data and determins the bits based on the time the sensor holds the
voltage high.  Data is converted to temperature and humidity.

NOTE:  Requires SPI bus to run at a minimum of 125kHz to properly read the data.
  Only SPI0 on RPI4 functions properly.  SPI1+ is not able to read data.

Parts:
  - DHT11 / DHT22 (DHT11: https://amzn.to/3DCEH0W, DHT22: https://amzn.to/3Y2BTm5)
  - NPN 2N5551 transistor (https://amzn.to/3YiM59Y)
  - 1x 470ohm resistor (https://amzn.to/3HvG6I6)
  - 1x 1k ohm resistor\___use for 5v to 3.3v level shift if board is 3.3v 
  - 1x 2k ohm resistor/   or use a level shifter (https://amzn.to/3X0jdSL)
  - Linux SBC

Cabling:

         -------------------------------------------                    
         |                                         |                    
         |                        5v               |                    
         |                         +               |                    
        .-.                        |               |                    
        2k|                        |               |                    
        | |                        |               |                    
        '-'     ___                |               |                    
         |-----|1k_|----           |               |                    
         |             |           |               |                    
         |             |           o##########     |                    
         |             |            ##########     |                    
SPI MISO o             |-----------o#DHT11/22#     |                    
               ___   |/             ##########     |                    
    GPIO o----|470|--|        |----o##########     |                    
                     |>       |                    |                    
         o     2N5551  |      |                    |                    
                       |      |                    |                    
     GND o-----------------------------------------|                    
                                                                        
(created by AACircuit.py © 2020 JvO)

Usage:
======

# Import DHT SPI class
from dht11_spi import DHT11_Spi, DHT22_Spi

# initialize device, use (DHT11_Spi or DHT22_Spi)
# cs_chip and cs_pin from "gpioinfo".  gpiod used for platform compatibility.
dht = DHT22_Spi(spiBus=0, cs_chip=0, cs_pin=26)
reading = dht22.read() # returns instance of DhtReadings
print(reading)
# --or--
print(reading.temp_c, reading.temp_f, reading.humidity)


DHT11: 105/105 (100.0%): Temps (min/avg/max): 73.54/75.2/75.34 deg FHumidity (min/avg/max): 17.0/17.0/17.0 %
DHT22: 112/112 (100.0%): Temps (min/avg/max): 74.48/74.51/74.66 deg FHumidity (min/avg/max): 14.1/14.21/16.0 %


MIT License

Copyright (c) 2022 LearningToPi <contact@learningtopi.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

'''

import spidev
import logging_handler
from threading import Lock, Thread
from time import sleep
from math import ceil, floor


DEBUG = False

SPI_BUS_HZ = 500_000
SPI_MIN_BUS_HZ = 250_000
DHT_INIT_SIGNAL_MS   = 20

DHT_SIG_0 = (20, 28)
DHT_SIG_LOW = (46, 56)
DHT_SIG_1 = (66, 74)
DHT_SIG_START = (76,90)

DHT11_MIN_INTERVAL = 1
DHT22_MIN_INTERVAL = 2

BIT_VALS = (128, 64, 32, 16, 8, 4, 2, 1)

class DHT11_Exception(Exception):
    pass

# Attempt import of different gpio libraries
try:
    import gpiod
    GPIO_LIBRARY = 'gpiod'
except:
    raise DHT11_Exception("Could not load gpio library.  Supported libraries: gpiod")


class DhtReadings:
    ''' Class to represent a reading performed on BME280 or BMP280 sensor '''
    def __init__(self, temp, humidity):
        self.temp = temp
        self.humidity = humidity

    def __str__(self):
        ''' Return the class as a printable string '''
        return f"Temp: {self.temp_c}degC ({self.temp_f}degF), Humidity {self.humidity}%"

    @property
    def temp_c(self):
        ''' Return temp in degrees celcius '''
        return self.temp

    @property
    def temp_f(self):
        ''' return temp in degrees Fahrenheit '''
        return round(self.temp * 9 / 5 + 32, 4)


class DHT11_Spi:
    ''' Class to represent a DHT sensor connectefd to an SPI bus using.  SPI bus will be configured for 3-wire mode for half-duplex communication '''
    def __init__(self, spiBus:int, cs_pin:int, cs_chip=None, spi_hz=SPI_BUS_HZ, set_spi_hz=True, logger=None, dht22=False, cs=0):
        self.spi_bus_hz = spi_hz
        self.spi_clock_us = 1 / self.spi_bus_hz * 1_000_000
        self.init_ms = DHT_INIT_SIGNAL_MS
        self._data = []
        self._bit_counts = []
        self._binary_data = []
        self.discard_count = 0
        self.dht22 = dht22
        self.min_interval = DHT22_MIN_INTERVAL if dht22 else DHT11_MIN_INTERVAL
        
        self._read_lock = Lock()
        self._log_extra = f"SPI{spiBus}-{self.spi_bus_hz/1000}kHz"
        self._logger = logger if logger is not None else logging_handler.create_logger('DEBUG' if DEBUG else 'INFO', name=__name__)
        self._logger.info(f"{self.info_str}: Opening")
        self._spi = spidev.SpiDev()
        self._spi.open(spiBus, cs)

        # check and lower or raise the speed if needed
        if self._spi.max_speed_hz != self.spi_bus_hz and set_spi_hz:
            self._logger.info(f"{self.info_str}: Setting SPI bus speed from {self._spi.max_speed_hz} to {self.spi_bus_hz}")
            self._spi.max_speed_hz = self.spi_bus_hz
        if self._spi.max_speed_hz != self.spi_bus_hz:
            raise DHT11_Exception(f'SPI Bus {spiBus} Speed is: {self._spi.max_speed_hz} must be {self.spi_bus_hz}')

        # Create Soft CS
        if GPIO_LIBRARY == 'gpiod':
            self._cs = gpio_SPI_Cs(chip=cs_chip, pin=cs_pin, logger=self._logger)
        else:
            raise DHT11_Exception('Unable to open CS. GPIO Library error.')

    def __del__(self):
        self.close()

    def close(self):
        self._spi.close()
        self._cs.close()

    @property
    def info_str(self):
        ''' Returns the info string for the class (used in logging commands) '''
        class_name = __class__.__name__ if not self.dht22 else str(__class__.__name__).replace('DHT11', 'DHT22')
        return f"{class_name} ({self._log_extra})"

    def read(self):
        ''' Read the DHT device '''
        self._stop_read_thread = False
        with self._read_lock:
            self._logger.debug(f"{self.info_str}: Starting read...")
            thread1 = Thread(target=self._read_thread)
            self._cs.high()
            thread1.start()
            sleep(.02)
            self._cs.low()
            sleep(.01)
            self._stop_read_thread = True
            # wait for background read to complete
            thread1.join(timeout=5)
            
            # calculate temp and humidity
            data = self._process_data()
        return data

    def _read_thread(self):
        ''' Background thread to read the SPI data '''
        self._data = self._spi.xfer2([0] * 4096)

    def _process_data(self):
        ''' Take the raw data read from the SPI bus and convert to a usable format '''
        self._logger.debug(f"{self.info_str}: Starting process of {len(self._data)} bytes...")
        # data should start at 0 since the soft CS pin is set to high, if not return an error
        if self._data[0] != 0:
            raise DHT11_Exception(f"Data received does not start with 0's. If the soft-CS pin is set to high the NPN transistor should cause a read of 0.  Data (first 20 btes): {self._data[0:20]}")
        # loop until we reach the end of the data
        pos = 0 # where we are in the data
        last_bit = 0 # what is the current bit we are couting
        self._bit_counts = [0]
        while pos < len(self._data):
            for bit in range(len(BIT_VALS)):
                if (self._data[pos] & BIT_VALS[bit]) >> (7 - bit) == last_bit:
                    # matching bit, increment the count
                    self._bit_counts[len(self._bit_counts)-1] += 1
                else:
                    # bit didn't match, start a new count
                    last_bit = (self._data[pos] & BIT_VALS[bit]) >> (7 - bit)
                    self._bit_counts.append(1)
            pos += 1

        # convert the bit counts into binary data
        bus_factor = self._spi.max_speed_hz / 1_000_000 # scale signal counts up or down
        pos = 0 # where we are in the bit count list
        self._binary_data = []
        while pos < len(self._bit_counts):
            if floor(DHT_SIG_0[0] * bus_factor) > self._bit_counts[pos]:
                self._binary_data.append(f'ERR-LOW:{self._bit_counts[pos]}')
            elif floor(DHT_SIG_0[0] * bus_factor) <= self._bit_counts[pos] <= ceil(DHT_SIG_0[1] * bus_factor):
                self._binary_data.append('0')
            elif floor(DHT_SIG_0[1] * bus_factor) < self._bit_counts[pos] < ceil(DHT_SIG_LOW[0] * bus_factor): # LOW is the 0 level between bits
                self._binary_data.append(f'ERR-BTW-0-LOW:{self._bit_counts[pos]}')
            elif floor(DHT_SIG_LOW[1] * bus_factor) < self._bit_counts[pos] < ceil(DHT_SIG_1[0] * bus_factor):
                self._binary_data.append(f'ERR-BTW-LOW-1:{self._bit_counts[pos]}')
            elif floor(DHT_SIG_LOW[0] * bus_factor) <= self._bit_counts[pos] <= ceil(DHT_SIG_LOW[1] * bus_factor):
                pass    # 0 level between bits, just skip it
            elif floor(DHT_SIG_1[0] * bus_factor) <= self._bit_counts[pos] <= ceil(DHT_SIG_1[1] * bus_factor):
                self._binary_data.append('1')
            elif floor(DHT_SIG_1[1] * bus_factor) < self._bit_counts[pos] < ceil(DHT_SIG_START[0] * bus_factor):
                self._binary_data.append(f'ERR-BTW-1-S:{self._bit_counts[pos]}')
            elif floor(DHT_SIG_START[0] * bus_factor) <= self._bit_counts[pos] <= ceil(DHT_SIG_START[1] * bus_factor):
                self._binary_data.append(f'INIT:{pos%2}') # print 0 or 1 if it is even or odd
            else:
                self._binary_data.append(f'ERR-HIGH:{self._bit_counts[pos]}')
            pos += 1

        # verify we have 40 binary values
        binary_values = 0
        first_bin_value = 255 # set a high value and drop down if we found it
        init_set = -1 # Flag to ensure we start counting after the INIT:0 & INIT:1
        for x in range(len(self._binary_data)):
            if self._binary_data[x] == 'INIT:0' or self._binary_data[x] == 'INIT:1':
                init_set += 1
            elif (self._binary_data[x] == '0' or self._binary_data[x] == '1') and init_set == 1:
                binary_values += 1
                if x < first_bin_value:
                    first_bin_value = x
        if binary_values != 40:
            self._logger.error(f"{self.info_str}: Received {binary_values}, expected 40. Data discarded.")
            self.discard_count += 1
            return

        # get humidity high and low, temp high and low, checksum
        hh_b = ''.join(self._binary_data[first_bin_value:first_bin_value+8])
        hl_b = ''.join(self._binary_data[first_bin_value+8:first_bin_value+16])
        th_b = ''.join(self._binary_data[first_bin_value+16:first_bin_value+24])
        tl_b = ''.join(self._binary_data[first_bin_value+24:first_bin_value+32])
        c_b = ''.join(self._binary_data[first_bin_value+32:first_bin_value+40])
        self._logger.debug(f"{self.info_str}: Binary Data: Humidity {hh_b}.{hl_b}, Temp: {th_b}.{tl_b}, Checksum: {c_b}")

        # convert to integers
        hh = int(hh_b, 2)
        hl = int(hl_b, 2)
        th = int(th_b, 2)
        tl = int(tl_b, 2)
        c = int(c_b, 2)
        c_calc = (hh+hl+th+tl) & ((1 << 8) - 1)
        humidity = (hh + hl/100) if not self.dht22 else unsigned_short([int(hh_b,2), int(hl_b,2)]) / 10.0
        temp = (th + tl/100) if not self.dht22 else signed_short([int(th_b,2), int(tl_b,2)]) / 10.0

        if c_calc != c:   # take only last 8 bits
            self._logger.error(f"{self.info_str}: Checksum failed: Humidity {humidity}, temp {temp}, Checksum: {c} calculated {c_calc}")
        else:
            self._logger.debug(f"{self.info_str}: Checksum ok: Humidity {humidity}, temp {temp}, Checksum: {c} calculated {c_calc}")

        return DhtReadings(temp=temp, humidity=humidity)

class gpio_SPI_Cs:
    ''' Class to represent a soft CS pin for the SPI bus (separate from the hardware CS pins) 
        per: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#spi-software
        the spi-bcm2835 Linux SPI driver does not use the hardware CS lines!
    '''
    def __init__(self, chip, pin, logger):
        chip = gpiod.chip(str(chip), gpiod.chip.OPEN_BY_NUMBER)
        self._pin = chip.get_line(int(pin))
        pin_config = gpiod.line_request()
        pin_config.consumer = 'SPI_Soft_CS'
        pin_config.request_type = gpiod.line_request.DIRECTION_OUTPUT
        self._pin.request(pin_config)

        try:
            self._pin.set_flags(gpiod.line_request.FLAG_BIAS_DISABLE)
        except Exception as e:
            logger.warning(f"Unable to set pull. Platform may not be capable. Error: {e}")

        self.low()

    def __del__(self):
        self.close()

    def close(self):
        self._pin.release()

    @property
    def state(self):
        ''' Return current CS state '''
        return self._pin.get_value()

    def high(self):
        ''' Set the CS to on '''
        self._pin.set_value(1)
        
    def low(self):
        ''' Set the CS to off '''
        self._pin.set_value(0)


class DHT22_Spi(DHT11_Spi):
    ''' Class to represent a DHT22 with alternate temp calculations ''' 
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs, dht22=True)


def unsigned_short(registers:list, reverse=False) -> int:
    ''' Takes a pair of registers and returns an unsigned short (16bit) int. If reverse, registers are read in reverse order ''' 
    reg = registers
    if reverse:
        reg = registers.copy()
        reg.reverse()
    if len(reg) != 2:
        raise DHT11_Exception(f'Unable to convert list of registers to an unsigned short. Received {reg}.  Requires list of 2.')
    return (reg[0] << 8 ) | reg[1]


def signed_short(registers:list, reverse=False) -> int:
    ''' Takes a pair of registers and returns a signed short (16bit) int. If reverse, registers are read in reverse order '''
    reg = registers
    if reverse:
        reg = registers.copy()
        reg.reverse()
    if len(reg) != 2:
        raise DHT11_Exception(f'Unable to convert list of registers to an signed short. Received {reg}.  Requires list of 2.')
    unsigned = unsigned_short(reg)
    if unsigned & (1 << (16-1)):
        unsigned -= 1 << 16
    return unsigned