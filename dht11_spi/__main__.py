'''
Usage:
======

$ python3 -m dht11_spi --help
usage: __main__.py [-h] [--gpio-chip GPIO_CHIP] [--dht22] [--time TIME] [--interval INTERVAL] [--spi SPI_BUS] [--spi-freq SPI_FREQ] [--rpi-lib] [--temp-f] gpio

Start a DHT11/22 test run using the dht11_spi library or the default RPi based library. Defaults to DHT11 unless --dht22 flag provided.

positional arguments:
  gpio                  GPIO to use for signaling the DHT sensor. If using GPIO_CHIP other than 0, set the "--gpio-chip x" option.

optional arguments:
  -h, --help            show this help message and exit
  --gpio-chip GPIO_CHIP
                        (default 0) GPIO chip for the GPIO provided. (0 typical for Pi4)
  --dht22               (default False) Marks the sensor as a DHT22 (different calculations used)
  --time TIME           (default 120) Time in seconds to run the test
  --interval INTERVAL   (default 1 for dht11, 2 for dht22) Interval between reads. 1sec min for DHT11, 2sec min for DHT22 (per spec)
  --spi SPI_BUS         (default 0) SPI Bus number to use (assumes kernel driver loaded and accessible by spidev)
  --spi-freq SPI_FREQ   (default 500000Hz) Frequence to run on the SPI Bus. Min tested is 250000Hz
  --rpi-lib             (default False) Use the RPi DHT11 library instead of the dht11_spi (for comparison)
  --temp-f              (default False) Print temps in F rather than C

$ python3 -m dht11_spi 

DHT11:
2023-03-03 20:44:48,315 - root - INFO - 105/105 (100.0%): Temps (min/avg/max): 23.0/24.03/24.1 deg CHumidity (min/avg/max): 19.0/19.01/20.0 %

DHT22:
2023-03-03 20:48:17,395 - root - INFO - 57/57 (100.0%): Temps (min/avg/max): 24.0/24.52/24.6 deg CHumidity (min/avg/max): 27.6/27.9/35.7 %


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
import argparse
from time import time, sleep
from . import DHT11_Spi, DhtReadings, SPI_BUS_HZ, SPI_MIN_BUS_HZ, DHT11_Exception
from logging_handler import create_logger

DEF_GPIO_CHIP = 0
DEF_TIME = 120
DEF_SPI_BUS = 0

DEF_DHT22 = False
DEF_DHT22_INTERVAL = 2
DEF_DHT11_INTERVAL = 1

DEF_RPI_LIB = False
DEF_TEMP_F = False


class data_reading:
    ''' Take a reading from the dht11 library using RPi.GPIO or DhtReadings, returns a standardized output ''' 
    def __init__(self, obj):
        self.data = obj
        self.dht_spi = isinstance(obj, DhtReadings)

    @property
    def humidity(self):
        if self.dht_spi:
            return self.data.humidity
        return round(self.data.humidity, 1) # treat as dht11 RPi library

    @property
    def temp_f(self):
        if self.dht_spi:
            return round(self.data.temp_f, 2) # treat as dht11 RPi library
        return round(self.temp_c * 9 / 5 + 32, 2)
    
    @property
    def temp_c(self):
        if self.dht_spi:
            return round(self.data.temp_c, 1)
        return self.data.temperature # treat as dht11 RPi library

    @property
    def error(self):
        ''' return true if the read is an error '''
        if self.dht_spi or self.data is None:
            return True if self.data is None else False
        return not self.data.is_valid()

    @property
    def error_msg(self):
        ''' return an error message if applicable '''
        return ('Error Code:' + str(self.data.error_code)) if (not self.dht_spi and self.data is not None) else ''

    def __str__(self):
        if self.dht_spi:
            return str(self.data)
        return f"Temp: {self.data.temp_c}degC ({self.data.temp_f}defF), humidity {self.humidity}"


def run_test(**kwargs):
    ''' Initiate the test using the provided arguments '''
    interval = (DEF_DHT11_INTERVAL if not kwargs.get('dht22', DEF_DHT22) else DEF_DHT22_INTERVAL) if kwargs.get('interval', None) is None else kwargs.get('interval')
    if kwargs.get('dht22', DEF_DHT22) and interval < DEF_DHT22_INTERVAL:
        raise DHT11_Exception(f"DHT22 requires a minimum of 2 second interval.")
    if not kwargs.get('rpi', DEF_RPI_LIB) and not kwargs.get('dht22', DEF_DHT22) and interval < DEF_DHT11_INTERVAL:
        raise DHT11_Exception(f"DHT11 requires a minimum of 1 second interval.")
    if kwargs.get('spi_freq', SPI_BUS_HZ) < SPI_MIN_BUS_HZ:
        raise DHT11_Exception(f"dht11_spi requires a minimum of {SPI_MIN_BUS_HZ}Hz for the SPI bus")
    logger = create_logger('info')

    # initiate the sensor
    if not kwargs.get('rpi', DEF_RPI_LIB):
        sensor = DHT11_Spi(spiBus=kwargs.get('spi_bus', DEF_SPI_BUS),
                spi_hz=kwargs.get('spi_freq', SPI_BUS_HZ),
                cs_pin=kwargs.get('gpio'),
                cs_chip=kwargs.get('gpio_chip', DEF_GPIO_CHIP),
                logger=logger,
                dht22=kwargs.get('dht22', DEF_DHT22)
            )
    else:
        # initialize using rpi library
        import RPi.GPIO as GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        import dht11
        sensor = dht11.DHT11(pin = kwargs.get('gpio'))

    # Run the test!
    data = []
    count = 0
    success = 0
    logger.info(f"Starting test.  Will run for {kwargs.get('time', DEF_TIME)} seconds")
    stop_time = time() + kwargs.get('time', DEF_TIME)
    try:
        while time() < stop_time:
            count += 1
            reading = data_reading(sensor.read())
            if not reading.error:
                success += 1
                logger.info(reading)
                data.append(reading)
            else:
                logger.warning('Failed reading! ' + reading.error_msg)
            if time() + interval < stop_time:
                sleep(interval)
            else:
                break
    except KeyboardInterrupt:
        print('Keyboard interrupt!')
    
    # calculate the min, avg and max
    if len(data) == 0 or success == 0:
        logger.error(f'No successful reads out of {count} attempts!')
        return
    temp_min = round(min([read.temp_f if kwargs.get('temp_f', DEF_TEMP_F) else read.temp_c for read in data]), 2) 
    temp_max = round(max([read.temp_f if kwargs.get('temp_f', DEF_TEMP_F) else read.temp_c for read in data]), 2) 
    temp_avg = round(sum([read.temp_f if kwargs.get('temp_f', DEF_TEMP_F) else read.temp_c for read in data]) / success, 2)
    humid_min = round(min([read.humidity for read in data]), 2) 
    humid_max = round(max([read.humidity for read in data]), 2) 
    humid_avg = round(sum([read.humidity for read in data]) / success, 2) 

    logger.info(f"{success}/{count} ({round(success/count, 4) * 100}%): Temps (min/avg/max): {temp_min}/{temp_avg}/{temp_max} deg {'F' if kwargs.get('temp_f', DEF_TEMP_F) else 'C'}, " \
            + f"Humidity (min/avg/max): {humid_min}/{humid_avg}/{humid_max} %")


if __name__ == '__main__':
    # setup the argument parser
    parser = argparse.ArgumentParser(description="Start a DHT11/22 test run using the dht11_spi library or the default RPi based library.  Defaults to DHT11 unless --dht22 flag provided.")
    parser.add_argument('gpio', metavar='gpio', type=int, help='GPIO to use for signaling the DHT sensor. If using GPIO_CHIP other than 0, set the "--gpio-chip x" option. ')
    parser.add_argument('--gpio-chip', dest='gpio_chip', required=False, type=int, default=DEF_GPIO_CHIP, help=f'(default {DEF_GPIO_CHIP}) GPIO chip for the GPIO provided. (0 typical for Pi4)')
    parser.add_argument('--dht22', dest='dht22', required=False, action='store_true', default=DEF_DHT22, help=f"(default {DEF_DHT22}) Marks the sensor as a DHT22 (different calculations used)")
    parser.add_argument('--time', dest='time', required=False, type=int, default=DEF_TIME, help=f"(default {DEF_TIME}) Time in seconds to run the test")
    parser.add_argument('--interval', dest='interval', required=False, type=int, default=None, help=f"(default {DEF_DHT11_INTERVAL} for dht11, {DEF_DHT22_INTERVAL} for dht22) Interval between reads.  1sec min for DHT11, 2sec min for DHT22 (per spec)")
    parser.add_argument('--spi', dest='spi_bus', required=False, type=int, default=DEF_SPI_BUS, help=f"(default {DEF_SPI_BUS}) SPI Bus number to use (assumes kernel driver loaded and accessible by spidev)")
    parser.add_argument('--spi-freq', dest='spi_freq', required=False, type=int, default=SPI_BUS_HZ, help=f"(default {SPI_BUS_HZ}Hz) Frequence to run on the SPI Bus.  Min tested is {SPI_MIN_BUS_HZ}Hz")
    parser.add_argument('--rpi-lib', dest='rpi', required=False, action='store_true', default=DEF_RPI_LIB, help=f"(default {DEF_RPI_LIB}) Use the RPi DHT11 library instead of the dht11_spi (for comparison)")
    parser.add_argument("--temp-f", dest='temp_f', required=False, action='store_true', default=DEF_TEMP_F, help=f"(default {DEF_TEMP_F}) Print temps in F rather than C")
    run_test(**vars(parser.parse_args()))