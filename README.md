# DHT11 / DHT22 over SPI

Homepage: https://www.learningtopi.com/python-modules-applications/dht11_spi/


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

## Usage:


    # Import DHT SPI class
    from dht11_spi import DHT11_Spi, DHT22_Spi

    # initialize device, use (DHT11_Spi or DHT22_Spi)
    # cs_chip and cs_pin from "gpioinfo".  gpiod used for platform compatibility.
    dht = DHT22_Spi(spiBus=0, cs_chip=0, cs_pin=26)
    reading = dht22.read() # returns instance of DhtReadings
    print(reading)
    # --or--
    print(reading.temp_c, reading.temp_f, reading.humidity)

## Testing
Included in the module is a basic test script that can be executed with the following:

    python3 -m dht11_spi [gpio]

Additional test options are available for interval, run time, dht22.  Documentation is available using the "--help" option.

### Example Output

    DHT11: 105/105 (100.0%): Temps (min/avg/max): 73.54/75.2/75.34 deg FHumidity (min/avg/max): 17.0/17.0/17.0 %
    DHT22: 112/112 (100.0%): Temps (min/avg/max): 74.48/74.51/74.66 deg FHumidity (min/avg/max): 14.1/14.21/16.0 %
