
#!/usr/bin/env python3
"""
This file holds the ADC class which is used within HX711 in order to track multiple ADCs
"""

import RPi.GPIO as GPIO
#from GPIOEmulator.EmulatorGUI import GPIO
from time import sleep, perf_counter
from statistics import mean, median, stdev
from utils import convert_to_list
from logging import getLogger, Logger, StreamHandler



class ADC:
    """
    ADC class holds data for one hx711 ADC

    Args:
        dout_pin (int): Raspberry Pi GPIO pin where data from HX711 is received
        logger (logger): logger from main class to use for logging

    Attrs:
        _dout_pin (int):            gpio pin for read
        _logger (logger):           logger from main HX711 class
        _zero_offset (float):       offset set after performing a zero read
        _weight_multiple (float):   multiple to convert from raw measurement to real world value
        _ready (bool):              bool for checking sensor ready
        _current_raw_read (int):    current raw read from binary bit read
        raw_reads ([int]):          raw reads from binary bit read as 2s complement from ADC
        reads ([signed int])        raw reads after convert to signed integer
        _max_stdev (int):           max standard deviation value of raw reads (future todo: expose for user input? Does this vary per hardware?)
        _reads_filtered ([int])     filtered reads after removing failed reads and bad datapoints
        _max_number_of_stdev_from_med (float): maximium number of deviations from median (future todo: expose for user input?)
        _read_med (float)           median of reads
        _devs_from_med ([float]):   deviations of reads from median
        _read_stdev (float)         st dev of reads
        _ratios_to_stdev ([float]): ratios of dev from med of each read to the st dev of all the data
        measurement (float):        mean value of raw reads after filtering
        measurement_from_zero (float): measurement minus offset
        weight (float):             measurement_from_zero divided by weight_multiple
    """

    def __init__(self,
                 dout_pin: int,
                 logger: Logger,
                 ):
        self._dout_pin = dout_pin
        self._logger = logger
        self._zero_offset = 0.
        self._weight_multiple = 1.
        self._ready = False
        self._current_raw_read = 0
        self.raw_reads = []
        self.reads = []
        self._max_stdev = 100
        self._reads_filtered = []
        self._max_number_of_stdev_from_med = 2.0
        self._read_med = None
        self._devs_from_med = []
        self._read_stdev = 0.
        self._ratios_to_stdev = []
        self.measurement = None
        self.measurement_from_zero = None
        self.weight = None

    def zero_from_last_measurement(self):
        """ sets offset based on current value for measurement """
        if self.measurement:
            self.zero(self.measurement)
        else:
            raise ValueError(f'Trying to zero ADC (dout={self._dout_pin}) with a bad mean value. '
                             f'Value of measurement: {self.measurement}')

    def zero(self, offset: float = None):
        """ sets offset based on current value for measurement """
        if offset is not None:
            self._zero_offset = offset
        else:
            raise ValueError(f'No offset provided to zero() function')

    def get_zero_offset(self):
        return self._zero_offset

    def set_weight_multiple(self, weight_multiple: float):
        """ simply sets multiple. example: scale indicates value of 5000 for 1 gram on scale, weight_multiple = 5000 """
        self._weight_multiple = weight_multiple

    def _init_set_of_reads(self):
        """ init arrays and calculated values before beginning a set of reads for a measurement """
        self.raw_reads = []
        self.reads = []
        self._reads_filtered = []
        self._read_med = None
        self._devs_from_med = []
        self._read_stdev = 0.
        self._ratios_to_stdev = []
        self.measurement = None
        self.measurement_from_zero = None
        # self.weight = None ## don't overwrite weight so we can keep using older weight value
        self._init_raw_read()

    def _init_raw_read(self):
        """ set raw read value to zero, so each bit can be shifted into this value """
        self._ready = False
        self._current_raw_read = 0

    def _is_ready(self):
        """ return True if already _ready or GPIO input is zero """
        if self._ready:
            return True
        else:
            self._ready = (GPIO.input(self._dout_pin) == 0)
            return self._ready

    def _shift_and_read(self):
        """ left shift by one bit then bitwise OR with the new bit """
        self._current_raw_read = (
            self._current_raw_read << 1) | GPIO.input(self._dout_pin)

    def _finish_raw_read(self):
        """ append current raw read value and signed value to raw_reads list and reads list """
        self.raw_reads.append(self._current_raw_read)
        # convert to signed value
        self._current_signed_value = self.convert_to_signed_value(
            self._current_raw_read)
        self.reads.append(self._current_signed_value)
        # log 2's complement value and signed value
        self._logger.debug(
            f'Binary value: {bin(self._current_raw_read)} -> Signed: {self._current_signed_value}')

    def convert_to_signed_value(self, raw_value):
        # convert to signed value after verifying value is valid
        # raise error if value is exactly the min or max value, or a value of all 1's
        if raw_value in [0x800000, 0x7FFFFF, 0xFFFFFF]:
            self._logger.debug(
                'Invalid raw value detected: {}'.format(hex(raw_value)))
            return None  # return None because the data is invalid
        # calculate int from 2's complement
        # check if the sign bit is 1, indicating a negative number
        if (raw_value & 0x800000):
            # convert from 2's complement to negative int
            signed_value = -((raw_value ^ 0xffffff) + 1)
        else:  # else do not do anything the value is positive number
            signed_value = raw_value
        return signed_value

    def _calculate_measurement(self):
        """
        analyzes read values to calculate mean value
            1) filter by valid data only
            2) calculate median and standard deviations from median
            3) filter based on the standard deviations from the median
            4) calculate mean of remaining values

        Returns:
            bool: pass or fail boolean based on filtering of data
        """

        # filter reads to valid data only
        self._reads_filtered = [r for r in self.reads if (
            (r is not None) and (type(r) is int))]
        if not len(self._reads_filtered):
            # no values after filter, so return False to indicate no read value
            return False
        elif len(self._reads_filtered) == 1:
            self.measurement = self._reads_filtered[0]
            return True

        # get median and deviations from med
        self._read_med = median(self._reads_filtered)
        self._devs_from_med = [(abs(r - self._read_med))
                               for r in self._reads_filtered]
        self._read_stdev = stdev(self._devs_from_med)

        # filter by number of standard deviations from med
        if self._read_stdev > self._max_stdev:
            # if standard deviation is too large, the scale isn't actually ready
            # sometimes with a bad scale connection, the bit will come back ready out of chance and the binary values are garbage data
            self._ready = False
            self._logger.warn(
                f'ADC (dout {self._dout_pin}) not ready, stdev from median was over {self._max_stdev}: {self._read_stdev}')
        elif self._read_stdev:
            self._ratios_to_stdev = [(dev / self._read_stdev)
                                     for dev in self._devs_from_med]
        else:
            # stdev is 0. Therefore set to the median
            self.measurement = self._read_med
            return True
        _new_reads_filtered = []
        for (read_val, ratio) in zip(self._reads_filtered, self._ratios_to_stdev):
            if ratio <= self._max_number_of_stdev_from_med:
                _new_reads_filtered.append(read_val)
        self._reads_filtered = _new_reads_filtered

        # get mean value
        if not self._reads_filtered:
            # no values after filter, so return False to indicate no read value
            return False
        self.measurement = mean(self._reads_filtered)
        self.measurement_from_zero = self.measurement - self._zero_offset
        self.weight = self.measurement_from_zero / self._weight_multiple

        return True
