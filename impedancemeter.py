"""
=====================================
Tested on Python 3.9.2
Refer to "Datasheet" : AD5933
Refer to "Application Note" : 847 & 1252
=====================================
"""

from time import sleep
from smbus import SMBus

# ALWAYS LEFT TO RIGH <-> HIGH-BYTE TO LOW-BYTE

# REGISTRES
CTRL_REG = (0x80, 0x81)
STRT_FREQ_REG = (0x82, 0x83, 0x84)
FREQ_INCR_REG = (0x85, 0x86, 0x87)
NUMB_INCR_REG = (0x88, 0x89)
STTL_CYCL_REG = (0x8A, 0x8B)
STAT_REG = 0x8F
TEMP_DATA_REG = (0x92, 0x93)
REAL_DATA_REG = (0x94, 0x95)
IMAG_DATA_REG = (0x96, 0x97)

# MODES
INIT_START_FREQ = (0b1 << 4)
START_FREQ_SWEEP = (0b10 << 4)
INCR_FREQ = (0b0011 << 4)
REPEAT_FREQ = (0b100 << 4)
MEASURE_TEMP = (0b1001 << 4)
POWER_DOWN  = (0b1010 << 4)
STANDBY = (0b1011 << 4)

RESET = (0b1 << 4)
ZERO = 0b0

MAX_SWEEP_FREQ = 100000
MIN_SWEEP_FREQ = 1000

MAX_CLK_FREQ = 16776000
MIN_CLK_FREQ = 1

MAX_INC = 511
MIN_INC = 1 # 0 is doublon of 1

MAX_STTL_TIME = 511
MIN_STTL_TIME = 1 # 0 provides unreliable measures

FACTOR = {1 : (0b00 << 1), 2 : (0b01 << 1), 4 : 0b11 << 1} # Datasheet Table 11

OUT = {1 : (0b00 << 1), 4 : (0b01 << 1), 3 : (0b10 << 1), 2 : (0b11 << 1)} # Datasheet Table 10

PGA = {5 : 0b0, 1 : 0b1} # Datasheet Table 11

CLK = {0 : (0b0 << 3), 1 : (0b1 << 3)} # Datasheet Table 11

def hertz_to_3hexa(freq:int, clk:int) -> tuple:
    freq = int((freq / (clk / 4)) * (2 ** 27))
    return ((freq >> 16), (freq >> 8) & 255, freq & 255)

def uint_to_int(raw_number:int, resolution:int, divider:int=1):
    if raw_number > resolution / 2 - 1:
        raw_number -= resolution - 1
    return raw_number / divider

# RAISED IN CASE OF WRONG ARGUMENT

class AD5933Error(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)

class AD5933:
    def __init__(self, port=1, addr=0x0d) -> None:
        self.BUS = SMBus(port)
        self.ADDR = addr

        self.operation_mode = 0b0
        self.output_voltage = 0b0
        self.pga_gain = 0b0
        self.control_register0 = 0b0
        
        self.reset = 0b0
        self.system_clock = 0b0
        self.control_register1 = 0b0

        self.clock = 0
        self.start_frequency = 0

        self.flag_temperature = False
        self.flag_magnitude = False
        self.flag_sweep = False
        
        self.is_clock_set = False
        self.is_powered = False
        self.is_freq_init = False
        self.is_sweeping = False

    # PRIVATE METHODS
    
    def __read_i2c(self, target, tries=10) -> bytes:
        for _ in range(tries):
            try:
                return self.BUS.read_byte_data(self.ADDR, target)
            except OSError:
                print("Failed I2C read, trying again in 0.1 second...")
                sleep(0.1)
        print("Value of registry", target, "replaced by NaN")
        return float("NaN")
    def __write_i2c(self, target, value, tries=10) -> None:
        for t in range(tries):
            try:
                self.BUS.write_byte_data(self.ADDR, target, value)
                return
            except OSError:
                print("Failed I2C write, trying again in", t, "second...")
                sleep(tries)
        raise AD5933Error("I2C failure, 60s timeout, check connection quality")

    def __update_reg0(self) -> None:
        self.control_register0 = (self.operation_mode | self.output_voltage | self.pga_gain) 
        self.__write_i2c(CTRL_REG[0], self.control_register0)
        print("Set_Register0", bin(self.control_register0))

    def __update_reg1(self) -> None:
        self.control_register1 = (self.reset | self.system_clock)
        self.__write_i2c(CTRL_REG[1], self.control_register1)
        print("Set_Register1", bin(self.control_register1))

    def __operation(self, op_mode) -> None:
        self.operation_mode = op_mode
        self.__update_reg0()
        print("Set_Mode")

    # ALL THE SETTERS

    def Set_Frequency_Range(self, strt_freq:int, incr_freq:int, numb_incr:int) -> None:
        if not self.is_clock_set:
            raise AD5933Error("Didn't set system clock")
        if not(MIN_SWEEP_FREQ <= strt_freq <= MAX_SWEEP_FREQ):
            raise AD5933Error("Didn't specify start_freq within constructor's datasheet range")
        if not(0 <= incr_freq <= MAX_SWEEP_FREQ):
            raise AD5933Error("Didn't specify incr_freq within constructor's datasheet range")
        if not(MIN_INC <= numb_incr <= MAX_INC):
            raise AD5933Error("Didn't specify num_inc within constructor's datasheet range") 
        if not(strt_freq + incr_freq * numb_incr <= MAX_SWEEP_FREQ):
            raise AD5933Error("Didn't specify a valid combination of arguments")
        strt_hexa = hertz_to_3hexa(strt_freq, self.clock)
        self.__write_i2c(STRT_FREQ_REG[0], strt_hexa[0])
        self.__write_i2c(STRT_FREQ_REG[1], strt_hexa[1])
        self.__write_i2c(STRT_FREQ_REG[2], strt_hexa[2])
        incr_hexa = hertz_to_3hexa(incr_freq, self.clock)
        self.__write_i2c(FREQ_INCR_REG[0], incr_hexa[0])
        self.__write_i2c(FREQ_INCR_REG[1], incr_hexa[1])
        self.__write_i2c(FREQ_INCR_REG[2], incr_hexa[2])
        numb_hexa = ((numb_incr >> 8), numb_incr & 255)
        self.__write_i2c(NUMB_INCR_REG[0], numb_hexa[0])
        self.__write_i2c(NUMB_INCR_REG[1], numb_hexa[1])
        if self.start_frequency != strt_freq:
            self.start_frequency = strt_freq
            self.is_freq_init = False
        
    def Set_Number_Settling(self, nb_sttl:int, fact_key:int) -> None:
        if not MIN_STTL_TIME <= nb_sttl <= MAX_STTL_TIME:
            raise AD5933Error("Didn't specify nb_sttl within constructor's datasheet range")
        if not fact_key in FACTOR.keys():
            raise AD5933Error("Didn't specify fact_key within constructor's datasheet range")
        sttl_hexa = ((nb_sttl >> 8) | FACTOR[fact_key], nb_sttl & 255)
        self.__write_i2c(STTL_CYCL_REG[0], sttl_hexa[0])
        self.__write_i2c(STTL_CYCL_REG[1], sttl_hexa[1])
        print("Set_Number_Settling")

    def Set_Output_Voltage_Range(self, out_key:int) -> None:
        if not out_key in OUT.keys():
            raise AD5933Error("Didn't specify out_key within constructor's datasheet range")
        self.output_voltage = OUT[out_key]
        self.__update_reg0()
        print("Set_Output_Voltage_Range")

    def Set_PGA_Gain(self, pga_key:int) -> None:
        if not pga_key in PGA.keys():
            raise AD5933Error("Didn't specify pga_key within constructor's datasheet range")
        self.pga_gain = PGA[pga_key]
        self.__update_reg0()
        print("Set_PGA_Gain")

    def Set_Reset(self) -> None:
        self.reset = RESET
        self.__update_reg1()
        self.reset = ZERO
        self.flag_temperature = self.flag_sweep = self.flag_magnitude = self.is_sweeping = False
        #self.is_powered = True
        print("Set_Reset")

    def Set_System_Clock(self, clk_key:int, clk_freq:int) -> None:
        if not clk_key in CLK.keys():
            raise AD5933Error("Didn't specify clk_key within constructor's datasheet range")
        if not MIN_CLK_FREQ <= clk_freq <= MAX_CLK_FREQ:
            raise AD5933Error("Didn't specify clk_freq within constructor's datasheet range")
        self.system_clock = CLK[clk_key]
        self.clock = clk_freq
        self.is_clock_set = True
        self.__update_reg1()
        print("Set_System_Clock")

    # ALL THE FLAGS (before calling the getter)

    def is_temperature_flagged(self) -> bool:
        self.flag_temperature = (self.__read_i2c(STAT_REG) & 1) == 1
        return self.flag_temperature

    def is_magnitude_flagged(self) -> bool:
        self.flag_magnitude = (self.__read_i2c(STAT_REG) & 2) == 2
        return self.flag_magnitude

    def is_sweep_flagged(self) -> bool:
        self.flag_sweep = (self.__read_i2c(STAT_REG) & 4) == 4
        if self.flag_sweep:
            self.is_sweeping = False
        return self.flag_sweep

    # ALL THE GETTERS (after calling the flag)

    def Get_Magnitude(self) -> tuple:
        if not self.flag_magnitude:
            raise AD5933Error("Didn't check magnitude's flag according to procedure")
        real = (self.__read_i2c(REAL_DATA_REG[0]) << 8) | self.__read_i2c(REAL_DATA_REG[1])
        imag = (self.__read_i2c(IMAG_DATA_REG[0]) << 8) | self.__read_i2c(IMAG_DATA_REG[1])
        self.flag_magnitude = False
        return uint_to_int(real, 65536), uint_to_int(imag, 65536)

    def Get_Temperature(self) -> int:
        if not self.flag_temperature:
            raise AD5933Error("Didn't check temperature's flag according to procedure")
        temp = (self.__read_i2c(TEMP_DATA_REG[0]) << 8) | self.__read_i2c(TEMP_DATA_REG[1])
        self.flag_temperature = False
        return uint_to_int(temp, 16384, 32)

    # ALL THE COMMANDS

    def Initialize_Start_Frequency(self) -> None:
        self.__operation(INIT_START_FREQ)
        self.is_freq_init = True

    def Start_Frequency_Sweep(self) -> None:
        if not self.is_powered:
            raise AD5933Error("Didn't program standby mode, VOUT and VIN internally connected to GND")
        if not self.is_freq_init:
            raise AD5933Error("Didn't initialize start frequency or forgot to do it again if the value was changed")
        if self.is_sweeping:
            raise AD5933Error("There's an ongoing frequency sweep... issue a reset or wait till the end (check doublon error)")
        self.__operation(START_FREQ_SWEEP)
        self.flag_sweep = self.flag_magnitude = False
        self.is_sweeping = True

    def Increment_Frequency(self) -> None:
        if not self.is_powered:
            raise AD5933Error("Didn't program standby mode, VOUT and VIN internally connected to GND")
        if not self.is_sweeping:
            raise AD5933Error("There's no ongoing frequency sweep : forgot to start, issued reset or sweep ended")
        if self.flag_sweep:
            raise AD5933Error("Issued reset or sweep ended, might be an accidental last call (check loops error)")
        self.__operation(INCR_FREQ)
        self.flag_magnitude = False

    def Repeat_Frequency(self) -> None:
        if not self.is_powered:
            raise AD5933Error("Didn't program standby mode, VOUT and VIN internally connected to GND")
        if not self.is_sweeping:
            raise AD5933Error("There's no ongoing frequency sweep... forgot to start, reset issued or sweep ended")
        self.__operation(REPEAT_FREQ)
        self.flag_magnitude = False

    def Measure_Temperature(self) -> None:
        self.__operation(MEASURE_TEMP)
        self.flag_temperature = False

    def Power_Down_Mode(self) -> None:
        self.__operation(POWER_DOWN)
        self.is_powered = False

    def Standby_Mode(self) -> None:
        self.__operation(STANDBY)
        self.is_powered = True

    # CLEAN EXIT SYSTEM

    def clean(self) -> None:
        self.Power_Down_Mode()
        self.BUS.close()
