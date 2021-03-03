#ifndef GPIOCONF_H
#define GPIOCONF_H

enum OutputList{
    none = -1,
    LED_Y = 3,
    LED_R1 = 11,
    LED_B,
    LED_G1 = 21,
    LED_EXCEPTION_THROWED = 28,
    LED_G2 = 32,
    TRIG = 36,
    SensorSS = 37,
    MemoryWriteProtect = 40,
    RadioCSN = 45,
    RadioCE,
};

enum InputList{
};

enum InterruptInputList{
  RadioIRQ = 0,
  ECHO,
  endOfInterruptInputList
};

enum ADClist{
	batteryCell1,
	batteryCell2,
	batteryCell3,
	temperature,
  end
};

#endif
