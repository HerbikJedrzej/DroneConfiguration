#ifndef GPIOCONF_H
#define GPIOCONF_H

enum OutputList{
    none = -1,
    MemoryWriteProtect = 40,
    RadioCSN = 1,
    RadioCE = 2,
    LED_Y1 = 19,
    LED_R1 = 20,
    LED_EXCEPTION_THROWED = 21,
    LED_G1 = 42,
    LED_G2 = 43,
    LED_Y2 = 44
};

enum InputList{
};

enum InterruptInputList{
  RadioIRQ = 0,
  MPUIRQ,
  endOfInterruptInputList
};

enum ADClist{
	battery,
  end
};

#endif
