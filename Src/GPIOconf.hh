#ifndef GPIOCONF_H
#define GPIOCONF_H

enum OutputList{
    none = -1,
    LED_R = 6,
    LED_Y1 = 7,
    RadioCE2 = 12,
    RadioCSN2 = 20,
    LED_B = 28,
    LED_G1 = 31,
    RadioCSN1 = 32,
    RadioCE1 = 33,
    LED_Y2 = 36,
    MemoryWriteProtect = 40,
    LED_G2 = 45,
    LED_IR = 47,
};

enum InputList{
  MPU6050IRQ = 21,
};

enum InterruptInputList{
  RadioIRQ1 = 0,
  RadioIRQ2,
  BarometerIRQ,
  endOfInterruptInputList
};

enum ADClist{
	batteryCell1,
	batteryCell2,
	landingSensor,
  end
};

#endif
