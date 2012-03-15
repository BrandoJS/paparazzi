#include "boards/tiny_2.1.h"

#ifndef CONFIG_TWOG_H
#define CONFIG_TWOG_H

#undef DefaultVoltageOfAdc

#ifdef SixCellBatt
//#warning "Using a 6S battery" 0.02926638911
#define DefaultVoltageOfAdc(adc) (0.02936638911*adc)
#else
#define DefaultVoltageOfAdc(adc) (0.0247311828*adc)
//#warning "Using < 6S battery"
#endif






#endif /* CONFIG_TWOG_H */
