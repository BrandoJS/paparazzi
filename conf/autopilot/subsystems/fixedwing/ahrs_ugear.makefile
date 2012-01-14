# attitude estimation for fixedwings via GX3

ifndef UGEAR_PORT
  UGEAR_PORT=UART1
endif
ifndef UGEAR_BAUD
  UGEAR_BAUD=B115200
endif


$(TARGET).CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_ugear.h\"

ifeq ($(TARGET), ap)

ap.CFLAGS += -DUSE_UGEAR
ap.CFLAGS += -DUSE_AHRS
ap.CFLAGS += -DAHRS_UPDATE_FW_ESTIMATOR

ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_ugear.c
ap.srcs   += $(SRC_SUBSYSTEMS)/imu.c


ifdef CPU_LED
  ap.CFLAGS += -DAHRS_CPU_LED=$(CPU_LED)
endif

ap.CFLAGS += -DUSE_$(UGEAR_PORT) -D$(UGEAR_PORT)_BAUD=$(UGEAR_BAUD)
ap.CFLAGS += -DUGEAR_LINK=$(UGEAR_PORT) 

endif



