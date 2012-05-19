# attitude estimation for fixedwings via GX3

ifndef GX3_PORT
  GX3_PORT=UART3
endif
ifndef GX3_BAUD
  GX3_BAUD=B115200
endif


$(TARGET).CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_gx3.h\"

ifeq ($(TARGET), ap)

ap.CFLAGS += -DUSE_GX3
ap.CFLAGS += -DUSE_AHRS
ap.CFLAGS += -DAHRS_UPDATE_FW_ESTIMATOR

ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_gx3.c
ap.srcs   += $(SRC_SUBSYSTEMS)/imu.c


ifdef CPU_LED
  ap.CFLAGS += -DAHRS_CPU_LED=$(CPU_LED)
endif

ap.CFLAGS += -DUSE_$(GX3_PORT) -D$(GX3_PORT)_BAUD=$(GX3_BAUD)
ap.CFLAGS += -DUSE_GX3 -DGX3_LINK=$(GX3_PORT) 

endif



