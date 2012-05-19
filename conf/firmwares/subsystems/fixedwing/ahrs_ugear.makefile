# attitude estimation for fixedwings via GX3






ifeq ($(TARGET), ap)

ifndef UGEAR_PORT
  UGEAR_PORT=UART0
endif
ifndef UGEAR_BAUD
  UGEAR_BAUD=B115200
endif

ap.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_ugear_new.h\"
ap.CFLAGS += -DUSE_UGEAR
ap.CFLAGS += -DUSE_AHRS
ap.CFLAGS += -DAHRS_UPDATE_FW_ESTIMATOR

ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_ugear_new.c
ap.srcs   += $(SRC_SUBSYSTEMS)/imu.c


ifdef CPU_LED
  ap.CFLAGS += -DAHRS_CPU_LED=$(CPU_LED)
endif

ap.CFLAGS += -DUSE_$(UGEAR_PORT) -D$(UGEAR_PORT)_BAUD=$(UGEAR_BAUD)
ap.CFLAGS += -DUGEAR_LINK=$(UGEAR_PORT) 

endif

#
# Simple simulation of the AHRS result
#
ahrssim_CFLAGS  = -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
ahrssim_CFLAGS += -DUSE_AHRS -DAHRS_UPDATE_FW_ESTIMATOR

ahrssim_srcs    = $(SRC_SUBSYSTEMS)/ahrs.c
ahrssim_srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

sim.CFLAGS += $(ahrssim_CFLAGS)
sim.srcs += $(ahrssim_srcs)

