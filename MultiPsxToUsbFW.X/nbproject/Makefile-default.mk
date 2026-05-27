#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=mcc_generated_files/spi/src/spi0.c mcc_generated_files/system/src/syscfg.c mcc_generated_files/system/src/interrupt.c mcc_generated_files/system/src/clock.c mcc_generated_files/system/src/system.c mcc_generated_files/system/src/protected_io.S mcc_generated_files/system/src/config_bits.c mcc_generated_files/system/src/pins.c mcc_generated_files/timer/src/rtc.c mcc_generated_files/usb/src/usb0.c mcc_generated_files/usb/usb_common/usb_core_descriptors.c mcc_generated_files/usb/usb_common/usb_core_transfer.c mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.c mcc_generated_files/usb/usb_common/usb_core_requests.c mcc_generated_files/usb/usb_common/usb_core_requests_device.c mcc_generated_files/usb/usb_common/usb_core_events.c mcc_generated_files/usb/usb_common/usb_core_requests_interface.c mcc_generated_files/usb/usb_common/usb_core.c mcc_generated_files/usb/usb_hid/usb_hid.c mcc_generated_files/usb/usb_hid/usb_hid_transfer.c mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.c mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.c mcc_generated_files/usb/usb_peripheral/usb_peripheral.c mcc_generated_files/usb/usb_device.c mcc_generated_files/usb/usb_descriptors.c main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o ${OBJECTDIR}/mcc_generated_files/system/src/clock.o ${OBJECTDIR}/mcc_generated_files/system/src/system.o ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o ${OBJECTDIR}/mcc_generated_files/system/src/pins.o ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o ${OBJECTDIR}/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o.d ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o.d ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o.d ${OBJECTDIR}/mcc_generated_files/system/src/clock.o.d ${OBJECTDIR}/mcc_generated_files/system/src/system.o.d ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o.d ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o.d ${OBJECTDIR}/mcc_generated_files/system/src/pins.o.d ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o.d ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o.d ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o.d ${OBJECTDIR}/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o ${OBJECTDIR}/mcc_generated_files/system/src/clock.o ${OBJECTDIR}/mcc_generated_files/system/src/system.o ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o ${OBJECTDIR}/mcc_generated_files/system/src/pins.o ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o ${OBJECTDIR}/main.o

# Source Files
SOURCEFILES=mcc_generated_files/spi/src/spi0.c mcc_generated_files/system/src/syscfg.c mcc_generated_files/system/src/interrupt.c mcc_generated_files/system/src/clock.c mcc_generated_files/system/src/system.c mcc_generated_files/system/src/protected_io.S mcc_generated_files/system/src/config_bits.c mcc_generated_files/system/src/pins.c mcc_generated_files/timer/src/rtc.c mcc_generated_files/usb/src/usb0.c mcc_generated_files/usb/usb_common/usb_core_descriptors.c mcc_generated_files/usb/usb_common/usb_core_transfer.c mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.c mcc_generated_files/usb/usb_common/usb_core_requests.c mcc_generated_files/usb/usb_common/usb_core_requests_device.c mcc_generated_files/usb/usb_common/usb_core_events.c mcc_generated_files/usb/usb_common/usb_core_requests_interface.c mcc_generated_files/usb/usb_common/usb_core.c mcc_generated_files/usb/usb_hid/usb_hid.c mcc_generated_files/usb/usb_hid/usb_hid_transfer.c mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.c mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.c mcc_generated_files/usb/usb_peripheral/usb_peripheral.c mcc_generated_files/usb/usb_device.c mcc_generated_files/usb/usb_descriptors.c main.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=AVR16DU14
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o: mcc_generated_files/spi/src/spi0.c  .generated_files/flags/default/dc2ddc800a694593e90262237a5c85bf041aeac .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/spi/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o -o ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o mcc_generated_files/spi/src/spi0.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o: mcc_generated_files/system/src/syscfg.c  .generated_files/flags/default/d1f173cf2a05a708b27ab7051eac32cb0bd2cbab .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o -o ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o mcc_generated_files/system/src/syscfg.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o: mcc_generated_files/system/src/interrupt.c  .generated_files/flags/default/2c9357134ee5f611e1bee1d5a98cc1c43d37b5c9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o -o ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o mcc_generated_files/system/src/interrupt.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/clock.o: mcc_generated_files/system/src/clock.c  .generated_files/flags/default/82b6dd3620d6cac83ba590f55d57d409a41410ad .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/clock.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/clock.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/clock.o -o ${OBJECTDIR}/mcc_generated_files/system/src/clock.o mcc_generated_files/system/src/clock.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/system.o: mcc_generated_files/system/src/system.c  .generated_files/flags/default/b4178407fb6d2a2ec0c5825809f6d8d649f37b9f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/system.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/system.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/system.o -o ${OBJECTDIR}/mcc_generated_files/system/src/system.o mcc_generated_files/system/src/system.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o: mcc_generated_files/system/src/config_bits.c  .generated_files/flags/default/b6a12fa7a2d2f67dbf3aa9f7b82d17ba0eb5d662 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o -o ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o mcc_generated_files/system/src/config_bits.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/pins.o: mcc_generated_files/system/src/pins.c  .generated_files/flags/default/f6d7ca25ea21947b88aa2ffeedb262e432c26355 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/pins.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/pins.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/pins.o -o ${OBJECTDIR}/mcc_generated_files/system/src/pins.o mcc_generated_files/system/src/pins.c 
	
${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o: mcc_generated_files/timer/src/rtc.c  .generated_files/flags/default/6e655604791cf400469db138f761819f22586e32 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/timer/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o.d" -MT "${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o.d" -MT ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o -o ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o mcc_generated_files/timer/src/rtc.c 
	
${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o: mcc_generated_files/usb/src/usb0.c  .generated_files/flags/default/23d35d4777c59690e4e4f3806bf49f9123c71624 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o -o ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o mcc_generated_files/usb/src/usb0.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o: mcc_generated_files/usb/usb_common/usb_core_descriptors.c  .generated_files/flags/default/cd73a4e2934d46bd017db56802cc8386e98d2d0f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o mcc_generated_files/usb/usb_common/usb_core_descriptors.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o: mcc_generated_files/usb/usb_common/usb_core_transfer.c  .generated_files/flags/default/f2b40366d7af9f988bf2ae48df8f3497e98613e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o mcc_generated_files/usb/usb_common/usb_core_transfer.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o: mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.c  .generated_files/flags/default/2c60225b007ec7a30b4107b1af1dd772749edbd1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o: mcc_generated_files/usb/usb_common/usb_core_requests.c  .generated_files/flags/default/986daae25c17035d7b700c40883f13b43d88dce5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o mcc_generated_files/usb/usb_common/usb_core_requests.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o: mcc_generated_files/usb/usb_common/usb_core_requests_device.c  .generated_files/flags/default/15708e8bfffa5d71b32e9fdca4e03b3e6847ac31 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o mcc_generated_files/usb/usb_common/usb_core_requests_device.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o: mcc_generated_files/usb/usb_common/usb_core_events.c  .generated_files/flags/default/5c6316efd7ed51d3676cdff136a82f919f043c3b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o mcc_generated_files/usb/usb_common/usb_core_events.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o: mcc_generated_files/usb/usb_common/usb_core_requests_interface.c  .generated_files/flags/default/94c318b2633d6db09eca81237ab2b8419c92667c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o mcc_generated_files/usb/usb_common/usb_core_requests_interface.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o: mcc_generated_files/usb/usb_common/usb_core.c  .generated_files/flags/default/518a8200f78a84fc7a5f43e400af92173460fb43 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o mcc_generated_files/usb/usb_common/usb_core.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o: mcc_generated_files/usb/usb_hid/usb_hid.c  .generated_files/flags/default/60ecb2029485850b7f0e19473d4ee7f1f999a96f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_hid" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o mcc_generated_files/usb/usb_hid/usb_hid.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o: mcc_generated_files/usb/usb_hid/usb_hid_transfer.c  .generated_files/flags/default/aa163f8f67e20b018c8bd6809d903a0c3b10f994 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_hid" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o mcc_generated_files/usb/usb_hid/usb_hid_transfer.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o: mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.c  .generated_files/flags/default/1b4c1386a49fae2b0a5e391d71c1f6faeb5a4a50 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o: mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.c  .generated_files/flags/default/788c156130cdf2d28236ad16eb7a5b89bc13c034 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o: mcc_generated_files/usb/usb_peripheral/usb_peripheral.c  .generated_files/flags/default/56e534d3769f640daee9e74edeef18161e7ffadd .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o mcc_generated_files/usb/usb_peripheral/usb_peripheral.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_device.o: mcc_generated_files/usb/usb_device.c  .generated_files/flags/default/7476930e73e217522b21f9b3a8536398cfe2e2a2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_device.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_device.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o mcc_generated_files/usb/usb_device.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o: mcc_generated_files/usb/usb_descriptors.c  .generated_files/flags/default/42a41c2aead711b9d5874c4f00c4c4c6fba79305 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o mcc_generated_files/usb/usb_descriptors.c 
	
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/674b73c4df75efcad7661bae38810c991d61d0e2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/main.o.d" -MT "${OBJECTDIR}/main.o.d" -MT ${OBJECTDIR}/main.o -o ${OBJECTDIR}/main.o main.c 
	
else
${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o: mcc_generated_files/spi/src/spi0.c  .generated_files/flags/default/373991a56088866e261884f466d4003f1a2b4947 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/spi/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o -o ${OBJECTDIR}/mcc_generated_files/spi/src/spi0.o mcc_generated_files/spi/src/spi0.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o: mcc_generated_files/system/src/syscfg.c  .generated_files/flags/default/927ce3012c4739629abf5be8086b9522d1832b66 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o -o ${OBJECTDIR}/mcc_generated_files/system/src/syscfg.o mcc_generated_files/system/src/syscfg.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o: mcc_generated_files/system/src/interrupt.c  .generated_files/flags/default/2709c932b23767c73b39aaf647a991a2e7e10a19 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o -o ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.o mcc_generated_files/system/src/interrupt.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/clock.o: mcc_generated_files/system/src/clock.c  .generated_files/flags/default/2e698948df405adea485f959dfe93f89fd535e78 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/clock.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/clock.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/clock.o -o ${OBJECTDIR}/mcc_generated_files/system/src/clock.o mcc_generated_files/system/src/clock.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/system.o: mcc_generated_files/system/src/system.c  .generated_files/flags/default/f7761ea3bef06460433b5e382a246c3c83163352 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/system.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/system.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/system.o -o ${OBJECTDIR}/mcc_generated_files/system/src/system.o mcc_generated_files/system/src/system.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o: mcc_generated_files/system/src/config_bits.c  .generated_files/flags/default/8c1e60fa14b2424c704c8b64601c4e04f950ec6a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o -o ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.o mcc_generated_files/system/src/config_bits.c 
	
${OBJECTDIR}/mcc_generated_files/system/src/pins.o: mcc_generated_files/system/src/pins.c  .generated_files/flags/default/ca879021a7e7ca9506ef7e5a5bf79fe817f65bd7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/pins.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/pins.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/pins.o -o ${OBJECTDIR}/mcc_generated_files/system/src/pins.o mcc_generated_files/system/src/pins.c 
	
${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o: mcc_generated_files/timer/src/rtc.c  .generated_files/flags/default/cfa75dd4f41c3e8d8c27801c338ec6aabfe5fcf7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/timer/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o.d" -MT "${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o.d" -MT ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o -o ${OBJECTDIR}/mcc_generated_files/timer/src/rtc.o mcc_generated_files/timer/src/rtc.c 
	
${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o: mcc_generated_files/usb/src/usb0.c  .generated_files/flags/default/f2e1a66c2c2da9e34baf10346e8c7d50fd57012f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o -o ${OBJECTDIR}/mcc_generated_files/usb/src/usb0.o mcc_generated_files/usb/src/usb0.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o: mcc_generated_files/usb/usb_common/usb_core_descriptors.c  .generated_files/flags/default/8955a8897a26dfaffb24145e29a18e10d4b68ef1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_descriptors.o mcc_generated_files/usb/usb_common/usb_core_descriptors.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o: mcc_generated_files/usb/usb_common/usb_core_transfer.c  .generated_files/flags/default/1c6e7a3344b7f6312b6c85ee04d22d2aa8e058c6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_transfer.o mcc_generated_files/usb/usb_common/usb_core_transfer.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o: mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.c  .generated_files/flags/default/428b6c5df515b719a12c80979b65441553c72820 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.o mcc_generated_files/usb/usb_common/usb_core_requests_endpoint.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o: mcc_generated_files/usb/usb_common/usb_core_requests.c  .generated_files/flags/default/2286259812a05d84ec969f37bd6ddb0af43de49d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests.o mcc_generated_files/usb/usb_common/usb_core_requests.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o: mcc_generated_files/usb/usb_common/usb_core_requests_device.c  .generated_files/flags/default/4429b8d33214c162a487471ca71524e767edc3cb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_device.o mcc_generated_files/usb/usb_common/usb_core_requests_device.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o: mcc_generated_files/usb/usb_common/usb_core_events.c  .generated_files/flags/default/666b64ab55aa759f916c6a3df47d0733d888a07a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_events.o mcc_generated_files/usb/usb_common/usb_core_events.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o: mcc_generated_files/usb/usb_common/usb_core_requests_interface.c  .generated_files/flags/default/32dafebe193abce831ab72f3dbe41815fab6bcdf .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core_requests_interface.o mcc_generated_files/usb/usb_common/usb_core_requests_interface.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o: mcc_generated_files/usb/usb_common/usb_core.c  .generated_files/flags/default/238dd868e5e4b2e56693aa2e279808cfe9e3dcb7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_common" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_common/usb_core.o mcc_generated_files/usb/usb_common/usb_core.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o: mcc_generated_files/usb/usb_hid/usb_hid.c  .generated_files/flags/default/860383dc8f34106fe9ac232fca090c24b8942a0f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_hid" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid.o mcc_generated_files/usb/usb_hid/usb_hid.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o: mcc_generated_files/usb/usb_hid/usb_hid_transfer.c  .generated_files/flags/default/91b7bdb3ac99a9e83a73c5add1554a0c1593f329 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_hid" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_hid/usb_hid_transfer.o mcc_generated_files/usb/usb_hid/usb_hid_transfer.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o: mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.c  .generated_files/flags/default/64a4fdd293344a1e3cbc331c374f1c72fa85dc36 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.o mcc_generated_files/usb/usb_peripheral/usb_peripheral_endpoint.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o: mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.c  .generated_files/flags/default/f376c857dddfdeccfb70b653169e15c58236c35c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.o mcc_generated_files/usb/usb_peripheral/usb_peripheral_read_write.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o: mcc_generated_files/usb/usb_peripheral/usb_peripheral.c  .generated_files/flags/default/f83bababf1a4a88d2b65c1720eaed55664d82c18 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_peripheral/usb_peripheral.o mcc_generated_files/usb/usb_peripheral/usb_peripheral.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_device.o: mcc_generated_files/usb/usb_device.c  .generated_files/flags/default/80a67f1ec60db0e2096298c2d24fd42783343ec9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_device.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_device.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_device.o mcc_generated_files/usb/usb_device.c 
	
${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o: mcc_generated_files/usb/usb_descriptors.c  .generated_files/flags/default/e78a40aa4aca38043bc729a9ef701c84c114f98 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/usb" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o.d" -MT "${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o.d" -MT ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o -o ${OBJECTDIR}/mcc_generated_files/usb/usb_descriptors.o mcc_generated_files/usb/usb_descriptors.c 
	
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/28554691464cbd6b9c124ba33e91208cca68f7c1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -x c -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -gdwarf-3 -mno-const-data-in-progmem     -MD -MP -MF "${OBJECTDIR}/main.o.d" -MT "${OBJECTDIR}/main.o.d" -MT ${OBJECTDIR}/main.o -o ${OBJECTDIR}/main.o main.c 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o: mcc_generated_files/system/src/protected_io.S  .generated_files/flags/default/aa4f8a18e188ba31391868581a8799e2175afb52 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o 
	${MP_CC} -c $(MP_EXTRA_AS_PRE) -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG=1 -g -DDEBUG  -gdwarf-2  -x assembler-with-cpp -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  -gdwarf-3 -mno-const-data-in-progmem -Wa,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1   -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o -o ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o  mcc_generated_files/system/src/protected_io.S 
	
else
${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o: mcc_generated_files/system/src/protected_io.S  .generated_files/flags/default/ec787a192d3f799fbab18a5231f1f4abd4bdf075 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o 
	${MP_CC} -c $(MP_EXTRA_AS_PRE) -mcpu=$(MP_PROCESSOR_OPTION)  -x assembler-with-cpp -D__$(MP_PROCESSOR_OPTION)__   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -DXPRJ_default=$(CND_CONF)  -gdwarf-3 -mno-const-data-in-progmem -Wa,--defsym=__MPLAB_BUILD=1   -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o.d" -MT "${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o.d" -MT ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o -o ${OBJECTDIR}/mcc_generated_files/system/src/protected_io.o  mcc_generated_files/system/src/protected_io.S 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.map  -D__DEBUG=1  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"   -gdwarf-2 -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -gdwarf-3 -mno-const-data-in-progmem     $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  -o ${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -Wl,--start-group  -Wl,-lm -Wl,--end-group  -Wl,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1
	@${RM} ${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.hex 
	
	
else
${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -Wl,--gc-sections -O1 -ffunction-sections -fdata-sections -fshort-enums -fno-common -funsigned-char -funsigned-bitfields -I"mcc_generated_files/usb" -I"mcc_generated_files/usb/usb_common" -I"mcc_generated_files/usb/usb_peripheral" -I"mcc_generated_files/usb/usb_vendor" -I"mcc_generated_files/usb/usb_hid" -I"mcc_generated_files/usb/usb_cdc" -I"mcc_generated_files/usb/usb_cdc/circular_buffer" -Wall -gdwarf-3 -mno-const-data-in-progmem     $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  -o ${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -Wl,--start-group  -Wl,-lm -Wl,--end-group 
	${MP_CC_DIR}\\avr-objcopy -O ihex "${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "${DISTDIR}/MultiPsxToUsbFW.X.${IMAGE_TYPE}.hex"
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
