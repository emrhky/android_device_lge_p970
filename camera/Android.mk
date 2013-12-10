ifeq ($(TARGET_BOARD_PLATFORM),omap3)

LOCAL_PATH:= $(call my-dir)

BLACKG_OPEN_COM_DEVICE := true

ifdef PRODUCT_MANUFACTURER
    LOCAL_CFLAGS += -DPRODUCT_MANUFACTURER="\"$(PRODUCT_MANUFACTURER)\""
endif

ifdef PRODUCT_MODEL
    LOCAL_CFLAGS += -DPRODUCT_MODEL="\"$(PRODUCT_MODEL)\""
endif

ifdef PRODUCT_DEVICE
    LOCAL_CFLAGS += -DPRODUCT_DEVICE="\"$(PRODUCT_DEVICE)\""
endif

ifdef TARGET_PRODUCT
    LOCAL_CFLAGS += -DTARGET_PRODUCT="\"$(TARGET_PRODUCT)\""
endif

ifdef TARGET_DEVICE
    LOCAL_CFLAGS += -DTARGET_DEVICE="\"$(TARGET_DEVICE)\""
endif

ifdef BLACKG_OPEN_COM_DEVICE
    LOCAL_CFLAGS += -DBLACKG_OPEN_COM_DEVICE="\"$(BLACKG_OPEN_COM_DEVICE)\""
endif

LOCAL_CFLAGS += -fno-short-enums

OMAP3_CAMERA_HAL_SRC := \
	CameraHal_Module.cpp \
	CameraHal.cpp \
	CameraHalUtilClasses.cpp \
	AppCallbackNotifier.cpp \
	ANativeWindowDisplayAdapter.cpp \
	CameraProperties.cpp \
	MemoryManager.cpp \
	Encoder_libjpeg.cpp \
	SensorListener.cpp  \
	NV12_resize.c \
	CameraHal_Utils.cpp

OMAP3_CAMERA_COMMON_SRC:= \
	CameraParameters.cpp \
	TICameraParameters.cpp \
	CameraHalCommon.cpp

OMAP3_CAMERA_USB_SRC:= \
	BaseCameraAdapter.cpp \
	V4LCameraAdapter/V4LCameraAdapter.cpp

#
# USB Camera Adapter
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	$(OMAP3_CAMERA_HAL_SRC) \
	$(OMAP3_CAMERA_USB_SRC) \
	$(OMAP3_CAMERA_COMMON_SRC)

LOCAL_C_INCLUDES := hardware/ti/omap3/dspbridge/inc \
    hardware/ti/omap3/omx/system/src/openmax_il/lcml/inc \
    hardware/ti/omap3/omx/system/src/openmax_il/omx_core/inc \
    hardware/ti/omap3/omx/system/src/openmax_il/common/inc \
    hardware/ti/omap3/omx/image/src/openmax_il/jpeg_enc/inc \
    hardware/ti/omap3/libexif \
    $(LOCAL_PATH)/inc \
    $(LOCAL_PATH)/../hwc \
    $(LOCAL_PATH)/../include \
    $(LOCAL_PATH)/inc/V4LCameraAdapter \
    hardware/ti/omap3/libtiutils \
    hardware/ti/omap3/ion \
    frameworks/native/include \
    frameworks/native/include/ui \
    frameworks/native/include/utils \
    frameworks/av/include \
    frameworks/av/include/camera \
    frameworks/native/include/media/hardware \
    frameworks/native/include/media/openmax \
    external/jhead \
    external/jpeg

LOCAL_SHARED_LIBRARIES:= \
    libui \
    libbinder \
    libutils \
    libcutils \
    libtiutils \
    libcamera_client \
    libion \
    libjpeg \
    libexif \
    libgui \
    libdl

LOCAL_CFLAGS := -fno-short-enums -DCOPY_IMAGE_BUFFER

ifdef HARDWARE_OMX

LOCAL_SRC_FILES += \
    scale.c \
    JpegEncoder.cpp \
    JpegEncoderEXIF.cpp \

LOCAL_CFLAGS += -O0 -g3 -fpic -fstrict-aliasing -DIPP_LINUX -D___ANDROID___ -DHARDWARE_OMX

LOCAL_SHARED_LIBRARIES += \
    libbridge \
    libLCML \
    libOMX_Core

LOCAL_STATIC_LIBRARIES := \
        libexifgnu

endif

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE:= camera.$(TARGET_BOOTLOADER_BOARD_NAME)
LOCAL_MODULE_TAGS:= optional

include $(BUILD_SHARED_LIBRARY)
endif
