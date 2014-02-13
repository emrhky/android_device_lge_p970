#USE_CAMERA_STUB := true

# inherit from the proprietary version
-include vendor/lge/p970/BoardConfigVendor.mk

# Board configuration
TARGET_BOARD_PLATFORM := omap3
TARGET_CPU_ABI := armeabi-v7a
TARGET_CPU_ABI2 := armeabi
TARGET_ARCH := arm
TARGET_ARCH_VARIANT := armv7-a-neon
TARGET_ARCH_VARIANT_CPU := cortex-a8
TARGET_ARCH_VARIANT_FPU := neon
TARGET_ARCH_LOWMEM := true
ARCH_ARM_HAVE_ARMV7A := true
ARCH_ARM_HAVE_TLS_REGISTER := true

TARGET_GLOBAL_CFLAGS += -mtune=cortex-a8 -mfpu=neon -mfloat-abi=softfp
TARGET_GLOBAL_CPPFLAGS += -mtune=cortex-a8 -mfpu=neon -mfloat-abi=softfp
TARGET_arm_CFLAGS := -O3 -fomit-frame-pointer -fstrict-aliasing -funswitch-loops \
                       -fmodulo-sched -fmodulo-sched-allow-regmoves
TARGET_thumb_CFLAGS := -mthumb \
                        -Os \
                        -fomit-frame-pointer \
                        -fstrict-aliasing

TARGET_BOOTLOADER_BOARD_NAME := p970
TARGET_PROVIDES_INIT_TARGET_RC := true
TARGET_NO_BOOTLOADER := true
TARGET_OMAP3 := true
OMAP_ENHANCEMENT := true

ifdef OMAP_ENHANCEMENT
COMMON_GLOBAL_CFLAGS += -DOMAP_ENHANCEMENT -DTARGET_OMAP3 -DOMAP_ENHANCEMENT_CPCAM -DOMAP_ENHANCEMENT_VTC
endif

# for frameworks/native/services/surfaceflinger
# use EGL_IMG_context_priority extension, which helps performance
COMMON_GLOBAL_CFLAGS += -DHAS_CONTEXT_PRIORITY

TARGET_SPECIFIC_HEADER_PATH := device/lge/p970/include

# Makefile variables and C/C++ macros to recognise current pastry
ifeq ($(shell test $(PLATFORM_SDK_VERSION) -ge 16 || echo 1),)
    ANDROID_API_JB_OR_LATER := true
    COMMON_GLOBAL_CFLAGS += -DANDROID_API_JB_OR_LATER
endif
ifeq ($(shell test $(PLATFORM_SDK_VERSION) -ge 14 || echo 1),)
    ANDROID_API_ICS_OR_LATER := true
    COMMON_GLOBAL_CFLAGS += -DANDROID_API_ICS_OR_LATER
endif

#Bootanimation
TARGET_SCREEN_HEIGHT := 800
TARGET_SCREEN_WIDTH := 480
TARGET_BOOTANIMATION_PRELOAD := true
TARGET_BOOTANIMATION_TEXTURE_CACHE := true

#Partitioning
TARGET_USERIMAGES_USE_EXT4 := true
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 665681920
BOARD_USERDATAIMAGE_PARTITION_SIZE := 1170259968
BOARD_FLASH_BLOCK_SIZE := 131072

#Kernel
BOARD_KERNEL_CMDLINE :=
BOARD_KERNEL_BASE := 0x80000000
BOARD_PAGE_SIZE := 0x00000800
# Try to build the kernel
TARGET_KERNEL_CONFIG := cyanogenmod_p970_defconfig
# Keep this as a fallback
TARGET_PREBUILT_KERNEL := device/lge/p970/kernel

BOARD_NEEDS_CUTILS_LOG := true

#Storage
TARGET_USE_CUSTOM_LUN_FILE_PATH := "/sys/devices/platform/omap/musb-omap2430/musb-hdrc/gadget/lun%d/file"

BOARD_HAS_NO_SELECT_BUTTON := true
# Use this flag if the board has a ext4 partition larger than 2gb
#BOARD_HAS_LARGE_FILESYSTEM := true

#Bluetooth
BOARD_HAVE_BLUETOOTH := true
BOARD_HAVE_BLUETOOTH_BCM := true
BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR := device/lge/p970/bluetooth

# OMX
HARDWARE_OMX := true

ifdef HARDWARE_OMX
OMX_JPEG := true
OMX_VENDOR := ti
TARGET_USE_OMX_RECOVERY := true
TARGET_USE_OMAP_COMPAT  := true
BUILD_WITH_TI_AUDIO := 1
BUILD_PV_VIDEO_ENCODERS := 1
OMX_VENDOR_INCLUDES := \
  hardware/ti/omap3/omx/system/src/openmax_il/omx_core/inc \
  hardware/ti/omap3/omx/image/src/openmax_il/jpeg_enc/inc
OMX_VENDOR_WRAPPER := TI_OMX_Wrapper
BOARD_OPENCORE_LIBRARIES := libOMX_Core
BOARD_OPENCORE_FLAGS := -DHARDWARE_OMX=1
#BOARD_CAMERA_LIBRARIES := libcamera
endif

#Mobiledata
BOARD_MOBILEDATA_INTERFACE_NAME := "vsnet0"

#Wifi
BOARD_WLAN_DEVICE := bcm4329
WIFI_DRIVER_FW_PATH_STA         := "/system/etc/firmware/fw_bcm4329.bin"
WIFI_DRIVER_FW_PATH_AP          := "/system/etc/firmware/fw_bcm4329_ap.bin"
WIFI_DRIVER_FW_PATH_P2P          := "/system/etc/firmware/fw_bcm4329_p2p.bin"
WIFI_DRIVER_FW_PATH_MFG          := "/system/etc/firmware/fw_bcm4329_mfg.bin"
WIFI_DRIVER_MODULE_NAME         := "wireless"
WIFI_DRIVER_MODULE_PATH         := "/system/lib/modules/wireless.ko"
WIFI_DRIVER_MODULE_ARG          := "firmware_path=/system/etc/firmware/fw_bcm4329.bin nvram_path=/system/etc/wifi/nvram.txt config_path=/data/misc/wifi/config"
WPA_SUPPLICANT_VERSION          := VER_0_8_X
BOARD_HOSTAPD_PRIVATE_LIB := lib_driver_cmd_wext
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_wext
WIFI_DRIVER_HAS_LGE_SOFTAP      := true
BOARD_WPA_SUPPLICANT_DRIVER := WEXT

# FM Radio
BOARD_HAVE_FM_RADIO := true
BOARD_GLOBAL_CFLAGS += -DHAVE_FM_RADIO

# HW Graphics (EGL fixes + webkit fix)
USE_OPENGL_RENDERER := true
BOARD_EGL_CFG := device/lge/p970/egl.cfg
ENABLE_WEBGL := true

#BOARD_TOUCH_RECOVERY := true
BOARD_CUSTOM_GRAPHICS := ../../../device/lge/p970/recovery/graphics.c
#TARGET_RECOVERY_PIXEL_FORMAT := "RGBX_8888"

BOARD_HAS_VIBRATOR_IMPLEMENTATION := ../../device/lge/p970/vibrator.c

COMMON_GLOBAL_CFLAGS += -DICS_AUDIO_BLOB -DICS_CAMERA_BLOB -DOMAP_ICS_CAMERA -DDONT_USE_FENCE_SYNC

LG_CAMERA_HARDWARE := true

COMMON_GLOBAL_CFLAGS += -DPRODUCT_LG_P970

BOARD_SYSFS_LIGHT_SENSOR := "/sys/devices/platform/omap/omap_i2c.2/i2c-2/2-0060/leds/lcd-backlight/als"

COMMON_GLOBAL_CFLAGS += -DBOARD_CHARGING_CMDLINE_NAME='"rs"' -DBOARD_CHARGING_CMDLINE_VALUE='"c"'

# Enable suspend in charger
BOARD_ALLOW_SUSPEND_IN_CHARGER := true

## Radio fixes
BOARD_RIL_CLASS := ../../../device/lge/p970/ril/

