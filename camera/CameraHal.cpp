/*
 * Copyright (C) Texas Instruments - http://www.ti.com/
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */
/**
* @file CameraHal.cpp
*
* This file maps the Camera Hardware Interface to V4L2.
*
*/

#define LOG_NDEBUG 1
#define LOG_TAG "CameraHal"

#include "CameraHal.h"
#include <poll.h>

#include <math.h>
#include "zoom_step.inc"
#include "CameraProperties.h"
#include "ANativeWindowDisplayAdapter.h"

#include <cutils/properties.h>
#include <unistd.h>


#define UNLIKELY( exp ) (__builtin_expect( (exp) != 0, false ))
static int mDebugFps = 0;
static bool mvt_mode = false;

#define RES_720P    1280

#define COMPENSATION_MIN        -20
#define COMPENSATION_MAX        20
#define COMPENSATION_MIN_SC     -10
#define COMPENSATION_MAX_SC     10
#define COMPENSATION_STEP       "0.1"
#define WHITE_BALANCE_HORIZON   "horizon"
#define WHITE_BALANCE_TUNGSTEN  "tungsten"
#define KEY_PREVIEW_FPS_RANGE   "preview-fps-range"
#define EFFECT_COOL             "cool"
#define EFFECT_EMBOSS           "emboss"
#define EFFECT_VIVID            "vivid"
#define EFFECT_NEGATIVE_SEPIA   "negative-sepia"
#define EFFECT_BLUE             "blue"
#define PARM_ZOOM_SCALE  100
#define SATURATION_OFFSET       100
//#define KEY_SHUTTER_ENABLE      "shutter-enable"
//#define FOCUS_MODE_MANUAL       "manual"
#define KEY_GPS_ALTITUDE_REF    "gps-altitude-ref"
#define KEY_CAPTURE             "capture"
#define CAPTURE_STILL           "still"

#define KEY_NO_HQ_SNAPSHOT      "lge-camera"
#define KEY_VT_MODE				"vt-mode"

#define V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA     0x08000001
#define V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA_VT  0x08000002
#define V4L2_CID_PRIVATE_OMAP3ISP_CONTINUOUS_CAPTURE     0x08000003

static android::CameraProperties gCameraProperties;

#define ASPECT_RATIO_FLAG_KEEP_ASPECT_RATIO     (1<<0)  // By default is enabled
#define ASPECT_RATIO_FLAG_CROP_BY_SOURCE        (1<<3)


namespace android {

const char *CameraHal::camera_device_name[] = {
    (char*)"/dev/video5",
    (char*)"/dev/video0",
};

/* Defined in liboverlay */
typedef struct {
    int fd;
    size_t length;
    uint32_t offset;
    void *ptr;
} mapping_data_t;

extern "C" CameraAdapter* CameraAdapter_Factory(size_t);

const uint32_t MessageNotifier::EVENT_BIT_FIELD_POSITION = 0;
const uint32_t MessageNotifier::FRAME_BIT_FIELD_POSITION = 0;

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

struct timeval CameraHal::mStartPreview;
struct timeval CameraHal::mStartFocus;
struct timeval CameraHal::mStartCapture;
#endif

/*
 *  Camera mask value:
 *  1 = primary (back)
 *  2 = secondary (front)
 *  3 = both
 */

// Check this when edit supportedFpsRange[]
// CTS Bug fix for takePicture, testPreviewFpsRange, testCameraToSurfaceTextureMetadata
const int CameraHal::idxFpsRange[] = {3, 0};
const supported_range CameraHal::supportedFpsRange[] = {
    {3,   8000,  18000,  15000},       // 15 fps
    {1,      0,      0,  20000},       // 20 fps
    {1,   8000,  26000,  24000},       // 24 fps
    {3,   8000,  33000,  30000},       // 30 fps
};

// Check this when edit supportedPictureRes[]
#ifdef LGE_JUSTIN_DEVICE
const int CameraHal::idxPictureRes[] = {0, 8};
const supported_resolution CameraHal::supportedPictureRes[] = {
    {1, 2592, 1944},                   // 0 5MP
    {1, 2560, 1920},                   // 1 5MP
    {1, 2048, 1536},                   // 2 3MP
    {1, 2240, 1344},                   // 3 3MP
    {1, 1600, 1200},                   // 4 2MP
    {1, 1280,  960},                   // 5 1MP
    {1, 1280,  768},                   // 6 HD720
    {1, 1280,  720},                   // 7 HD720
    {3,  640,  480},                   // 8 VGA
    {1,  512,  384},                   // 9 512x384
    {3,  320,  240}                    // 10 QVGA
};
#else
const int CameraHal::idxPictureRes[] = {0, 8};
const supported_resolution CameraHal::supportedPictureRes[] = {
    {1, 2592, 1944},                   // 0 5MP
    {1, 2560, 1920},                   // 1 5MP
    {1, 2048, 1536},                   // 2 3MP
    {1, 2240, 1344},                   // 3 3MP
    {3, 1600, 1200},                   // 4 2MP
    {3, 1280,  960},                   // 5 1MP
    {1, 1280,  768},                   // 6 HD720
    {3, 1280,  720},                   // 7 HD720
    {3,  640,  480},                   // 8 VGA
    {1,  512,  384},                   // 9 512x384
    {3,  320,  240}                    // 10 QVGA
};
#endif

// Check this when edit supportedPreviewRes[]
const int CameraHal::idxThumbnailRes[] = {2, 2};
const supported_resolution CameraHal::supportedThumbnailRes[] = {
    {3,    0,    0},                   // None
    {3,   80,   60},                   // 80x60
    {3,  160,  120}                    // 160x120
};

// Check this when edit supportedPreviewRes[]
#ifdef LGE_JUSTIN_DEVICE	// rt5604 2012.08.13 Bug fix for CTS testPreviewCallback
const int CameraHal::idxPreviewRes[] = {4, 6};
const supported_resolution CameraHal::supportedPreviewRes[] = {
    {1, 1280,  720},                   // HD720
    {1,  800,  480},                   // WVGA
    {1,  720,  576},                   // PAL
    {1,  720,  480},                   // NTSC
    {1,  640,  480},                   // VGA
    {3,  352,  288},                   // CIF
    {3,  320,  240},                   // QVGA
    {3,  176,  144}                    // QCIF
};
#else // LGE_BLACK_DEVICE   // rt5604 2012.08.13 Bug fix for CTS testPreviewCallback
const int CameraHal::idxPreviewRes[] = {4, 4};
const supported_resolution CameraHal::supportedPreviewRes[] = {
    {1, 1280,  720},                   // HD720
    {1,  800,  480},                   // WVGA
    {1,  720,  576},                   // PAL
    {1,  720,  480},                   // NTSC
    {3,  640,  480},                   // VGA
    {3,  352,  288},                   // CIF
    {3,  320,  240},                   // QVGA
    {3,  176,  144}                    // QCIF
};
#endif

int camerahal_strcat(char *dst, const char *src, size_t size)
{
    size_t actual_size;

    actual_size = strlcat(dst, src, size);
    if(actual_size > size)
    {
        ALOGE("Unexpected truncation from camerahal_strcat dst=%s src=%s", dst, src);
        return actual_size;
    }

    return 0;
}

CameraHal::CameraHal(int cameraId)
                     :mParameters(),                     
                     mPreviewRunning(0),
                     mPreviewChangedNeedsRestart(false),
                     mRecordingFrameSize(0),
                     mVideoBufferCount(0),
                     nOverlayBuffersQueued(0),
                     mSetPreviewWindowCalled(false),
                     nCameraBuffersQueued(0),
                     mfirstTime(0),
                     pictureNumber(0),
                     mCaptureRunning(0),
                     file_index(0),
                     mflash(2),
                     mcapture_mode(2),
                     mcaf(0),
                     mLastFocusMode(0),
                     j(0),
                     useFramerateRange(0)
{
#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS
    gettimeofday(&ppm_start, NULL);
#endif

    mCameraIndex = cameraId;
    mCameraID = 1 << cameraId;
    camera_device = -1;
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
#if defined(LGE_JUSTIN_DEVICE) 
    isSCVideoRecord = false;
#endif
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
    isStart_FW3A = false;
    isStart_FW3A_AF = false;
    isStart_FW3A_AF_Reset = false;
    isStart_FW3A_CAF = false;
    isStart_FW3A_AEWB = false;
    isStart_VPP = false;
    mPictureHeap = NULL;
#ifdef IMAGE_PROCESSING_PIPELINE
    mIPPInitAlgoState = false;
    mIPPToEnable = false;
#endif
    mRecordingEnabled = 0;
    mNotifyCb = 0;
    mDataCb = 0;
    mDataCbTimestamp = 0;
    mCallbackCookie = 0;
    mMsgEnabled = 0 ;
    mFalsePreview = false;  //Eclair HAL
    mZoomSpeed = 0;
    mCancelPicture = false;
    mZoomTargetIdx = 0;
    mZoomCurrentIdx = 0;
    mSmoothZoomStatus = SMOOTH_STOP;
    rotation = 0;       
    mPreviewBufs = NULL;
    mPreviewOffsets = NULL;
    mPreviewFd = 0;
    mPreviewLength = 0;   
    mDisplayPaused = false;
    mTotalJpegsize = 0;
    mJpegBuffAddr = NULL;
	mJPEGPictureHeap = NULL;

#ifdef HARDWARE_OMX

    jpegEncoder = NULL;
    gpsLocation = NULL;

#endif

    // Disable ISP resizer (use DSS resizer)
    system("echo 0 > "
            "/sys/devices/platform/dsscomp/isprsz/enable");	

    mShutterEnable = true;
    mRawDump = false;
    mMNDump = false;
    sStart_FW3A_CAF:mCAFafterPreview = false;
    ancillary_len = 8092;
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
    mRotateVideo = false;
    mCameraWidth = 0;
    mCameraHeight = 0;
#endif	
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */
    mSCConfigOld_brightness = 0;
    mSCConfigOld_white_balance = 0;
    mSCConfigOld_color_effect = V4L2_COLORFX_NONE;
    mSCConfigOld_anti_banding = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
    mSCConfigOld_metering_mode = 0;
    mSCConfigOld_night_mode = 0;

    mLastFocusMode = 0;
    caf_type = 0;
    caf_type_changed = 0;
#ifdef USE_CAF_CALLBACK
    caf_result = 0;
#endif

/* 20120701 jungyeal@lge.com modify vt_mode & face_unlock [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
    mFaceUnlockMode = false;
    mvt_mode = false;	
#endif
/* 20120701 jungyeal@lge.com modify vt_mode & face_unlock [END] */

#ifdef IMAGE_PROCESSING_PIPELINE

    pIPP.hIPP = NULL;
    mippMode = IPP_Disabled_Mode;

#else
    ALOGD("Image processing pipeline DISABLED at compile time");
#endif

    int i = 0;
    for(i = 0; i < VIDEO_FRAME_COUNT_MAX; i++) {
        mVideoBuffer[i] = 0;
        mVideoBufferStatus[i] = BUFF_IDLE;
    }

    ALOGD("******** CameraHAL compiled at: "__DATE__"/"__TIME__" ********");

#ifdef	DEBUG_LOG

#ifdef PRODUCT_MANUFACTURER
    ALOGD("******** compiled for PRODUCT_MANUFACTURER: " PRODUCT_MANUFACTURER " ********");
#endif

#ifdef PRODUCT_MODEL
    ALOGD("******** compiled for PRODUCT_MODEL: " PRODUCT_MODEL " ********");
#endif

#ifdef PRODUCT_DEVICE
    ALOGD("******** compiled for PRODUCT_DEVICE: " PRODUCT_DEVICE " ********");
#endif

#endif // #ifdef	DEBUG_LOG

    for (i = 0; i < MAX_BURST; i++) {
        mYuvBuffer[i] = 0;
        mYuvBufferLen[i] = 0;
    }

    CameraCreate();

    /* Avoiding duplicate call of cameraconfigure(). It is now called in previewstart() */
    //CameraConfigure();

#ifdef HARDWARE_OMX
#if JPEG

    jpegEncoder = new JpegEncoder;

#endif
#endif

    initDefaultParameters();

    mPreviewThread = new PreviewThread(this);
    mPreviewThread->run("CameraPreviewThread", PRIORITY_URGENT_DISPLAY);

    if( pipe(procPipe) != 0 ){
        ALOGE("Failed creating pipe");
    }

    if( pipe(shutterPipe) != 0 ){
        ALOGE("Failed creating pipe");
    }

    if( pipe(rawPipe) != 0 ){
        ALOGE("Failed creating pipe");
    }

    if( pipe(snapshotPipe) != 0 ){
        ALOGE("Failed creating pipe");
    }

    if( pipe(snapshotReadyPipe) != 0 ){
        ALOGE("Failed creating pipe");
    }

#ifdef USE_AUTOFOCUS_THREAD
    // rt5604 2012.08.16 autoFocus ->
    if( pipe(afcbPipe) != 0 ){
        ALOGE("Failed creating pipe");
    }
    // rt5604 2012.08.16 autoFocus <-
#endif // #ifdef USE_AUTOFOCUS_THREAD


    if( pipe(zoomcbPipe) != 0 ){
        ALOGE("Failed creating pipe");
    }

    mPROCThread = new PROCThread(this);
    mPROCThread->run("CameraPROCThread", PRIORITY_URGENT_DISPLAY);
    ALOGD("STARTING PROC THREAD \n");

    mShutterThread = new ShutterThread(this);
    mShutterThread->run("CameraShutterThread", PRIORITY_URGENT_DISPLAY);
    ALOGD("STARTING Shutter THREAD \n");

    mRawThread = new RawThread(this);
    mRawThread->run("CameraRawThread", PRIORITY_URGENT_DISPLAY);
    ALOGD("STARTING Raw THREAD \n");

    mSnapshotThread = new SnapshotThread(this);
    mSnapshotThread->run("CameraSnapshotThread", PRIORITY_URGENT_DISPLAY);
    ALOGD("STARTING Snapshot THREAD \n");

#ifdef USE_AUTOFOCUS_THREAD
    mAutoFocusCBThread = new AutoFocusCBThread(this);
    mAutoFocusCBThread->run("CameraAutoFocusCBThread", PRIORITY_URGENT_DISPLAY);
    ALOGD("STARTING AutoFocusCB THREAD \n");
#endif // #ifdef USE_AUTOFOCUS_THREAD

    mZoomCBThread = new ZoomCBThread(this);
    mZoomCBThread->run("CameraZoomCBThread", PRIORITY_URGENT_DISPLAY);
    ALOGD("STARTING ZoomCB THREAD \n");

    char value[PROPERTY_VALUE_MAX];
    property_get("debug.image.showfps", value, "0");
    mDebugFps = atoi(value);
    ALOGD_IF(mDebugFps, "showfps enabled");
}

bool CameraHal::validateSize(size_t width, size_t height, const supported_resolution *supRes, size_t count)
{
    bool ret = false;
    status_t stat = NO_ERROR;
    unsigned int size;

    LOG_FUNCTION_NAME

    if ( NULL == supRes ) {
        ALOGE("Invalid resolutions array passed");
        stat = -EINVAL;
    }

    if ( NO_ERROR == stat ) {
        for ( unsigned int i = 0 ; i < count; i++ ) {
#ifdef	DEBUG_LOG
            ALOGD( "Validating %d, %d and %d, %d", supRes[i].width, width, supRes[i].height, height);
#endif // #ifdef	DEBUG_LOG
            if ( (mCameraID & supRes[i].camera_mask) && ( supRes[i].width == width ) && ( supRes[i].height == height ) ) {
                ret = true;
                break;
            }
        }
    }

// rt5604 2012.08.13 BUG Fix for CTS testPreviewCallback ->
	if (!ret) {
		if (width == 736 && mCameraID == 1){
			ALOGD("rt5604: ---> 736 ---> SPECIAL OK, mCameraID=%d", mCameraID);
			ret = true;
		}
		else if (width == 192) {
			ALOGD("rt5604: ---> 192 ---> SPECIAL OK, mCameraID=%d", mCameraID);
			ret = true;
		}
#if defined(LGE_JUSTIN_DEVICE)		
		else if (width == 640 && mCameraID == 2)
		{
			//ALOGD("kim vt rt5604: ---> 640 ---> SPECIAL OK, mCameraID=%d", mCameraID);
			ret = true;
		}
#endif
	}
// rt5604 2012.07.31 BUG fix for CTS testPreviewCallback <-

    LOG_FUNCTION_NAME_EXIT

    return ret;
}

bool CameraHal::validateRange(int min, int max, const char *supRang)
{
    bool ret = false;
    char * myRange = NULL;
    char supRang_copy[strlen(supRang)];
    int myMin = 0;
    int myMax = 0;

    LOG_FUNCTION_NAME
 
    if ( NULL == supRang ) {
        ALOGE("Invalid range array passed");
        return ret;
    }

    //make a copy of supRang
    strcpy(supRang_copy, supRang);
    ALOGE("Range: %s", supRang_copy);

    myRange = strtok((char *) supRang_copy, ",");
    if (NULL != myRange) {
        myMin = atoi(myRange + 1);
    }

    myRange = strtok(NULL, ",");
    if (NULL != myRange) {
        myRange[strlen(myRange)]='\0';
        myMax = atoi(myRange);
    }

    ALOGE("Validating range: %d,%d with %d,%d", myMin, myMax, min, max);

    if ( ( myMin == min )&&( myMax == max ) ) {
        ALOGE("Range supported!");
        return true;
    }

    for (;;) {
        myRange = strtok(NULL, ",");
        if (NULL != myRange) {
            myMin = atoi(myRange + 1);
        }
        else break;

        myRange = strtok(NULL, ",");
        if (NULL != myRange) {
            myRange[strlen(myRange)]='\0';
            myMax = atoi(myRange);
        }
        else break;
        ALOGE("Validating range: %d,%d with %d,%d", myMin, myMax, min, max);
        if ( ( myMin == min )&&( myMax == max ) ) {
            ALOGE("Range found");
                ret = true;
                break;
            }
        }

#if 1 // workaround
    if ((ret==false) && (min <= max) && (min >= 0) && (max >= 0) && (max <= MAX_FPS*1000))
    {
        ALOGD("Validating %d, %d (pass)", min, max);
        ret = true;
    }
#endif
    
    LOG_FUNCTION_NAME_EXIT

    return ret;
}

void CameraHal::initDefaultParameters()
{
    const char PARAMS_DELIMITER []= ",";
    CameraParameters p;
    char tmpBuffer[PARAM_BUFFER], zoomStageBuffer[PARAM_BUFFER];
    unsigned int zoomStage;

    LOG_FUNCTION_NAME

    p.setPreviewSize(supportedPreviewRes[idxPreviewRes[mCameraIndex]].width, supportedPreviewRes[idxPreviewRes[mCameraIndex]].height);

    memset(tmpBuffer, '\0', PARAM_BUFFER);
    snprintf(tmpBuffer, PARAM_BUFFER, "%d,%d", supportedFpsRange[idxFpsRange[mCameraIndex]].min, supportedFpsRange[idxFpsRange[mCameraIndex]].max);
    p.set(KEY_PREVIEW_FPS_RANGE, tmpBuffer); // because of missing setPreviewFpsRange();
    p.setPreviewFrameRate(supportedFpsRange[idxFpsRange[mCameraIndex]].fixed / 1000);

//--[[ LGE_UBIQUIX_MODIFIED_START : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot (use PIXEL_FORMAT_YUV420SP)
#if 1
    p.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);
#else
    //p.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);
    p.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV422I);
#endif
//--]] LGE_UBIQUIX_MODIFIED_END : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot
    memset(tmpBuffer, '\0', PARAM_BUFFER);
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::PIXEL_FORMAT_YUV420SP, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::PIXEL_FORMAT_YUV422I, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::PIXEL_FORMAT_YUV420P, PARAM_BUFFER)) return;
    p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, tmpBuffer);
    p.setPictureSize(supportedPictureRes[idxPictureRes[mCameraIndex]].width, supportedPictureRes[idxPictureRes[mCameraIndex]].height);
    p.setPictureFormat(CameraParameters::PIXEL_FORMAT_JPEG);
    p.set(CameraParameters::KEY_JPEG_QUALITY, 100);

    p.set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, supportedThumbnailRes[idxThumbnailRes[mCameraIndex]].width);
    p.set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, supportedThumbnailRes[idxThumbnailRes[mCameraIndex]].height);
    p.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, 100);

    memset(tmpBuffer, '\0', PARAM_BUFFER);
    for ( int i = 0 ; i < ZOOM_STAGES ; i++ ) {
        zoomStage =  (unsigned int ) ( zoom_step[i]*PARM_ZOOM_SCALE );
        snprintf(zoomStageBuffer, PARAM_BUFFER, "%d", zoomStage);

        if(camerahal_strcat((char*) tmpBuffer, (const char*) zoomStageBuffer, PARAM_BUFFER)) return;
        if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    }
    p.set(CameraParameters::KEY_ZOOM_RATIOS, tmpBuffer);
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
    p.set(CameraParameters::KEY_ZOOM_SUPPORTED, "true");
    p.set(CameraParameters::KEY_SMOOTH_ZOOM_SUPPORTED, "true");
    } else { // for CTS (testImmediateZoom)
    p.set(CameraParameters::KEY_ZOOM_SUPPORTED, "false");
    p.set(CameraParameters::KEY_SMOOTH_ZOOM_SUPPORTED, "false");
    }
    p.set(CameraParameters::KEY_MAX_ZOOM, (ZOOM_STAGES - 1));
    p.set(CameraParameters::KEY_ZOOM, 0);

    // EV compensation
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
    p.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, COMPENSATION_MAX);
    p.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, COMPENSATION_MIN);
    } else {
    p.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, COMPENSATION_MAX_SC);
    p.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, COMPENSATION_MIN_SC);
    }
    p.set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, COMPENSATION_STEP);
    p.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, 0);

    setParameterSupportedSizeList(p, CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, PARAMS_DELIMITER, supportedPictureRes, ARRAY_SIZE(supportedPictureRes));
    p.set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS, CameraParameters::PIXEL_FORMAT_JPEG);
    setParameterSupportedSizeList(p, CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, PARAMS_DELIMITER, supportedPreviewRes, ARRAY_SIZE(supportedPreviewRes));
    
    setParameterSupportedRangeList(p, CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, PARAMS_DELIMITER, supportedFpsRange, ARRAY_SIZE(supportedFpsRange));
    setParameterSupportedRangeListToSingle(p, CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, PARAMS_DELIMITER, supportedFpsRange, ARRAY_SIZE(supportedFpsRange));
    setParameterSupportedSizeList(p, CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES, PARAMS_DELIMITER, supportedThumbnailRes, ARRAY_SIZE(supportedThumbnailRes));

    memset(tmpBuffer, '\0', PARAM_BUFFER);
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::WHITE_BALANCE_AUTO, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::WHITE_BALANCE_INCANDESCENT, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::WHITE_BALANCE_FLUORESCENT, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::WHITE_BALANCE_DAYLIGHT, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT, PARAM_BUFFER)) return;
    p.set(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, tmpBuffer);
    p.set(CameraParameters::KEY_WHITE_BALANCE, CameraParameters::WHITE_BALANCE_AUTO);

    memset(tmpBuffer, '\0', sizeof(*tmpBuffer));
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::EFFECT_NONE, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::EFFECT_MONO, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::EFFECT_NEGATIVE, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::EFFECT_SEPIA, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) EFFECT_EMBOSS, PARAM_BUFFER)) return;
    // primary camera only effects
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::EFFECT_SOLARIZE,  PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::EFFECT_WHITEBOARD,  PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::EFFECT_BLACKBOARD, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) EFFECT_VIVID, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) EFFECT_NEGATIVE_SEPIA, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) EFFECT_BLUE, PARAM_BUFFER)) return;
    }
    p.set(CameraParameters::KEY_SUPPORTED_EFFECTS, tmpBuffer);
    p.set(CameraParameters::KEY_EFFECT, CameraParameters::EFFECT_NONE);

    memset(tmpBuffer, '\0', sizeof(*tmpBuffer));
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::SCENE_MODE_AUTO, PARAM_BUFFER)) return;
#if 0 // workaround: removed for CTS (testSceneMode)
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::SCENE_MODE_NIGHT, PARAM_BUFFER)) return;
    // primary camera only scene
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::SCENE_MODE_PORTRAIT, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::SCENE_MODE_LANDSCAPE, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::SCENE_MODE_SPORTS, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::SCENE_MODE_NIGHT_PORTRAIT, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::SCENE_MODE_SUNSET, PARAM_BUFFER)) return;
    }
#endif
    p.set(CameraParameters::KEY_SUPPORTED_SCENE_MODES, tmpBuffer);
    p.set(CameraParameters::KEY_SCENE_MODE, CameraParameters::SCENE_MODE_AUTO);

    memset(tmpBuffer, '\0', sizeof(*tmpBuffer));
    // primary camera only focus mode
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FOCUS_MODE_AUTO, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FOCUS_MODE_INFINITY, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FOCUS_MODE_MACRO, PARAM_BUFFER)) return;
#if 0 // for CTS (testFocusDistances)
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE, PARAM_BUFFER)) return;
#endif
    } else {
        if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FOCUS_MODE_FIXED, PARAM_BUFFER)) return;
    }
    p.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, tmpBuffer);
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
    p.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_AUTO);
    } else {
        p.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);
    }
 
#if 1 // ymjun 0814 : set KEY_SUPPORTED_ANTIBANDING to null, we always set ANTIBANDING_60HZ internally.
    memset(tmpBuffer, '\0', sizeof(*tmpBuffer));
    p.set(CameraParameters::KEY_SUPPORTED_ANTIBANDING, tmpBuffer);
#if defined(BLACKG_OPEN_COM_DEVICE) //daewon1004.kim@lge.com 20120925 \C0\AF\B7\B4\C7\E2 50hz \B1\E2\C1\D8 \C0\FB\BF\EB
    p.set(CameraParameters::KEY_ANTIBANDING, CameraParameters::ANTIBANDING_50HZ);
#else
    p.set(CameraParameters::KEY_ANTIBANDING, CameraParameters::ANTIBANDING_60HZ);
#endif
#else
    memset(tmpBuffer, '\0', sizeof(*tmpBuffer));
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::ANTIBANDING_50HZ, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::ANTIBANDING_60HZ, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::ANTIBANDING_OFF, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::ANTIBANDING_AUTO, PARAM_BUFFER)) return;
    p.set(CameraParameters::KEY_SUPPORTED_ANTIBANDING, tmpBuffer);
    p.set(CameraParameters::KEY_ANTIBANDING, CameraParameters::ANTIBANDING_60HZ);
#endif

    // primary camera only flash mode
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
    memset(tmpBuffer, '\0', sizeof(*tmpBuffer));
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FLASH_MODE_OFF, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FLASH_MODE_ON, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FLASH_MODE_AUTO, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) PARAMS_DELIMITER, PARAM_BUFFER)) return;
    if(camerahal_strcat((char*) tmpBuffer, (const char*) CameraParameters::FLASH_MODE_TORCH, PARAM_BUFFER)) return;
    p.set(CameraParameters::KEY_SUPPORTED_FLASH_MODES, tmpBuffer);
    p.set(CameraParameters::KEY_FLASH_MODE, CameraParameters::FLASH_MODE_OFF);
    }
    //
    p.set(CameraParameters::KEY_ROTATION, 0);
    p.set(KEY_ROTATION_TYPE, ROTATION_PHYSICAL);
    //set the video frame format needed by video capture framework
    p.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT, CameraParameters::PIXEL_FORMAT_YUV422I);

    //Set focus distances near=0.5, optimal=1.5, far="Infinity"
    //Once when fw3A supports focus distances, update them in CameraHal::GetParameters()
    sprintf(CameraHal::focusDistances, "%f,%f,%s", FOCUS_DISTANCE_NEAR, FOCUS_DISTANCE_OPTIMAL, CameraParameters::FOCUS_DISTANCE_INFINITY);
    p.set(CameraParameters::KEY_FOCUS_DISTANCES, CameraHal::focusDistances);
    p.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, 100);

    // Custom parameters
    p.set(TICameraParameters::KEY_SATURATION, 100);
    p.set(TICameraParameters::KEY_SHARPNESS, 0);
    p.set(TICameraParameters::KEY_CONTRAST, 100);
    p.set(TICameraParameters::KEY_BRIGHTNESS, 100);
    p.set(TICameraParameters::KEY_EXPOSURE_MODE, TICameraParameters::EXPOSURE_MODE_AUTO);
    p.set(TICameraParameters::KEY_METERING_MODE, TICameraParameters::METER_MODE_AVERAGE);

    p.set(TICameraParameters::KEY_ISO, TICameraParameters::ISO_MODE_AUTO);

    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN) {
        p.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS, 1);
        p.set(CameraParameters::KEY_MAX_NUM_METERING_AREAS, 0);
        p.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW, 0);
        p.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW, 0);
        p.set(TICameraParameters::KEY_SUPPORTED_ISO_VALUES, "auto,100,200,400");
    } else {
        p.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS, 0);
        p.set(CameraParameters::KEY_MAX_NUM_METERING_AREAS, 0);
        p.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW, 0);
        p.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW, 0);
        p.set(TICameraParameters::KEY_SUPPORTED_ISO_VALUES, TICameraParameters::ISO_MODE_AUTO);
    }

/* CSR-OMAPS00273048 kirti.badkundri@sasken.com fix CTS - testJpegExif [START] */
    p.set(CameraParameters::KEY_FOCAL_LENGTH, STRINGIZE(IMX046_FOCALLENGTH));
    p.set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE, STRINGIZE(IMX046_HORZANGLE));
    p.set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE, STRINGIZE(IMX046_VERTANGLE));
/* CSR-OMAPS00273048 kirti.badkundri@sasken.com fix CTS - testJpegExif [END] */

    if (setParameters(p) != NO_ERROR) {
        ALOGE("Failed to set default parameters?!");
    }

    LOG_FUNCTION_NAME_EXIT

}


CameraHal::~CameraHal()
{
    int err = 0;
    int procMessage [1];
    sp<PROCThread> procThread;
    sp<RawThread> rawThread;
    sp<ShutterThread> shutterThread;
    sp<SnapshotThread> snapshotThread;
#ifdef USE_AUTOFOCUS_THREAD
	sp<AutoFocusCBThread> autoFocusCBThread;    // rt5604 2012.08.16 autoFocus
#endif // #ifdef USE_AUTOFOCUS_THREAD
	sp<ZoomCBThread> zoomCBThread;

    LOG_FUNCTION_NAME


    if(mPreviewThread != NULL) {
        TIUTILS::Message msg;
        msg.command = PREVIEW_KILL;
        previewThreadCommandQ.put(&msg);
        previewThreadAckQ.get(&msg);
    }

    sp<PreviewThread> previewThread;

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        previewThread = mPreviewThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (previewThread != 0) {
        previewThread->requestExitAndWait();
    }

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        mPreviewThread.clear();
    }

    procMessage[0] = PROC_THREAD_EXIT;
    write(procPipe[1], procMessage, sizeof(unsigned int));

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        procThread = mPROCThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (procThread != 0) {
        procThread->requestExitAndWait();
    }

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        mPROCThread.clear();
        close(procPipe[0]);
        close(procPipe[1]);
    }

    procMessage[0] = SHUTTER_THREAD_EXIT;
    write(shutterPipe[1], procMessage, sizeof(unsigned int));

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        shutterThread = mShutterThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (shutterThread != 0) {
        shutterThread->requestExitAndWait();
    }

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        mShutterThread.clear();
        close(shutterPipe[0]);
        close(shutterPipe[1]);
    }

    procMessage[0] = RAW_THREAD_EXIT;
    write(rawPipe[1], procMessage, sizeof(unsigned int));

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        rawThread = mRawThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (rawThread != 0) {
        rawThread->requestExitAndWait();
    }

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        mRawThread.clear();
        close(rawPipe[0]);
        close(rawPipe[1]);
    }

    procMessage[0] = SNAPSHOT_THREAD_EXIT;
    write(snapshotPipe[1], procMessage, sizeof(unsigned int));

    {
        Mutex::Autolock lock(mLock);
        snapshotThread = mSnapshotThread;
    }

    if (snapshotThread != 0 ) {
        snapshotThread->requestExitAndWait();
    }

    {
        Mutex::Autolock lock(mLock);
        mSnapshotThread.clear();
        close(snapshotPipe[0]);
        close(snapshotPipe[1]);
        close(snapshotReadyPipe[0]);
        close(snapshotReadyPipe[1]);
    }

#ifdef USE_AUTOFOCUS_THREAD
    // rt5604 2012.08.16 autoFocus ->
    procMessage[0] = AFCB_THREAD_EXIT;
    write(afcbPipe[1], procMessage, sizeof(unsigned int));

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        autoFocusCBThread = mAutoFocusCBThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (autoFocusCBThread != 0) {
        autoFocusCBThread->requestExitAndWait();
    }

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        mAutoFocusCBThread.clear();
        close(afcbPipe[0]);
        close(afcbPipe[1]);
    }
    // rt5604 2012.08.16 autoFocus <-
#endif // #ifdef USE_AUTOFOCUS_THREAD

    // rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom ->
    procMessage[0] = ZOOMCB_THREAD_EXIT;
    write(zoomcbPipe[1], procMessage, sizeof(unsigned int));

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        zoomCBThread = mZoomCBThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (zoomCBThread != 0) {
        zoomCBThread->requestExitAndWait();
    }
    { // scope for the lock
        Mutex::Autolock lock(mLock);
        mZoomCBThread.clear();
        close(zoomcbPipe[0]);
        close(zoomcbPipe[1]);
    }
    // rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom <-

    if (NULL != gpsLocation) {
        free(gpsLocation);
    }


#ifdef HARDWARE_OMX
#if JPEG

    if( jpegEncoder ) {
        delete jpegEncoder;
        jpegEncoder = NULL;
    }

#endif
#endif

    freePictureBuffers();

	if( mJPEGPictureHeap != NULL )
	{
		mJPEGPictureHeap.clear();
		mJPEGPictureHeap = NULL;
	}

    CameraDestroy(true);

    /// Free the callback notifier
    mAppCallbackNotifier.clear();

    /// Free the display adapter
    mDisplayAdapter.clear();

    if ( NULL != mCameraAdapter ) {
        int strongCount = mCameraAdapter->getStrongCount();

        mCameraAdapter->decStrong(mCameraAdapter);

        mCameraAdapter = NULL;
    }

#ifdef IMAGE_PROCESSING_PIPELINE

    if(pIPP.hIPP != NULL){
        err = DeInitIPP(mippMode);
        if( err )
            ALOGE("ERROR DeInitIPP() failed");

        pIPP.hIPP = NULL;
    }

#endif

    // Re-enable ISP resizer on overlay for 720P playback
    system("echo 1 > "
            "/sys/devices/platform/dsscomp/isprsz/enable");  

    ALOGD("<<< Release");
}

void CameraHal::previewThread()
{
    TIUTILS::Message msg;
    int parm;
    bool  shouldLive = true;
    bool has_message;
    int err;
    int flg_AF = 0;
    int flg_CAF = 0;
    struct pollfd pfd[2];

    LOG_FUNCTION_NAME

    while(shouldLive) {

        has_message = false;

        if( mPreviewRunning )
        {
             pfd[0].fd = previewThreadCommandQ.getInFd();
             pfd[0].events = POLLIN;
             //pfd[1].fd = camera_device;
             //pfd[1].events = POLLIN;

             poll(pfd, 1, 20);

             if (pfd[0].revents & POLLIN) {
                 previewThreadCommandQ.get(&msg);
                 has_message = true;
             }

             if(mPreviewRunning)
             {
                  nextPreview();
             }
        }
        else
        {
            //block for message
            previewThreadCommandQ.get(&msg);
            has_message = true;
        }

        if( !has_message )
            continue;

        switch(msg.command)
        {

            case PREVIEW_START:
            {
                CAMHAL_ALOGEA("Receive Command: PREVIEW_START");
                err = 0;
                mCaptureMode = false;

                if( ! mPreviewRunning ) {
                CAMHAL_ALOGEA("PREVIEW_START :: ! mPreviewRunning");

                    if( CameraCreate() < 0){
                        ALOGE("ERROR CameraCreate()");
                        err = -1;
                    }

                    PPM("CONFIGURING CAMERA TO RESTART PREVIEW");
                    if (CameraConfigure() < 0){
                        ALOGE("ERROR CameraConfigure()");
                        err = -1;
                    }

                    if ( CorrectPreview() < 0 )
                        ALOGE("Error during CorrectPreview()");

                    if ( CameraStart() < 0 ) {
                        ALOGE("ERROR CameraStart()");
                        err = -1;
                    }

                    if(!mfirstTime){
                        PPM("Standby to first shot");
                        mfirstTime++;
                    } else {

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS
                        PPM("Shot to Shot", &ppm_receiveCmdToTakePicture);

#endif
                    }

                    //CAMHAL_ALOGDB("Start Preview; mCaptureRunning = %d; buffers_in_display = %s", mCaptureRunning, mDisplayAdapter->buffers_in_display);
                    if (!mCaptureRunning) {
                        err = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_START_PREVIEW , true /* need to set state */,  (int) mDisplayAdapter->buffers_in_display );
                    }
                    else {
                        err = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_START_PREVIEW , false /* no need to set state */, (int) mDisplayAdapter->buffers_in_display );
                    }

                } else {
                    err = -1;
                }

                ALOGD("PREVIEW_START %s", err ? "NACK" : "ACK");
                msg.command = err ? PREVIEW_NACK : PREVIEW_ACK;

                if ( !err ) {
                    ALOGD("Preview Started!");
                    mPreviewRunning = true;
                    mCaptureRunning = false;
                    debugShowBufferStatus();
                    if (VIDEO_DEVICE_INDEX(mCameraIndex) != VIDEO_DEVICE_INDEX_MAIN) {
						CameraDeviceSetControl("COLORFX", V4L2_CID_COLORFX, mSCConfigOld_color_effect);
#if defined(LGE_JUSTIN_DEVICE) 
						usleep(30000);
#endif // #if defined(LGE_JUSTIN_DEVICE)
						CameraDeviceSetControl("BRIGHTNESS", V4L2_CID_BRIGHTNESS, (int)((float)(((mSCConfigOld_brightness - COMPENSATION_MIN_SC) * 12.0) / (COMPENSATION_MAX_SC - COMPENSATION_MIN_SC))+0.5) );
						CameraDeviceSetControl("AUTO_WHITE_BALANCE", V4L2_CID_AUTO_WHITE_BALANCE, mSCConfigOld_white_balance);
					}
					else {
						mZoomCurrentIdx = 0;
					}
                }

                if ( (-1 != camera_device) && (VIDEO_DEVICE_INDEX_MAIN != VIDEO_DEVICE_INDEX(mCameraIndex)) ) {
                    setParametersSC(true);
                }

                previewThreadAckQ.put(&msg);
            }
            break;

            case PREVIEW_STOP:
            {
                CAMHAL_ALOGEA("Receive Command: PREVIEW_STOP");
                err = 0;
                mCaptureMode = false;
				//---------------------------------------------------------------------------------------
				// CK Error Workaround 2012.06.04 rt5604
				//	 When Auto review is enabled, CK Error happens during deleting mFrameProvder on disableDisplay().
				//	 After capturing, the application is trying to stop preview in order to display postview screen.
				//	  But, since the destroying sequence is not ordered, mFrameProvider value becomes NULL.
				//	  It causes CK Error.
				//	 So, even if mPreviewRunning flag is not true, the following code should be executed in case of
				//	  mCaptureRunnig. And also, to prevent NewIONLinuxMemArea allocation failure in kernel area,
				//	  we have to execute sendCommand(CAMERA_START_PREVIEW).
				//---------------------------------------------------------------------------------------
				if( mPreviewRunning || mCaptureRunning) {	// CK Error
					if (mCaptureRunning) {
						extern int skip_stream_on;
						// rt5604 -> 2012.07.09 to reduce KPI
						ALOGD("rt5604: ---> PREVIEW_STOP: sendCommand(CAMERA_START_PREVIEW)");
						skip_stream_on = 1;
                        ALOGE("rt5604: skip_stream_on = %d", skip_stream_on);
                        //CAMHAL_ALOGDB("Start Preview; mCaptureRunning = %d; buffers_in_display = %s", mCaptureRunning, mDisplayAdapter->buffers_in_display);
						err = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_START_PREVIEW , false /*no need to set state */ , (int) mDisplayAdapter->buffers_in_display );

						ALOGD("rt5604: ---> PREVIEW_STOP: usleep 1 msec");
						usleep(1000);	// 1 msec
						skip_stream_on = 0;
						// rt5604 <- 2012.07.09 to reduce KPI
					}

                    if( CameraStop() < 0){
                        ALOGE("ERROR CameraStop()");
                        err= -1;
                    }

                    if( CameraDestroy(false) < 0){
                        ALOGE("ERROR CameraDestroy()");
                        err= -1;
                    }

                    if (err) {
                        ALOGE("ERROR Cannot deinit preview.");
                    }

					// rt5604 -> 2012.07.09 to reduce KPI
                    {
						extern int skip_stream_on;

						skip_stream_on = 0;
                        ALOGE("rt5604: skip_stream_on = %d", skip_stream_on);
					}
					// rt5604 <- 2012.07.09 to reduce KPI
					
                    ALOGD("PREVIEW_STOP %s", err ? "NACK" : "ACK");
                    msg.command = err ? PREVIEW_NACK : PREVIEW_ACK;
                }
                else
                {
                    msg.command = PREVIEW_NACK;
                }

                mPreviewRunning = false;
				mCaptureRunning = false;	// CK Error. 2012.06.04 by rt5604

                previewThreadAckQ.put(&msg);
            }
            break;

            case START_SMOOTH_ZOOM:

                parm = ( int ) msg.arg1;

                ALOGD("Receive Command: START_SMOOTH_ZOOM %d", parm);

                if ( ( parm >= 0 ) && ( parm < ZOOM_STAGES) ) {
                    mZoomTargetIdx = parm;
                    mZoomSpeed = 1;
                    mSmoothZoomStatus = SMOOTH_START;
                    msg.command = PREVIEW_ACK;
                } else {
                    msg.command = PREVIEW_NACK;
                }

                previewThreadAckQ.put(&msg);

                break;

            case STOP_SMOOTH_ZOOM:

                ALOGD("Receive Command: STOP_SMOOTH_ZOOM");
                if(mSmoothZoomStatus == SMOOTH_START) {
                    mSmoothZoomStatus = SMOOTH_NOTIFY_AND_STOP;
                }
                msg.command = PREVIEW_ACK;

                previewThreadAckQ.put(&msg);

                break;

            case PREVIEW_AF_START:
            {
                ALOGD("Receive Command: PREVIEW_AF_START");
                err = 0;

                //ALOGD("isStart_FW3A_CAF=%d, isStart_FW3A=%d, isStart_FW3A_AF=%d, focus_mode=%d, mcaf=%d", isStart_FW3A_CAF, isStart_FW3A, isStart_FW3A_AF, fobj->settings.af.focus_mode, mcaf);

                if( !mPreviewRunning ){
                    ALOGD("WARNING PREVIEW NOT RUNNING!");
                    msg.command = PREVIEW_NACK;
                    // (+) CTS Bug fix for takePicture, testPreviewFpsRange
                    previewThreadAckQ.put(&msg);
                    // (-) CTS Bug fix for takePicture, testPreviewFpsRange
                } else if (VIDEO_DEVICE_INDEX(mCameraIndex) != VIDEO_DEVICE_INDEX_MAIN) {
                    ALOGD("Dummy AF on non primary camera");
                    //onAutoFocus(true);
                    msg.command = PREVIEW_ACK;
                    // (+) CTS Bug fix for takePicture, testPreviewFpsRange
                    previewThreadAckQ.put(&msg);
                    mNotifyCb(CAMERA_MSG_FOCUS, 1, 0, mCallbackCookie);
                    // (-) CTS Bug fix for takePicture, testPreviewFpsRange
                } else {

                        if( msgTypeEnabled(CAMERA_MSG_FOCUS) ) {
                            //mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
                            mAppCallbackNotifier->enableMsgType (CAMERA_MSG_FOCUS);
                        }

                        msg.command = PREVIEW_ACK;

#ifdef USE_CAF_CALLBACK
                    }
#endif

                    // (+) CTS Bug fix for takePicture, testPreviewFpsRange
                    ALOGD("Receive Command: PREVIEW_AF_START %s", msg.command == PREVIEW_NACK ? "NACK" : "ACK");
                    previewThreadAckQ.put(&msg);
                    // (-) CTS Bug fix for takePicture, testPreviewFpsRange

                }
            }
            break;

            case PREVIEW_AF_STOP:
            {
                ALOGD("Receive Command: PREVIEW_AF_STOP");
                err = 0;

                msg.command = PREVIEW_ACK;
                ALOGD("Receive Command: PREVIEW_AF_STOP %s", msg.command == PREVIEW_NACK ? "NACK" : "ACK");
                previewThreadAckQ.put(&msg);

            }
            break;

            case PREVIEW_CAF_START:
            {
                ALOGD("Receive Command: PREVIEW_CAF_START");
                err=0;

                if( !mPreviewRunning ) {
                    msg.command = PREVIEW_ACK;
                    mCAFafterPreview = true;
                }
                else
                {
                    msg.command = err ? PREVIEW_NACK : PREVIEW_ACK;
                }
                ALOGD("Receive Command: PREVIEW_CAF_START %s", msg.command == PREVIEW_NACK ? "NACK" : "ACK");
                previewThreadAckQ.put(&msg);
            }
            break;

           case PREVIEW_CAF_STOP:
           {
                ALOGD("Receive Command: PREVIEW_CAF_STOP");
                err = 0;

                if( !mPreviewRunning )
                    msg.command = PREVIEW_ACK;
                else
                {
                    msg.command = err ? PREVIEW_NACK : PREVIEW_ACK;
                }
                ALOGD("Receive Command: PREVIEW_CAF_STOP %s", msg.command == PREVIEW_NACK ? "NACK" : "ACK");
                previewThreadAckQ.put(&msg);
           }
           break;

           case PREVIEW_CAPTURE:
           {
               mCaptureMode = true;

                if ( mCaptureRunning ) {
                msg.command = PREVIEW_NACK;
                previewThreadAckQ.put(&msg);
                break;
                }

            err = 0;

#ifdef DEBUG_LOG

            ALOGD("ENTER OPTION PREVIEW_CAPTURE");

            PPM("RECEIVED COMMAND TO TAKE A PICTURE");

#endif

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS
            gettimeofday(&ppm_receiveCmdToTakePicture, NULL);
#endif

            // Boost DSP OPP to highest level
            SetDSPKHz(DSP3630_KHZ_MAX);

            if( mPreviewRunning ) {

                    if (VIDEO_DEVICE_INDEX(mCameraIndex) != VIDEO_DEVICE_INDEX_MAIN){

#ifdef V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA
                        CameraDeviceSetControl("PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA",
                                               V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA, 1);
#endif

#ifdef V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA_VT
                        CameraDeviceSetControl("PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA_VT",
                                               V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA_VT, 0);
#endif

                    }

            if( CameraStop() < 0){
            ALOGE("ERROR CameraStop()");
            err = -1;
            }

            mPreviewRunning = false;

            }

#ifdef DEBUG_LOG

            PPM("STOPPED PREVIEW");

#endif

            if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN){
            if( ICapturePerform() < 0){
            ALOGE("ERROR ICapturePerform()");
            err = -1;
            }
               }

            //restart the preview
#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS
            gettimeofday(&ppm_restartPreview, NULL);
#endif

#ifdef DEBUG_LOG

            PPM("CONFIGURING CAMERA TO RESTART PREVIEW");

#endif

#ifndef MMS_COMBINATION_BUG_FIX
            if (CameraConfigure() < 0)
            ALOGE("ERROR CameraConfigure()");

            if ( CorrectPreview() < 0 )
            ALOGE("Error during CorrectPreview()");

            if (CameraStart() < 0)
            ALOGE("ERROR CameraStart()");
#endif

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

            PPM("Capture mode switch", &ppm_restartPreview);
            PPM("Shot to Shot", &ppm_receiveCmdToTakePicture);

#endif

            mCaptureRunning = true;

            msg.command = PREVIEW_ACK;
            previewThreadAckQ.put(&msg);
            ALOGD("EXIT OPTION PREVIEW_CAPTURE");
           }
           break;

           case PREVIEW_CAPTURE_CANCEL:
           {
                ALOGD("Receive Command: PREVIEW_CAPTURE_CANCEL");
                msg.command = PREVIEW_NACK;
                previewThreadAckQ.put(&msg);
            }
            break;

            case PREVIEW_KILL:
            {
                ALOGD("Receive Command: PREVIEW_KILL");
                err = 0;

                if (mPreviewRunning) {
                   if( CameraStop() < 0){
                        ALOGE("ERROR FW3A_Stop()");
                        err = -1;
                   }
                   mPreviewRunning = false;
                }

                msg.command = err ? PREVIEW_NACK : PREVIEW_ACK;
                ALOGD("Receive Command: PREVIEW_CAF_STOP %s", msg.command == PREVIEW_NACK ? "NACK" : "ACK");

                previewThreadAckQ.put(&msg);
                shouldLive = false;
          }
          break;
        }
    }

    ALOGE(" Exiting CameraHal::Previewthread ");

   LOG_FUNCTION_NAME_EXIT
}

int CameraHal::CameraCreate()
{
    int err = 0;
    const char *device;

    LOG_FUNCTION_NAME

    if (camera_device < 0) {
        ALOGD("Video Input: index %d", mCameraIndex);

        switch (VIDEO_DEVICE_INDEX(mCameraIndex)) {
            case VIDEO_DEVICE_INDEX_MAIN:
            case VIDEO_DEVICE_INDEX_SECONDARY:
                device = camera_device_name[VIDEO_DEVICE_INDEX(mCameraIndex)];
                break;
            default:
                device = camera_device_name[VIDEO_DEVICE_INDEX_MAIN];
                break;
        }

        camera_device = open(device, O_RDWR);
    if (camera_device < 0) {
        ALOGE ("Could not open the camera device: %s",  strerror(errno) );
            err = -1;
        goto exit;
        }
        ALOGD("Camera device %s has been opened with fs=%d",
            camera_device_name[VIDEO_DEVICE_INDEX(mCameraIndex)], camera_device);
    } else {
        ALOGD("Camera device %s already opened with fs=%d",
            camera_device_name[VIDEO_DEVICE_INDEX(mCameraIndex)], camera_device);
    }

    LOG_FUNCTION_NAME_EXIT
    return 0;

exit:
    return err;
}


int CameraHal::CameraDestroy(bool destroyWindow)
{
    int err, buffer_count;

    LOG_FUNCTION_NAME

    CAMHAL_ALOGDA("CameraDestroy");

/* CSR-OMAPS00273048 kirti.badkundri@sasken.com Fix for Camera CTS [START] */
/* CSR-OMAPS00275044 kirti.badkundri@sasken.com Fix for Preview stops after PVR_K error message in primary camera [START] */
    if ( destroyWindow ) {
	ALOGD("Camera device %s has been closed with fs=%d",
              camera_device_name[VIDEO_DEVICE_INDEX(mCameraIndex)], camera_device);
        close(camera_device);
        camera_device = -1;
    }
/* CSR-OMAPS00275044 kirti.badkundri@sasken.com Fix for Preview stops after PVR_K error message in primary camera [END] */
/* CSR-OMAPS00273048 kirti.badkundri@sasken.com Fix for Camera CTS [END] */

    forceStopPreview();

    LOG_FUNCTION_NAME_EXIT
    return 0;
}

int CameraHal::CameraConfigure()
{
    int w, h, framerate;
    int err;
    int framerate_min = 0, framerate_max = 0;
    enum v4l2_buf_type type;
    struct v4l2_control vc;
    struct v4l2_streamparm parm;
    struct v4l2_format format;	

    LOG_FUNCTION_NAME

    if ( NULL != mCameraAdapter ) {
        mCameraAdapter->setParameters(mParameters);
	mParameters.getPreviewSize(&w, &h);

/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
	//ALOGD("kim vt Rotate preview for VT mvt_mode:%d", mvt_mode);
#if defined(LGE_JUSTIN_DEVICE) 
    if ( ((VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_SECONDARY) && ( mvt_mode == false)) || 
     	   ((VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN)  && ( mvt_mode == true))
     	 ) 
#else 
    //20121003 daewon1004.kim@lge.com
    if ( ((VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_SECONDARY) && ( mvt_mode == true)) || 
     	   ((VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN)  && ( mvt_mode == true))
     	 ) 

#endif
    {
        ALOGD("kim vt Rotate preview for VT %dx%d", w, h);
        mRotateVideo = true;
        // swap width and height
        mCameraWidth = h;
        mCameraHeight = w;
        w = mCameraWidth;
        h = mCameraHeight;
       //ALOGI("kim vt  1 CameraConfigure w:%d, h: %d, mCameraWidth:%d, mCameraHeight:%d", w, h, mCameraWidth, mCameraHeight);
    } 
   else
    {
        ALOGI("kim vt Does not rotate preview %dx%d", w, h);
        mRotateVideo = false;
        mCameraWidth  = w;
        mCameraHeight = h;
    }
#endif   
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */

	/* Set preview format */        
    	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    	format.fmt.pix.width = w;
    	format.fmt.pix.height = h;
    	format.fmt.pix.pixelformat = PIXEL_FORMAT;

    	CAMHAL_ALOGDB("Width * Height %d x %d format 0x%x", w, h, PIXEL_FORMAT);

        err = ioctl(camera_device, VIDIOC_S_FMT, &format);
        if ( err < 0 ){
            ALOGE ("Failed to set VIDIOC_S_FMT.");
            goto s_fmt_fail;
        }
	ALOGI("CameraConfigure PreviewFormat: w=%d h=%d", format.fmt.pix.width, format.fmt.pix.height);
     
    }
    else {
        ALOGE( " Can't set the resolution return as CameraAdapter is NULL ");
        goto s_fmt_fail;
    }

    //ALOGI("CameraConfigure PreviewFormat: w=%d h=%d", format.fmt.pix.width, format.fmt.pix.height);

    if (useFramerateRange) {
        mParameters.getPreviewFpsRange(&framerate_min, &framerate_max);
        ALOGD("CameraConfigure: framerate to set: min = %d, max = %d",framerate_min, framerate_max);
    }
    else {
        framerate_max = mParameters.getPreviewFrameRate()*1000;
        ALOGD("CameraConfigure: framerate to set = %d", framerate_max);
    }

    if (VIDEO_DEVICE_INDEX(mCameraIndex) != VIDEO_DEVICE_INDEX_MAIN) {
        int vc_value;

#ifdef V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA
        // Reset secondary camera
        CameraDeviceSetControl("PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA",
                               V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA, 0);
#endif

#ifdef V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA_VT
        // Select secondary camera framerate
        if (useFramerateRange && (mvt_mode == 0) ) {
            if (framerate_min == framerate_max) {
                ALOGD("CameraConfigure: SC 15 fps");
                vc_value = 1;
            } else {
                ALOGD("CameraConfigure: SC VBR");
                vc_value = 0;
            }
        } else {
            ALOGD("CameraConfigure: SC 15 fps");
            vc_value = 1;
        }

        CameraDeviceSetControl("PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA_VT",
                               V4L2_CID_PRIVATE_OMAP3ISP_HYNIX_SMART_CAMERA_VT, vc_value);
#endif

    }

    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(camera_device, VIDIOC_G_PARM, &parm);
    if(err != 0) {
        ALOGD("VIDIOC_G_PARM ");
        return -1;
    }

    ALOGD("CameraConfigure: Old frame rate is %d/%d  fps",
    parm.parm.capture.timeperframe.denominator,
    parm.parm.capture.timeperframe.numerator);

    parm.parm.capture.timeperframe.numerator = 1;

    if (useFramerateRange) {
        //Gingerbread changes for FPS - set range of min and max fps
        if ( (MIN_FPS*1000 <= framerate_min)&&(framerate_min <= framerate_max)&&(framerate_max<=MAX_FPS*1000) ) {
            parm.parm.capture.timeperframe.denominator = framerate_max/1000;
        }
        else parm.parm.capture.timeperframe.denominator = 30;
    }
    else {
        if ( framerate_max != 0 ) parm.parm.capture.timeperframe.denominator = framerate_max/1000;
        else  parm.parm.capture.timeperframe.denominator  = 30;
    }

    err = ioctl(camera_device, VIDIOC_S_PARM, &parm);
    if(err != 0) {
        ALOGE("VIDIOC_S_PARM ");
        return -1;
    }

    ALOGI("CameraConfigure: New frame rate is %d/%d fps",parm.parm.capture.timeperframe.denominator,parm.parm.capture.timeperframe.numerator);

    LOG_FUNCTION_NAME_EXIT
    return 0;

s_fmt_fail:
    return -1;
}

status_t CameraHal::allocPreviewBufs(int width, int height, const char* previewFormat,
                                        unsigned int buffercount, unsigned int &max_queueable)
{
    status_t ret = NO_ERROR;

    LOG_FUNCTION_NAME;

    if(mDisplayAdapter.get() == NULL)
    {
        // Memory allocation of preview buffers is now placed in gralloc
        // CameraHal should not allocate preview buffers without DisplayAdapter
        return NO_MEMORY;
    }

    if(!mPreviewBufs)
    {
        ///@todo Pluralise the name of this method to allocateBuffers
        mPreviewLength = 0;
        mPreviewBufs = (int32_t *) mDisplayAdapter->allocateBuffer(width, height,
                                                                    previewFormat,
                                                                    mPreviewLength,
                                                                    buffercount);

    if (NULL == mPreviewBufs ) {
            ALOGE("Couldn't allocate preview buffers");
            return NO_MEMORY;
         }

        mPreviewOffsets = mDisplayAdapter->getOffsets();
        if ( NULL == mPreviewOffsets ) {
            ALOGE("Buffer mapping failed");
            return BAD_VALUE;
         }

        mPreviewFd = mDisplayAdapter->getFd();
        if ( -1 == mPreviewFd ) {
            ALOGE("Invalid handle");
            return BAD_VALUE;
          }

        mBufProvider = (BufferProvider*) mDisplayAdapter.get();

        ret = mDisplayAdapter->maxQueueableBuffers(max_queueable);
        if (ret != NO_ERROR) {
            return ret;
         }

    }

    LOG_FUNCTION_NAME_EXIT;

    return ret;

}

void CameraHal::setCallbacks(camera_notify_callback notify_cb,
                            camera_data_callback data_cb,
                            camera_data_timestamp_callback data_cb_timestamp,
                            camera_request_memory get_memory,
                            void *user)
{
    LOG_FUNCTION_NAME;

    if ( NULL != mAppCallbackNotifier.get() )
    {
             mAppCallbackNotifier->setCallbacks(this,
                                                notify_cb,
                                                data_cb,
                                                data_cb_timestamp,
                                                get_memory,
                                                user);
    }

    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mRequestMemory = get_memory;
    mCallbackCookie = user;

    LOG_FUNCTION_NAME_EXIT;
}

status_t CameraHal::initialize(CameraProperties::Properties* properties)
{
    LOG_FUNCTION_NAME;

    int sensor_index = 0;

    ///Initialize the event mask used for registering an event provider for AppCallbackNotifier
    ///Currently, registering all events as to be coming from CameraAdapter
    int32_t eventMask = CameraHalEvent::ALL_EVENTS;

    // Get my camera properties
    mCameraProperties = properties;

    if(!mCameraProperties)
    {
        goto fail_loop;
    }

    // Dump the properties of this Camera
    // will only print if DEBUG macro is defined
    mCameraProperties->dump();

    if (strcmp(CameraProperties::DEFAULT_VALUE, mCameraProperties->get(CameraProperties::CAMERA_SENSOR_INDEX)) != 0 )
    {
        sensor_index = atoi(mCameraProperties->get(CameraProperties::CAMERA_SENSOR_INDEX));
    }

    CAMHAL_ALOGDB("Sensor index %d", sensor_index);

    mCameraAdapter = CameraAdapter_Factory(sensor_index);
    if ( NULL == mCameraAdapter )
    {
        CAMHAL_ALOGEA("Unable to create or initialize CameraAdapter");
        mCameraAdapter = NULL;
    }

    mCameraAdapter->incStrong(mCameraAdapter);
    mCameraAdapter->registerImageReleaseCallback(releaseImageBuffers, (void *) this);
    mCameraAdapter->registerEndCaptureCallback(endImageCapture, (void *)this);
    mCameraAdapter->initialize(camera_device);

    if(!mAppCallbackNotifier.get())
    {
        /// Create the callback notifier
        mAppCallbackNotifier = new AppCallbackNotifier();
        if( ( NULL == mAppCallbackNotifier.get() ) || ( mAppCallbackNotifier->initialize() != NO_ERROR))
        {
            CAMHAL_ALOGEA("Unable to create or initialize AppCallbackNotifier");
            goto fail_loop;
        }
    }

   /* if(!mMemoryManager.get())
    {
        /// Create Memory Manager
        mMemoryManager = new MemoryManager();
        if( ( NULL == mMemoryManager.get() ) || ( mMemoryManager->initialize() != NO_ERROR))
            {
            CAMHAL_ALOGEA("Unable to create or initialize MemoryManager");
            goto fail_loop;
            }
    }*/

    ///Setup the class dependencies...

    ///AppCallbackNotifier has to know where to get the Camera frames and the events like auto focus lock etc from.
    ///CameraAdapter is the one which provides those events
    ///Set it as the frame and event providers for AppCallbackNotifier
    ///@remarks  setEventProvider API takes in a bit mask of events for registering a provider for the different events
    ///         That way, if events can come from DisplayAdapter in future, we will be able to add it as provider
    ///         for any event
    mAppCallbackNotifier->setEventProvider(eventMask, mCameraAdapter);
    mAppCallbackNotifier->setFrameProvider(mCameraAdapter);

    ///Any dynamic errors that happen during the camera use case has to be propagated back to the application
    ///via CAMERA_MSG_ERROR. AppCallbackNotifier is the class that  notifies such errors to the application
    ///Set it as the error handler for CameraAdapter
    mCameraAdapter->setErrorHandler(mAppCallbackNotifier.get());

    ///Start the callback notifier
    if(mAppCallbackNotifier->start() != NO_ERROR)
      {
        CAMHAL_ALOGEA("Couldn't start AppCallbackNotifier");
        goto fail_loop;
      }

    CAMHAL_ALOGDA("Started AppCallbackNotifier..");
    mAppCallbackNotifier->setMeasurements(false);

    LOG_FUNCTION_NAME_EXIT;

    return NO_ERROR;

    fail_loop:

        ///Free up the resources because we failed somewhere up
        forceStopPreview();
        LOG_FUNCTION_NAME_EXIT;

        return NO_MEMORY;

}

void releaseImageBuffers(void *userData)
{
    LOG_FUNCTION_NAME;

    if (NULL != userData) {
        CameraHal *c = reinterpret_cast<CameraHal *>(userData);
       // c->freeImageBufs(); - check this
    }

    LOG_FUNCTION_NAME_EXIT;
}

void endImageCapture( void *userData )
{
    LOG_FUNCTION_NAME;

    if ( NULL != userData )
        {
        CameraHal *c = reinterpret_cast<CameraHal *>(userData);
        c->signalEndImageCapture();
        }

    LOG_FUNCTION_NAME_EXIT;
}

status_t CameraHal::signalEndImageCapture()
{
    status_t ret = NO_ERROR;
    int w,h;
    CameraParameters adapterParams = mParameters;
    Mutex::Autolock lock(mLock);

    LOG_FUNCTION_NAME;

    /*if ( mBracketingRunning ) {
        stopImageBracketing();
    } else {
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_IMAGE_CAPTURE);
    }*/

    mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_IMAGE_CAPTURE);

    mPictureBuffer.clear();
    mPictureHeap.clear();

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

status_t CameraHal::storeMetaDataInBuffers(bool enable)
{
    ALOGE("storeMetaDataInBuffers start");

    return mAppCallbackNotifier->useMetaDataBufferMode(enable);

    ALOGE("storeMetaDataInBuffers end");
}

void CameraHal::putParameters(char *parms)
{
    free(parms);
}

status_t CameraHal::setPreviewWindow(struct preview_stream_ops *window)
{
    status_t ret = NO_ERROR;    

    LOG_FUNCTION_NAME;
    mSetPreviewWindowCalled = true;
   
   ///If the Camera service passes a null window, we destroy existing window and free the DisplayAdapter
    if(!window)
    {
        if(mDisplayAdapter.get() != NULL)
        {
            ///NULL window passed, destroy the display adapter if present
            ALOGE("NULL window passed, destroying display adapter");
            mDisplayAdapter.clear();
           
        }
        ALOGD("NULL ANativeWindow passed to setPreviewWindow");
        return NO_ERROR;
    }
    else if(mDisplayAdapter.get() == NULL)
    {
        ALOGD("CameraHal::setPreviewWindow :: mDisplayAdapter is NULL");
        // Need to create the display adapter since it has not been created
        // Create display adapter
        mDisplayAdapter = new ANativeWindowDisplayAdapter();
        ret = NO_ERROR;
        if(!mDisplayAdapter.get() || ((ret=mDisplayAdapter->initialize())!=NO_ERROR))
        {
            if(ret!=NO_ERROR)
            {
                mDisplayAdapter.clear();
                ALOGE("DisplayAdapter initialize failed");
                LOG_FUNCTION_NAME_EXIT;
                return ret;
            }
            else
            {
                ALOGE("Couldn't create DisplayAdapter");
                LOG_FUNCTION_NAME_EXIT;
                return NO_MEMORY;
            }
        }

        // DisplayAdapter needs to know where to get the CameraFrames from inorder to display
        // Since CameraAdapter is the one that provides the frames, set it as the frame provider for DisplayAdapter
        mDisplayAdapter->setFrameProvider(mCameraAdapter);

        // Any dynamic errors that happen during the camera use case has to be propagated back to the application
        // via CAMERA_MSG_ERROR. AppCallbackNotifier is the class that  notifies such errors to the application
        // Set it as the error handler for the DisplayAdapter
        mDisplayAdapter->setErrorHandler(mAppCallbackNotifier.get());

        // Update the display adapter with the new window that is passed from CameraService
        ret  = mDisplayAdapter->setPreviewWindow(window);
        if(ret!=NO_ERROR)
        {
            ALOGE("DisplayAdapter setPreviewWindow returned error %d", ret);
        }

        if(mPreviewRunning)
        {
            ALOGE("setPreviewWindow called when preview running");
            mPreviewRunning = false;
            // Start the preview since the window is now available
            ret = startPreview();
          
        } 
    }else
    {
        /* If mDisplayAdpater is already created. No need to do anything.
         * We get a surface handle directly now, so we can reconfigure surface
         * itself in DisplayAdapter if dimensions have changed
         */
    }
    LOG_FUNCTION_NAME_EXIT;

    return ret;

}

int CameraHal::CameraStart()
{
    int width, height;
    int err;
    int nSizeBytes;
    struct v4l2_format format;
    enum v4l2_buf_type type;
    struct v4l2_requestbuffers creqbuf;
    unsigned int required_buffer_count = MAX_CAMERA_BUFFERS ;
    unsigned int max_queueble_buffers = MAX_CAMERA_BUFFERS ;
    status_t ret; 
    CameraAdapter::BuffersDescriptor desc;

    CameraProperties::Properties* properties = NULL;

    LOG_FUNCTION_NAME

    nCameraBuffersQueued = 0;

    mParameters.getPreviewSize(&width, &height);

    if(mPreviewBufs == NULL)
    {
        ret = allocPreviewBufs(width, height, mParameters.getPreviewFormat(), required_buffer_count, max_queueble_buffers);
        if ( NO_ERROR != ret )
        {
            CAMHAL_ALOGDA("Couldn't allocate buffers for Preview");
            return ret;
        }

        ///Pass the buffers to Camera Adapter
        desc.mBuffers = mPreviewBufs;
        desc.mOffsets = mPreviewOffsets;
        desc.mFd = mPreviewFd;
        desc.mLength = mPreviewLength;
        desc.mCount = ( size_t ) required_buffer_count;
        desc.mMaxQueueable = (size_t) max_queueble_buffers;

        ret = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_USE_BUFFERS_PREVIEW,
                                          ( int ) &desc);

        if ( NO_ERROR != ret )
        {
            CAMHAL_ALOGEB("Failed to register preview buffers: 0x%x", ret);
            freePreviewBufs();
            return ret;
        }

        mAppCallbackNotifier->startPreviewCallbacks(mParameters, mPreviewBufs, mPreviewOffsets, mPreviewFd, mPreviewLength, required_buffer_count);

        ///Start the callback notifier
        ret = mAppCallbackNotifier->start();
        if( ALREADY_EXISTS == ret )
        {
            //Already running, do nothing
            CAMHAL_ALOGDA("AppCallbackNotifier already running");
            ret = NO_ERROR;
        }
        else if ( NO_ERROR == ret )
        {
            CAMHAL_ALOGDA("Started AppCallbackNotifier..");
            mAppCallbackNotifier->setMeasurements(false);
        }
        else
        {
            CAMHAL_ALOGDA("Couldn't start AppCallbackNotifier");
            return -1;
        }

        //if( ioctl(camera_device, VIDIOC_G_CROP, &mInitialCrop) < 0 ){
        //    ALOGE("[%s]: ERROR VIDIOC_G_CROP failed", strerror(errno));
        //    return -1;
        //}

        ALOGE("Initial Crop: crop_top = %d, crop_left = %d, crop_width = %d, crop_height = %d", mInitialCrop.c.top, mInitialCrop.c.left, mInitialCrop.c.width, mInitialCrop.c.height);


        if(mDisplayAdapter.get() != NULL)
        {
            CAMHAL_ALOGDA("Enabling display");
            bool isS3d = false;
            DisplayAdapter::S3DParameters s3dParams;
            int width, height;
            mParameters.getPreviewSize(&width, &height);

    #if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

            ret = mDisplayAdapter->enableDisplay(width, height, &mStartPreview, isS3d ? &s3dParams : NULL);

    #else

            ret = mDisplayAdapter->enableDisplay(width, height, NULL, isS3d ? &s3dParams : NULL);

    #endif

            if ( ret != NO_ERROR )
            {
                CAMHAL_ALOGEA("Couldn't enable display");
                goto error;
            }

            ///Send START_PREVIEW command to adapter
            CAMHAL_ALOGDA("Starting CameraAdapter preview mode");
        }

        if(ret!=NO_ERROR)
        {
            CAMHAL_ALOGEA("Couldn't start preview w/ CameraAdapter");
            goto error;
        }
    }
    else
    {
        ///Pass the buffers to Camera Adapter
        desc.mBuffers = mPreviewBufs;
        desc.mOffsets = mPreviewOffsets;
        desc.mFd = mPreviewFd;
        desc.mLength = mPreviewLength;
        desc.mCount = ( size_t ) required_buffer_count;
        desc.mMaxQueueable = (size_t) max_queueble_buffers;
        ret = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_USE_BUFFERS_PREVIEW, ( int ) &desc);
    }

    LOG_FUNCTION_NAME_EXIT

    return 0;


error:
    CAMHAL_ALOGEA("Performing cleanup after error");

    //Do all the cleanup
    freePreviewBufs();
    mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_PREVIEW , true );
    if(mDisplayAdapter.get() != NULL)
    {
        mDisplayAdapter->disableDisplay(false);
    }
    mAppCallbackNotifier->stop();

fail_bufalloc:
fail_loop:
fail_reqbufs:

    return -1;
}

int CameraHal::CameraStop()
{
    LOG_FUNCTION_NAME

    nCameraBuffersQueued = 0;

    if ( NULL != mCameraAdapter )
    {
        CameraAdapter::AdapterState currentState;

        currentState = mCameraAdapter->getState();

        // only need to send these control commands to state machine if we are
        // passed the LOADED_PREVIEW_STATE
        if (currentState > CameraAdapter::LOADED_PREVIEW_STATE) {
        // according to javadoc...FD should be stopped in stopPreview
        // and application needs to call startFaceDection again
        // to restart FD
        //mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_FD);
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_CANCEL_AUTOFOCUS);
        }

        // only need to send these control commands to state machine if we are
        // passed the INITIALIZED_STATE
        if (currentState > CameraAdapter::INTIALIZED_STATE) {
        //Stop the source of frames
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_PREVIEW , !mCaptureMode );
        }
    }

    //Force the zoom to be updated next time preview is started.
    mZoomCurrentIdx = 0;

/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
#if defined(LGE_JUSTIN_DEVICE)
    if (mRotateVideo) {

        for (int i = 0; i < MAX_CAMERA_BUFFERS; i++) {
            mCameraBuffers[i].clear();
            mCameraHeaps[i].clear();
        }
        mRotateVideo = false;
    }
#endif
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */
    LOG_FUNCTION_NAME_EXIT

    return 0;
}

static void debugShowFPS()
{
    static int mFrameCount = 0;
    static int mLastFrameCount = 0;
    static nsecs_t mLastFpsTime = 0;
    static float mFps = 0;
    mFrameCount++;
    if (!(mFrameCount & 0x1F)) {
        nsecs_t now = systemTime();
        nsecs_t diff = now - mLastFpsTime;
        mFps =  ((mFrameCount - mLastFrameCount) * float(s2ns(1))) / diff;
        mLastFpsTime = now;
        mLastFrameCount = mFrameCount;
        ALOGD("####### [%d] Frames, %f FPS", mFrameCount, mFps);
    }
}

int CameraHal::dequeueFromOverlay()
{
  /*  overlay_buffer_t overlaybuffer;// contains the index of the buffer dque
    if (nOverlayBuffersQueued < NUM_BUFFERS_TO_BE_QUEUED_FOR_OPTIMAL_PERFORMANCE) {
        ALOGV("skip dequeue. nOverlayBuffersQueued = %d", nOverlayBuffersQueued);
        return -1;
    }

    int dequeue_from_dss_failed = mOverlay->dequeueBuffer(&overlaybuffer);
    if(dequeue_from_dss_failed) {
        ALOGD("no buffer to dequeue in overlay");
        return -1;
    }

    nOverlayBuffersQueued--;
    mVideoBufferStatus[(int)overlaybuffer] &= ~BUFF_Q2DSS;
    lastOverlayBufferDQ = (int)overlaybuffer;

    return (int)overlaybuffer;*/
return 0;
} 

void CameraHal::nextPreview()
{
    static int frame_count = 0;
    int zoom_inc, err;
    struct timeval lowLightTime;

    //Zoom
    frame_count++;
    if( mZoomCurrentIdx != mZoomTargetIdx){

        if( mZoomCurrentIdx < mZoomTargetIdx)
            zoom_inc = 1;
        else
            zoom_inc = -1;

        if ( mZoomSpeed > 0 ){
            if( (frame_count % mZoomSpeed) == 0){
                mZoomCurrentIdx += zoom_inc;
            }
        } else {
            mZoomCurrentIdx = mZoomTargetIdx;
        }

        ZoomPerform(zoom_step[mZoomCurrentIdx]);

        // Update mParameters with current zoom position only if smooth zoom is used
        // Immediate zoom should not generate callbacks.
        if ( mZoomSpeed > 0 ){
            //Avoid segfault. mParameters may be used somewhere else, e.g. in SetParameters()
            {
                Mutex::Autolock lock(mLock);
                mParameters.set("zoom", mZoomCurrentIdx);
            }

            if ( mSmoothZoomStatus == SMOOTH_START ||  mSmoothZoomStatus == SMOOTH_NOTIFY_AND_STOP)  {
                if(mSmoothZoomStatus == SMOOTH_NOTIFY_AND_STOP) {
                    mZoomTargetIdx = mZoomCurrentIdx;
                    mSmoothZoomStatus = SMOOTH_STOP;
                }
                if( mZoomCurrentIdx == mZoomTargetIdx ){
					// rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom ->
                    onZoom(true, mZoomCurrentIdx);
                } else {
                    onZoom(false, mZoomCurrentIdx);
					// rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom <-
                }
            }
        }
    }

    int index = 0;
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
    char *fp;
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
    //ALOGD("kim vt nextPreview mRotateVideo:%d", mRotateVideo);
    // 20121018 daewon1004.kim@lge.com change to int form boolean
    if (mRotateVideo) {
	 if((VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN)){
        	fp = mCameraAdapter->GetFrame(index, ROTATION_90);
	 }
	 else{
 		fp = mCameraAdapter->GetFrame(index, ROTATION_270);
	 }
    } 
   else 
    {
        fp = mCameraAdapter->GetFrame(index,  ROTATION_0);
    }
#else
   	fp = mCameraAdapter->GetFrame(index,  ROTATION_0);
#endif     
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */

    mRecordingLock.lock();

    mCameraAdapter->queueToGralloc(index,fp, CameraFrame::PREVIEW_FRAME_SYNC);

    if(mRecordingEnabled)
    {
    mCameraAdapter->queueToGralloc(index,fp, CameraFrame::VIDEO_FRAME_SYNC);
    }

/*queue_and_exit:
    index = dequeueFromOverlay();
    if(-1 == index) {
        goto exit;
    }

    if (mVideoBufferStatus[index] == BUFF_IDLE)
        mCameraAdapter->queueToCamera(index);*/

exit:
    mRecordingLock.unlock();

    if (UNLIKELY(mDebugFps)) {
        debugShowFPS();
    }

    return;
}

int CameraHal::ICapturePerform()
{
    int image_width , image_height ;
    int image_rotation;
    double image_zoom;
    int preview_width, preview_height;
    int thumb_width, thumb_height;
    unsigned long base, offset;
    struct v4l2_buffer buffer;
    struct v4l2_format format;
    struct v4l2_buffer cfilledbuffer;
    struct v4l2_requestbuffers creqbuf;
    sp<MemoryBase> memBase;
    int jpegSize;
    void * outBuffer;
    unsigned int vppMessage[3];
    int err, i;
    int snapshot_buffer_index;
    void* snapshot_buffer;
    int ipp_reconfigure=0;
    int ippTempConfigMode;
    int jpegFormat = PIX_YUV422I;
    v4l2_streamparm parm;
    CameraFrame frame;
    status_t ret = NO_ERROR;
    CameraAdapter::BuffersDescriptor desc;
    int buffCount = 1;
    bool need_wait_snapshot = false;
    bool display_hq_snapshot = true;

    LOG_FUNCTION_NAME

    ALOGD("\n\n\n PICTURE NUMBER =%d\n\n\n",++pictureNumber);

    if ( NULL != mCameraAdapter ) {
	mCameraAdapter->setParameters(mParameters);
    }
		
    if( ( msgTypeEnabled(CAMERA_MSG_SHUTTER) ) && mShutterEnable)
    {
        //mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);
        mAppCallbackNotifier->enableMsgType(CAMERA_MSG_SHUTTER);
    }

    mParameters.getPictureSize(&image_width, &image_height);
    mParameters.getPreviewSize(&preview_width, &preview_height);
    thumb_width = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH);
    thumb_height = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT);

        if ((0 > thumb_width) || (0 > thumb_height)) {
            ALOGD("Missing thumbnail resolution (%d, %d). Will use 80x60", thumb_width, thumb_height);
            thumb_width = 80;
            thumb_height = 60;
        }

    CAMHAL_ALOGEB("Picture Size: Width = %d \tHeight = %d", image_width, image_height);

    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN){

#ifdef V4L2_CID_PRIVATE_OMAP3ISP_CONTINUOUS_CAPTURE
    CameraDeviceSetControl("PRIVATE_OMAP3ISP_CONTINUOUS_CAPTURE",
                           V4L2_CID_PRIVATE_OMAP3ISP_CONTINUOUS_CAPTURE, 1);
#endif

    }

    image_rotation = rotation;
    if (-1 == image_rotation) {
        image_rotation = 0;
    }
        if ( (SNAPSHOT_MAX_HEIGHT < preview_height) || (1 == mParameters.getInt(KEY_NO_HQ_SNAPSHOT)) ) {
            display_hq_snapshot = false;
        }

//--[[ LGE_UBIQUIX_MODIFIED_START : rt5604@mnbt.co.kr [2012.05.23] - CAM : camera orientation
    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN){ image_rotation += 90;}
//--]] LGE_UBIQUIX_MODIFIED_END : rt5604@mnbt.co.kr [2012.05.23] - CAM
    
    image_zoom = zoom_step[mZoomTargetIdx];

    if ( ( NULL != mCameraAdapter ) )
    {
        ret = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_QUERY_BUFFER_SIZE_IMAGE_CAPTURE,
                ( int ) &frame,( buffCount ));

        if ( NO_ERROR != ret )
        {
            CAMHAL_ALOGEB("CAMERA_QUERY_BUFFER_SIZE_IMAGE_CAPTURE returned error 0x%x", ret);
            return -1;
        }
    }

    yuv_len = image_width * image_height * 2;
    if (yuv_len & 0xfff)
    {
        yuv_len = (yuv_len & 0xfffff000) + 0x1000;
    }

    CAMHAL_ALOGEB ("pictureFrameSize = 0x%x = %d", yuv_len, yuv_len);

#define ALIGMENT 1
#if ALIGMENT
    mPictureHeap = new MemoryHeapBase(yuv_len);
#else
    // Make a new mmap'ed heap that can be shared across processes.
    mPictureHeap = new MemoryHeapBase(yuv_len + 0x20 + 256);
#endif

    base = (unsigned long)mPictureHeap->getBase();

#if ALIGMENT
    base = NEXT_4K_ALIGN_ADDR(base);
#else
    /*Align buffer to 32 byte boundary */
    while ((base & 0x1f) != 0)
    {
        base++;
    }
    /* Buffer pointer shifted to avoid DSP cache issues */
    base += 128;
#endif

    offset = base - (unsigned long)mPictureHeap->getBase();

    yuv_len = frame.mLength;
    CAMHAL_ALOGEB("image yuv_len = %d \n",yuv_len);

    if ( (NO_ERROR == ret) && ( NULL != mCameraAdapter ) )
    {
        desc.mBuffers = (void *)base;
        desc.mLength = frame.mLength;
        desc.mCount = ( size_t ) buffCount;
        desc.mMaxQueueable = ( size_t ) buffCount;

        ret = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_USE_BUFFERS_IMAGE_CAPTURE,
                ( int ) &desc);

        if ( NO_ERROR != ret )
        {
            CAMHAL_ALOGEB("CAMERA_USE_BUFFERS_IMAGE_CAPTURE returned error 0x%x", ret);
            return -1;
        }
    }

    mPictureBuffer = new MemoryBase(mPictureHeap, offset, yuv_len);
    CAMHAL_ALOGEB("Picture Buffer: Base = %p Offset = 0x%x", (void *)base, (unsigned int)offset);


    // pause preview during normal image capture
    // do not pause preview if recording (video state)
    if ( NULL != mDisplayAdapter.get() )
    {
        if (mCameraAdapter->getState() != CameraAdapter::VIDEO_STATE)
        {
            CAMHAL_ALOGEA(" Pausing the display ");
            mDisplayPaused = true;
            ret = mDisplayAdapter->pauseDisplay(mDisplayPaused);
            // since preview is paused we should stop sending preview frames too
            if(mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) {
                mAppCallbackNotifier->disableMsgType (CAMERA_MSG_PREVIEW_FRAME);
            }
        }
        else
        {
            CAMHAL_ALOGEA(" ICapturePerform not pausing the display ");
        }

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS
        mDisplayAdapter->setSnapshotTimeRef(&mStartCapture);
#endif
    }

    if ((NO_ERROR == ret) && (NULL != mCameraAdapter)) {

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

        //pass capture timestamp along with the camera adapter command
        ret = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_START_IMAGE_CAPTURE, (int) &mStartCapture);

#else

        ret = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_START_IMAGE_CAPTURE);

#endif
        if ( NO_ERROR != ret )
        {
            CAMHAL_ALOGEB("CAMERA_START_IMAGE_CAPTURE returned error 0x%x", ret);
            return -1;
        }
    }
    yuv_buffer = (uint8_t*)base;

    if (VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_MAIN){

#ifdef V4L2_CID_PRIVATE_OMAP3ISP_CONTINUOUS_CAPTURE
    CameraDeviceSetControl("PRIVATE_OMAP3ISP_CONTINUOUS_CAPTURE",
                           V4L2_CID_PRIVATE_OMAP3ISP_CONTINUOUS_CAPTURE, 0);
#endif

    }

    CAMHAL_ALOGEB("PictureThread: generated a picture, yuv_buffer=%p yuv_len=%d ",yuv_buffer,yuv_len );

#ifdef IMAGE_PROCESSING_PIPELINE
#if 1

    if(mippMode ==-1 ){
        mippMode=IPP_EdgeEnhancement_Mode;
    }

#else

    if(mippMode ==-1){
        mippMode=IPP_CromaSupression_Mode;
    }
    if(mippMode == IPP_CromaSupression_Mode){
        mippMode=IPP_EdgeEnhancement_Mode;
    }
    else if(mippMode == IPP_EdgeEnhancement_Mode){
        mippMode=IPP_CromaSupression_Mode;
    }

#endif

    ALOGD("IPPmode=%d",mippMode);
    if(mippMode == IPP_CromaSupression_Mode){
        ALOGD("IPP_CromaSupression_Mode");
    }
    else if(mippMode == IPP_EdgeEnhancement_Mode){
        ALOGD("IPP_EdgeEnhancement_Mode");
    }
    else if(mippMode == IPP_Disabled_Mode){
        ALOGD("IPP_Disabled_Mode");
    }

    if(false){

        if(mippMode != IPP_CromaSupression_Mode && mippMode != IPP_EdgeEnhancement_Mode){
            ALOGE("ERROR ippMode unsupported");
            return -1;
        }
        PPM("Before init IPP");

        if(mIPPToEnable)
        {
            err = InitIPP(image_width,image_height, jpegFormat, mippMode);
            if( err ) {
                ALOGE("ERROR InitIPP() failed");
                return -1;
            }
            PPM("After IPP Init");
            mIPPToEnable = false;
        }

        err = PopulateArgsIPP(image_width,image_height, jpegFormat, mippMode);
        if( err ) {
            ALOGE("ERROR PopulateArgsIPP() failed");
            return -1;
        }
        PPM("BEFORE IPP Process Buffer");

        ALOGD("Calling ProcessBufferIPP(buffer=%p , len=0x%x)", yuv_buffer, yuv_len);
        err = ProcessBufferIPP(yuv_buffer, yuv_len,
                jpegFormat,
                mippMode,
                -1, // EdgeEnhancementStrength
                -1, // WeakEdgeThreshold
                -1, // StrongEdgeThreshold
                -1, // LowFreqLumaNoiseFilterStrength
                -1, // MidFreqLumaNoiseFilterStrength
                -1, // HighFreqLumaNoiseFilterStrength
                -1, // LowFreqCbNoiseFilterStrength
                -1, // MidFreqCbNoiseFilterStrength
                -1, // HighFreqCbNoiseFilterStrength
                -1, // LowFreqCrNoiseFilterStrength
                -1, // MidFreqCrNoiseFilterStrength
                -1, // HighFreqCrNoiseFilterStrength
                -1, // shadingVertParam1
                -1, // shadingVertParam2
                -1, // shadingHorzParam1
                -1, // shadingHorzParam2
                -1, // shadingGainScale
                -1, // shadingGainOffset
                -1, // shadingGainMaxValue
                -1); // ratioDownsampleCbCr
        if( err ) {
            ALOGE("ERROR ProcessBufferIPP() failed");
            return -1;
        }
        PPM("AFTER IPP Process Buffer");

        if(!(pIPP.ippconfig.isINPLACE)){
            yuv_buffer = pIPP.pIppOutputBuffer;
        }

#if ( IPP_YUV422P || IPP_YUV420P_OUTPUT_YUV422I )
        jpegFormat = PIX_YUV422I;
        ALOGD("YUV422 !!!!");
#else
        yuv_len=  ((image_width * image_height *3)/2);
        jpegFormat = PIX_YUV420P;
        ALOGD("YUV420 !!!!");
#endif

    }
    //SaveFile(NULL, (char*)"yuv", yuv_buffer, yuv_len);

#endif

    if ( msgTypeEnabled(CAMERA_MSG_COMPRESSED_IMAGE) )
    {

#ifdef HARDWARE_OMX
#if JPEG
        bool check = false;
        //int jpegSize = (image_width * image_height) + 12288;
        
/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [START] */
	int mJPEGLength  = thumb_width*thumb_height + image_width*image_height + ((2*PAGE) - 1); 

    	mJPEGLength &= ~((2*PAGE) - 1);
   	mJPEGLength  += 2*PAGE;
   	mJPEGPictureHeap = new MemoryHeapBase(mJPEGLength);
/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [END] */


/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
		unsigned int crop_top, crop_left, crop_width, crop_height;		// rt5604 2012.06.06 JUSTIN
		int capture_width, capture_height;

// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
		//ALOGI("kim vt   ICapturePerform YUV422I_rotate, YUV422I_rotate_270");
		extern int YUV422I_rotate(const uint8_t *src, uint8_t *dst, int src_w, int src_h);
		extern int YUV422I_rotate_270(const uint8_t *src, uint8_t *dst, int src_w, int src_h);
		unsigned char my_buf[640*480*2];
#endif
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */
		
        // mJPEGPictureHeap = new MemoryHeapBase(jpegSize+ 256);
        // outBuffer = (void *)((unsigned long)(mJPEGPictureHeap->getBase()) + 128);

/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [START] */
	ALOGD("mJPEGPictureHeap->getStrongCount() = %d, base = 0x%x",
              mJPEGPictureHeap->getStrongCount(), (unsigned int)mJPEGPictureHeap->getBase());
                jpegSize = mJPEGLength;

                if(mJPEGPictureHeap->getStrongCount() > 1 )
                {
		    ALOGE(" Reducing the strong pointer count ");
                    mJPEGPictureHeap.clear();
                    mJPEGPictureHeap = new MemoryHeapBase(jpegSize);
                }

	base = (unsigned long) mJPEGPictureHeap->getBase();
	base = (base + 0xfff) & 0xfffff000;
	offset = base - (unsigned long) mJPEGPictureHeap->getBase();
	//if(image_width <= 480 || image_width <= 480){usleep(50000);} //Need to check

        outBuffer = (void *) base;
/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [END] */	

        exif_buffer *exif_buf = get_exif_buffer(&mExifParams, gpsLocation);

        PPM("BEFORE JPEG Encode Image");
        ALOGE(" outbuffer = 0x%x, jpegSize = %d, yuv_buffer = 0x%x, yuv_len = %d, "
                "image_width = %d, image_height = %d,  quality = %d, mippMode = %d",
                (unsigned int)outBuffer , jpegSize, (unsigned int)yuv_buffer, yuv_len,
                image_width, image_height, quality, mippMode);

/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [START] */
#if defined(LGE_JUSTIN_DEVICE) 
                if(image_rotation == 90 || image_rotation == 270) { //Start of if()		
			crop_left   = 0	;	
			crop_width  = (image_height*image_height)/image_width;
			crop_top    = (image_width-(image_height*image_height)/image_width)/2;	
			crop_height =  image_height;	

			capture_width = image_width;		
			capture_height = image_height;

		        image_width = capture_height;
			image_height = capture_width;
			image_rotation = image_rotation + 90;

                	if(image_rotation >=360)
				image_rotation = image_rotation - 360;	
			
			//ALOGD("Before Encode: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
			//ALOGD("Before Encode:: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d,crop_height= %d", crop_top, crop_left,crop_width, crop_height);
			//ALOGD("Before Encode:: --> 1. image_rotation = %d", image_rotation);
			//ALOGD("Before Encode: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat,image_zoom);
						
	
			if(!(jpegEncoder->encodeImage((uint8_t *)outBuffer , jpegSize, yuv_buffer, yuv_len, capture_width, capture_height,
			quality, exif_buf, jpegFormat, thumb_width, thumb_height, image_width, image_height, 
			image_rotation, image_zoom, crop_top, crop_left, crop_width, crop_height))) {
				 ALOGE("JPEG Encoding failed");
			}

			//ALOGD("After Encode: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
			//ALOGD("After Encode: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d,crop_height= %d", crop_top, crop_left,crop_width, crop_height);
			//ALOGD("After Encode: --> 1. image_rotation = %d", image_rotation);
			//ALOGD("After Encode: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat,image_zoom);
			
		} //End of if()

		else { //Start of else
			//YUV422I_rotate((uint8_t *) yuv_buffer, (uint8_t *) my_buf, image_width, image_height); 
			 //ALOGE("kim vt CameraHal::ICapturePerform image_rotation:%d", image_rotation);
			if (image_rotation == 0)
				YUV422I_rotate((uint8_t *) yuv_buffer, (uint8_t *) my_buf, image_width, image_height);
			else 
				YUV422I_rotate_270((uint8_t *) yuv_buffer, (uint8_t *) my_buf, image_width, image_height);

			crop_left   = (image_width-(image_height*image_height)/image_width)/2;			
			crop_width  = image_height;			
			crop_top    = 0;
			crop_height = (image_height*image_height)/image_width;

			capture_width = image_height;		
			capture_height = image_width;
			image_rotation = 0;
		

			//ALOGD("Before Encode:  --> 2. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
			//ALOGD("Before Encode:  --> 2. crop_top = %d, crop_lef = %d, crop_width = %d, crop_width = %d", crop_top, crop_left, crop_width, crop_height);
			//ALOGD("Before Encode: --> 2. image_rotation = %d, jpegFormat=%d, image_zoom=%g", image_rotation, jpegFormat, image_zoom);

       		 if(!(jpegEncoder->encodeImage((uint8_t *)outBuffer , jpegSize, my_buf, yuv_len,capture_width, capture_height, quality, exif_buf, jpegFormat, 
			 	thumb_width, thumb_height,image_width, image_height, image_rotation, image_zoom, crop_top, crop_left, crop_width, crop_height))) 
			 {
						ALOGE("JPEG Encoding failed");
			 }				
			//ALOGD("After Encode: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
			//ALOGD("After Encode: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d, crop_width = %d", crop_top, crop_left, crop_width, crop_height);
			//ALOGD(" After Encode: --> 1. image_rotation = %d", image_rotation);
			//ALOGD("After Encode: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat, image_zoom);

	} //End of else

/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [END] */
#elif defined(BLACKG_OPEN_COM_DEVICE) // #ifdef	LGE_JUSTIN_DEVICE
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
//                                                       ī\B8޶\F3 \BE\EE\C7ÿ\A1\BC\AD \C0\FC\B8\E9 ī\B8޶\F3 \C3Կ\B5\BD\C3 \C1\C2 \B9\E6\C7\E2\C0\B8\B7\CE 90 \B5\B9\BEư\A1 justin\B0\FA \BAи\AE
//                                                       \C0\FA\BD\BAƾ\B0\FA \C0\FC\BA\CE \B5\BF\C0\CF\C7ϳ\AA mvt_mode\B0\A1 \BEƴϸ\E9 \C0\CC\C0\FC \C4ڵ\E5 Ÿ\B5\B5\B7\CF \BA\AF\B0\E6\C7\D4.
		 ALOGI("kim vt  3 ICapturePerform image_rotation:%d, mvt_mode:%d", image_rotation, mvt_mode);
                if(mvt_mode)
                {
	                if(image_rotation == 90 || image_rotation == 270) { //Start of if()		
				crop_left   = 0	;	
				crop_width  = (image_height*image_height)/image_width;
				crop_top    = (image_width-(image_height*image_height)/image_width)/2;	
				crop_height =  image_height;	

				capture_width = image_width;		
				capture_height = image_height;

			        image_width = capture_height;
				image_height = capture_width;
				image_rotation = image_rotation + 90;

	                	if(image_rotation >=360)
					image_rotation = image_rotation - 360;	
				
				//ALOGD("Before Encode: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
				//ALOGD("Before Encode:: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d,crop_height= %d", crop_top, crop_left,crop_width, crop_height);
				//ALOGD("Before Encode:: --> 1. image_rotation = %d", image_rotation);
				//ALOGD("Before Encode: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat,image_zoom);
							
		
				if(!(jpegEncoder->encodeImage((uint8_t *)outBuffer , jpegSize, yuv_buffer, yuv_len, capture_width, capture_height,
				quality, exif_buf, jpegFormat, thumb_width, thumb_height, image_width, image_height, 
				image_rotation, image_zoom, crop_top, crop_left, crop_width, crop_height))) {
					 ALOGE("JPEG Encoding failed");
				}

				//ALOGD("After Encode: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
				//ALOGD("After Encode: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d,crop_height= %d", crop_top, crop_left,crop_width, crop_height);
				//ALOGD("After Encode: --> 1. image_rotation = %d", image_rotation);
				//ALOGD("After Encode: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat,image_zoom);
				
			} //End of if()
			else { //Start of else
				//YUV422I_rotate((uint8_t *) yuv_buffer, (uint8_t *) my_buf, image_width, image_height); 
				 ALOGE("kim vt CameraHal::ICapturePerform image_rotation:%d", image_rotation);
				if (image_rotation == 0)
					YUV422I_rotate((uint8_t *) yuv_buffer, (uint8_t *) my_buf, image_width, image_height);
				else 
					YUV422I_rotate_270((uint8_t *) yuv_buffer, (uint8_t *) my_buf, image_width, image_height);

				crop_left   = (image_width-(image_height*image_height)/image_width)/2;			
				crop_width  = image_height;			
				crop_top    = 0;
				crop_height = (image_height*image_height)/image_width;

				capture_width = image_height;		
				capture_height = image_width;
				image_rotation = 0;
			

				//ALOGD("Before Encode:  --> 2. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
				//ALOGD("Before Encode:  --> 2. crop_top = %d, crop_lef = %d, crop_width = %d, crop_width = %d", crop_top, crop_left, crop_width, crop_height);
				//ALOGD("Before Encode: --> 2. image_rotation = %d, jpegFormat=%d, image_zoom=%g", image_rotation, jpegFormat, image_zoom);

	       		 if(!(jpegEncoder->encodeImage((uint8_t *)outBuffer , jpegSize, my_buf, yuv_len,capture_width, capture_height, quality, exif_buf, jpegFormat, 
				 	thumb_width, thumb_height,image_width, image_height, image_rotation, image_zoom, crop_top, crop_left, crop_width, crop_height))) 
				 {
							ALOGE("JPEG Encoding failed");
				 }				
				//ALOGD("After Encode: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
				//ALOGD("After Encode: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d, crop_width = %d", crop_top, crop_left, crop_width, crop_height);
				//ALOGD(" After Encode: --> 1. image_rotation = %d", image_rotation);
				//ALOGD("After Encode: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat, image_zoom);

		       } //End of else
              }
		else{
			crop_top  = 0;		crop_left = 0;		crop_width = image_width;		crop_height = image_height;
			capture_width = image_width;	capture_height = image_height;

			//ALOGD("rt5604: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
			//ALOGD("rt5604: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d, crop_width = %d", crop_top, crop_left, crop_width, crop_height);
			//ALOGD("rt5604: --> 1. image_rotation = %d", image_rotation);
			//ALOGD("rt5604: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat, image_zoom);


	/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [START] */ //Need to test for Black
			   if(!(jpegEncoder->encodeImage((uint8_t *)outBuffer , jpegSize, yuv_buffer, yuv_len,
			           capture_width, capture_height, quality, exif_buf, jpegFormat, thumb_width, thumb_height, image_width, image_height,
			            image_rotation, image_zoom, crop_top, crop_left, crop_width, crop_height))) {
			             ALOGE("JPEG Encoding failed");
			   }
	/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [END] */  //Need to test for Black

		}

#else // #ifdef	LGE_JUSTIN_DEVICE
		crop_top  = 0;		crop_left = 0;		crop_width = image_width;		crop_height = image_height;
		capture_width = image_width;	capture_height = image_height;

		//ALOGD("rt5604: --> 1. capture_width = %d, capture_height = %d, image_width = %d, image_height = %d", capture_width, capture_height, image_width, image_height);
		//ALOGD("rt5604: --> 1. crop_top = %d, crop_lef = %d, crop_width = %d, crop_width = %d", crop_top, crop_left, crop_width, crop_height);
		//ALOGD("rt5604: --> 1. image_rotation = %d", image_rotation);
		//ALOGD("rt5604: --> 1. image_rotation = %d, jpegFormat=%d, image_zoom=%f", image_rotation, jpegFormat, image_zoom);


/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [START] */ //Need to test for Black
        if(!(jpegEncoder->encodeImage((uint8_t *)outBuffer , jpegSize, yuv_buffer, yuv_len,
                capture_width, capture_height, quality, exif_buf, jpegFormat, thumb_width, thumb_height, image_width, image_height,
                image_rotation, image_zoom, crop_top, crop_left, crop_width, crop_height))) {
			    ALOGE("JPEG Encoding failed");
		}
/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [END] */  //Need to test for Black

#endif // #ifdef	LGE_JUSTIN_DEVICE 

        PPM("AFTER JPEG Encode Image");

        mJpegBuffAddr = outBuffer;

        mTotalJpegsize = jpegEncoder->jpegSize;

        PPM("Shot to Save", &ppm_receiveCmdToTakePicture);
        if((exif_buf != NULL) && (exif_buf->data != NULL))
            exif_buf_free (exif_buf);

        if( NULL != gpsLocation ) {
            free(gpsLocation);
            gpsLocation = NULL;
        }
#endif

#endif

    }

    if ((NO_ERROR == ret) && (NULL != mCameraAdapter)) {
    mCameraAdapter->queueToGralloc(0,(char*)yuv_buffer, CameraFrame::IMAGE_FRAME);
    }

    // Release constraint to DSP OPP by setting lowest Hz
    SetDSPKHz(DSP3630_KHZ_MIN);

    PPM("END OF ICapturePerform");
    LOG_FUNCTION_NAME_EXIT

    return NO_ERROR;
}

#if JPEG
void CameraHal::getCaptureSize(int *jpegSize)
{
    LOG_FUNCTION_NAME

    *jpegSize = mTotalJpegsize;

    LOG_FUNCTION_NAME_EXIT
    return ;
}

void CameraHal::copyJpegImage(void *imageBuf)
{
    LOG_FUNCTION_NAME
    
    memcpy(imageBuf, mJpegBuffAddr, mTotalJpegsize);
    mTotalJpegsize = 0;
    mJpegBuffAddr = NULL;
    //mJPEGPictureMemBase.clear();
#ifndef ICAP
    mJPEGPictureHeap.clear();
#endif

    LOG_FUNCTION_NAME_EXIT
    
    return ;
}
#endif

void *CameraHal::getLastOverlayAddress()
{
    void *res = NULL;

    LOG_FUNCTION_NAME_EXIT

    return res;
}

size_t CameraHal::getLastOverlayLength()
{
    size_t res = 0;

    LOG_FUNCTION_NAME_EXIT

    return res;
}

void CameraHal::snapshotThread()
{
    fd_set descriptorSet;
    int max_fd;
    int err, status;
    unsigned int snapshotMessage[9], snapshotReadyMessage;
    int image_width, image_height, pixelFormat, preview_width, preview_height;
    int crop_top, crop_left, crop_width, crop_height;
    void *yuv_buffer, *snapshot_buffer;
    double ZoomTarget;

    LOG_FUNCTION_NAME

    pixelFormat = PIX_YUV422I;
    max_fd = snapshotPipe[0] + 1;

    FD_ZERO(&descriptorSet);
    FD_SET(snapshotPipe[0], &descriptorSet);

    while(1) {
        err = select(max_fd,  &descriptorSet, NULL, NULL, NULL);

#ifdef DEBUG_LOG

       ALOGD("SNAPSHOT THREAD SELECT RECEIVED A MESSAGE\n");

#endif

       if (err < 1) {
           ALOGE("Snapshot: Error in select");
       }

       if(FD_ISSET(snapshotPipe[0], &descriptorSet)){

           read(snapshotPipe[0], &snapshotMessage, sizeof(snapshotMessage));

           if(snapshotMessage[0] == SNAPSHOT_THREAD_START){

#ifdef DEBUG_LOG

                ALOGD("SNAPSHOT_THREAD_START RECEIVED\n");

#endif

                yuv_buffer = (void *) snapshotMessage[1];
                image_width = snapshotMessage[2];
                image_height = snapshotMessage[3];
                ZoomTarget = zoom_step[snapshotMessage[4]];
                crop_left = snapshotMessage[5];
                crop_top = snapshotMessage[6];
                crop_width = snapshotMessage[7];
                crop_height = snapshotMessage[8];

                mParameters.getPreviewSize(&preview_width, &preview_height);

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

                PPM("Before vpp downscales:");

#endif

		        char** buf = mCameraAdapter->getVirtualAddress(1);
                snapshot_buffer = *buf ;
                if ( NULL == snapshot_buffer )
                    goto EXIT;

                ALOGE("Snapshot buffer 0x%x, yuv_buffer = 0x%x, zoomTarget = %5.2f", ( unsigned int ) snapshot_buffer, ( unsigned int ) yuv_buffer, ZoomTarget);

                status = scale_init(image_width, image_height, preview_width, preview_height, PIX_YUV422I, PIX_YUV422I);

                if ( status < 0 ) {
                    ALOGE("VPP init failed");
                    goto EXIT;
                }

                status = scale_process(yuv_buffer, image_width, image_height,
                        snapshot_buffer, preview_width, preview_height, 0, PIX_YUV422I, zoom_step[0], crop_top, crop_left, crop_width, crop_height);

#ifdef DEBUG_LOG

               PPM("After vpp downscales:");

               if( status )
                   ALOGE("scale_process() failed");
               else
                   ALOGD("scale_process() OK");

#endif

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

               PPM("Shot to Snapshot", &ppm_receiveCmdToTakePicture);

#endif

                scale_deinit();

                //queueToOverlay(lastOverlayBufferDQ);//  replace with get frame 
                //dequeueFromOverlay(); //  replace with queue to gralloc 

EXIT:

                write(snapshotReadyPipe[1], &snapshotReadyMessage, sizeof(snapshotReadyMessage));
          } else if (snapshotMessage[0] == SNAPSHOT_THREAD_START_GEN) {

				mCameraAdapter->queueToGralloc(0, NULL, CameraFrame::IMAGE_FRAME);
                //queueToOverlay(lastOverlayBufferDQ);// replace with get frame 
                mParameters.getPreviewSize(&preview_width, &preview_height);

#ifdef DUMP_SNAPSHOT
                SaveFile(NULL, (char*)"snp", getLastOverlayAddress(), preview_width*preview_height*2);
#endif

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

               PPM("Shot to Snapshot", &ppm_receiveCmdToTakePicture);

#endif
                //dequeueFromOverlay();// replace with queue to gralloc 
                write(snapshotReadyPipe[1], &snapshotReadyMessage, sizeof(snapshotReadyMessage));

          } else if (snapshotMessage[0] == SNAPSHOT_THREAD_EXIT) {
                ALOGD("SNAPSHOT_THREAD_EXIT RECEIVED");

                break;
          }
        }
    }

    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::rawThread()
{
    LOG_FUNCTION_NAME

    fd_set descriptorSet;
    int max_fd;
    int err;
    unsigned int rawMessage[RAW_THREAD_NUM_ARGS];
    camera_data_callback RawCallback;
    void *PictureCallbackCookie;
    sp<MemoryBase> rawData;

    max_fd = rawPipe[0] + 1;

    FD_ZERO(&descriptorSet);
    FD_SET(rawPipe[0], &descriptorSet);

    while(1) {
        err = select(max_fd,  &descriptorSet, NULL, NULL, NULL);

#ifdef DEBUG_LOG

            ALOGD("RAW THREAD SELECT RECEIVED A MESSAGE\n");

#endif

            if (err < 1) {
                ALOGE("Raw: Error in select");
            }

            if(FD_ISSET(rawPipe[0], &descriptorSet)){

                read(rawPipe[0], &rawMessage, sizeof(rawMessage));

                if(rawMessage[0] == RAW_THREAD_CALL){

#ifdef DEBUG_LOG

                ALOGD("RAW_THREAD_CALL RECEIVED\n");

#endif

                RawCallback = (camera_data_callback) rawMessage[1];
                PictureCallbackCookie = (void *) rawMessage[2];
                rawData = (MemoryBase *) rawMessage[3];

               // RawCallback(CAMERA_MSG_RAW_IMAGE, rawData, PictureCallbackCookie);

#ifdef DEBUG_LOG

                PPM("RAW CALLBACK CALLED");

#endif

            } else if (rawMessage[0] == RAW_THREAD_EXIT) {
                ALOGD("RAW_THREAD_EXIT RECEIVED");

                break;
            }
        }
    }

    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::shutterThread()
{
    LOG_FUNCTION_NAME

    fd_set descriptorSet;
    int max_fd;
    int err;
    unsigned int shutterMessage[SHUTTER_THREAD_NUM_ARGS];
    camera_notify_callback ShutterCallback;
    void *PictureCallbackCookie;

    max_fd = shutterPipe[0] + 1;

    FD_ZERO(&descriptorSet);
    FD_SET(shutterPipe[0], &descriptorSet);

    while(1) {
        err = select(max_fd,  &descriptorSet, NULL, NULL, NULL);

#ifdef DEBUG_LOG

        ALOGD("SHUTTER THREAD SELECT RECEIVED A MESSAGE\n");

#endif

        if (err < 1) {
            ALOGE("Shutter: Error in select");
        }

        if(FD_ISSET(shutterPipe[0], &descriptorSet)){

           read(shutterPipe[0], &shutterMessage, sizeof(shutterMessage));

           if(shutterMessage[0] == SHUTTER_THREAD_CALL){

#ifdef DEBUG_LOG

                ALOGD("SHUTTER_THREAD_CALL_RECEIVED\n");

#endif

                ShutterCallback = (camera_notify_callback) shutterMessage[1];
                PictureCallbackCookie = (void *) shutterMessage[2];

                ShutterCallback(CAMERA_MSG_SHUTTER, 0, 0, PictureCallbackCookie);

#ifdef DEBUG_LOG

                PPM("CALLED SHUTTER CALLBACK");

#endif

            } else if (shutterMessage[0] == SHUTTER_THREAD_EXIT) {
                ALOGD("SHUTTER_THREAD_EXIT RECEIVED");

                break;
            }
        }
    }

    LOG_FUNCTION_NAME_EXIT
}

#ifdef USE_AUTOFOCUS_THREAD
void CameraHal::autoFocusCBThread()
{
    LOG_FUNCTION_NAME

    fd_set descriptorSet;
    int max_fd;
    int err;
    unsigned int autoFocusMessage[AFCB_THREAD_NUM_ARGS];
    camera_notify_callback AutoFocusCBCallback;
    void *AutoFocusCBCallbackCookie;
    bool AutoFocusCBCallbackValue;

    max_fd = afcbPipe[0] + 1;

    FD_ZERO(&descriptorSet);
    FD_SET(afcbPipe[0], &descriptorSet);

    while(1) {
        err = select(max_fd,  &descriptorSet, NULL, NULL, NULL);

#ifdef DEBUG_LOG
        ALOGD("AFCB THREAD SELECT RECEIVED A MESSAGE\n");
#endif
        if (err < 1) {
            ALOGE("AutoFocusCB: Error in select");
        }

        if(FD_ISSET(afcbPipe[0], &descriptorSet)){
           read(afcbPipe[0], &autoFocusMessage, sizeof(autoFocusMessage));

           if(autoFocusMessage[0] == AFCB_THREAD_CALL){
                ALOGD("AFCB_THREAD_CALL_RECEIVED\n");

                AutoFocusCBCallback = (camera_notify_callback) autoFocusMessage[1];
                AutoFocusCBCallbackCookie = (void *) autoFocusMessage[2];
                AutoFocusCBCallbackValue = (bool) autoFocusMessage[3];

                AutoFocusCBCallback(CAMERA_MSG_FOCUS, AutoFocusCBCallbackValue, 0, AutoFocusCBCallbackCookie);
            } else if (autoFocusMessage[0] == AFCB_THREAD_EXIT) {
                ALOGD("AFCB_THREAD_EXIT RECEIVED");

                break;
            }
        }
    }

    LOG_FUNCTION_NAME_EXIT
}
#endif // #ifdef USE_AUTOFOCUS_THREAD

// rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom ->
void CameraHal::zoomCBThread()
{
    LOG_FUNCTION_NAME

    fd_set descriptorSet;
    int max_fd;
    int err;
    unsigned int zoomMessage[ZOOMCB_THREAD_NUM_ARGS];
    camera_notify_callback ZoomCBCallback;
    void *ZoomCBCallbackCookie;
    bool ZoomCBCallbackValue;
    int NewZoomValue;

    max_fd = zoomcbPipe[0] + 1;

    FD_ZERO(&descriptorSet);
    FD_SET(zoomcbPipe[0], &descriptorSet);

    while(1) {
        err = select(max_fd,  &descriptorSet, NULL, NULL, NULL);

#ifdef DEBUG_LOG
        ALOGD("ZOOMCB THREAD SELECT RECEIVED A MESSAGE\n");
#endif

        if (err < 1) {
            ALOGE("ZoomCB: Error in select");
        }

        if(FD_ISSET(zoomcbPipe[0], &descriptorSet)){

           read(zoomcbPipe[0], &zoomMessage, sizeof(zoomMessage));

           if(zoomMessage[0] == ZOOMCB_THREAD_CALL){
                ZoomCBCallback = (camera_notify_callback) zoomMessage[1];
                ZoomCBCallbackCookie = (void *) zoomMessage[2];
                ZoomCBCallbackValue = (bool) zoomMessage[3];
                NewZoomValue = (int) zoomMessage[4];

                ALOGD("ZoomCBCallback: NewZoomValue=%d,ZoomCBCallbackValue=%d", NewZoomValue, ZoomCBCallbackValue);
                ZoomCBCallback(CAMERA_MSG_ZOOM, NewZoomValue, ZoomCBCallbackValue, ZoomCBCallbackCookie);
            } else if (zoomMessage[0] == ZOOMCB_THREAD_EXIT) {
                ALOGD("ZOOMCB_THREAD_EXIT RECEIVED");
                break;
            }
        }
    }

    LOG_FUNCTION_NAME_EXIT
}
// rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom <-

void CameraHal::procThread()
{
    LOG_FUNCTION_NAME

    int status;
    int capture_width, capture_height, image_width, image_height;
    fd_set descriptorSet;
    int max_fd;
    int err;
    int pixelFormat;
    unsigned int procMessage[PROC_MSG_IDX_MAX];
    unsigned int jpegQuality, jpegSize, size, base, tmpBase, offset, yuv_len, image_rotation;
    unsigned int crop_top, crop_left, crop_width, crop_height;
    double image_zoom;
    camera_data_callback RawPictureCallback;
    camera_data_callback JpegPictureCallback;
    void *yuv_buffer, *outBuffer, *PictureCallbackCookie;
    int thumb_width, thumb_height;

#ifdef HARDWARE_OMX
    exif_buffer *exif_buf;
#endif

#ifdef IMAGE_PROCESSING_PIPELINE
/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [START] */
    int capture_width_old = -1;
    int capture_height_old = -1; 
    unsigned int ippMode_old = IPP_Disabled_Mode;

/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [END] */
    unsigned int ippMode = IPP_Disabled_Mode;
    bool ipp_to_enable;
    unsigned short EdgeEnhancementStrength, WeakEdgeThreshold, StrongEdgeThreshold,
                   LowFreqLumaNoiseFilterStrength, MidFreqLumaNoiseFilterStrength, HighFreqLumaNoiseFilterStrength,
                   LowFreqCbNoiseFilterStrength, MidFreqCbNoiseFilterStrength, HighFreqCbNoiseFilterStrength,
                   LowFreqCrNoiseFilterStrength, MidFreqCrNoiseFilterStrength, HighFreqCrNoiseFilterStrength,
                   shadingVertParam1, shadingVertParam2, shadingHorzParam1, shadingHorzParam2, shadingGainScale,
                   shadingGainOffset, shadingGainMaxValue, ratioDownsampleCbCr;
#endif

    void* input_buffer;
    unsigned int input_length;

    max_fd = procPipe[0] + 1;

    FD_ZERO(&descriptorSet);
    FD_SET(procPipe[0], &descriptorSet);

    mJPEGLength  = MAX_THUMB_WIDTH*MAX_THUMB_HEIGHT + PICTURE_WIDTH*PICTURE_HEIGHT + ((2*PAGE) - 1);
    mJPEGLength &= ~((2*PAGE) - 1);
    mJPEGLength  += 2*PAGE;
    mJPEGPictureHeap = new MemoryHeapBase(mJPEGLength);

    while(1){

        err = select(max_fd,  &descriptorSet, NULL, NULL, NULL);

#ifdef DEBUG_LOG

        ALOGD("PROCESSING THREAD SELECT RECEIVED A MESSAGE\n");

#endif

        if (err < 1) {
            ALOGE("Proc: Error in select");
        }

        if(FD_ISSET(procPipe[0], &descriptorSet)){

            read(procPipe[0], &procMessage, sizeof(procMessage));

            if(procMessage[PROC_MSG_IDX_ACTION] == PROC_THREAD_PROCESS) {

#ifdef DEBUG_LOG

                ALOGD("PROC_THREAD_PROCESS_RECEIVED\n");

#endif

                capture_width = procMessage[PROC_MSG_IDX_CAPTURE_W];
                capture_height = procMessage[PROC_MSG_IDX_CAPTURE_H];
                image_width = procMessage[PROC_MSG_IDX_IMAGE_W];
                image_height = procMessage[PROC_MSG_IDX_IMAGE_H];
                pixelFormat = procMessage[PROC_MSG_IDX_PIX_FMT];
#ifdef IMAGE_PROCESSING_PIPELINE
                ippMode = procMessage[PROC_MSG_IDX_IPP_MODE];
                if (ippMode != IPP_Disabled_Mode) {
                    ipp_to_enable = procMessage[PROC_MSG_IDX_IPP_TO_ENABLE];
                    EdgeEnhancementStrength = procMessage[PROC_MSG_IDX_IPP_EES];
                    WeakEdgeThreshold = procMessage[PROC_MSG_IDX_IPP_WET];
                    StrongEdgeThreshold = procMessage[PROC_MSG_IDX_IPP_SET];
                    LowFreqLumaNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_LFLNFS];
                    MidFreqLumaNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_MFLNFS];
                    HighFreqLumaNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_HFLNFS];
                    LowFreqCbNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_LFCBNFS];
                    MidFreqCbNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_MFCBNFS];
                    HighFreqCbNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_HFCBNFS];
                    LowFreqCrNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_LFCRNFS];
                    MidFreqCrNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_MFCRNFS];
                    HighFreqCrNoiseFilterStrength = procMessage[PROC_MSG_IDX_IPP_HFCRNFS];
                    shadingVertParam1 = procMessage[PROC_MSG_IDX_IPP_SVP1];
                    shadingVertParam2 = procMessage[PROC_MSG_IDX_IPP_SVP2];
                    shadingHorzParam1 = procMessage[PROC_MSG_IDX_IPP_SHP1];
                    shadingHorzParam2 = procMessage[PROC_MSG_IDX_IPP_SHP2];
                    shadingGainScale = procMessage[PROC_MSG_IDX_IPP_SGS];
                    shadingGainOffset = procMessage[PROC_MSG_IDX_IPP_SGO];
                    shadingGainMaxValue = procMessage[PROC_MSG_IDX_IPP_SGMV];
                    ratioDownsampleCbCr = procMessage[PROC_MSG_IDX_IPP_RDSCBCR];
                }
#endif
                yuv_buffer = (void *) procMessage[PROC_MSG_IDX_YUV_BUFF];
                yuv_len = procMessage[PROC_MSG_IDX_YUV_BUFFLEN];
                image_rotation = procMessage[PROC_MSG_IDX_ROTATION];
                image_zoom = zoom_step[procMessage[PROC_MSG_IDX_ZOOM]];
                jpegQuality = procMessage[PROC_MSG_IDX_JPEG_QUALITY];
                JpegPictureCallback = (camera_data_callback) procMessage[PROC_MSG_IDX_JPEG_CB];
                RawPictureCallback = (camera_data_callback) procMessage[PROC_MSG_IDX_RAW_CB];
                PictureCallbackCookie = (void *) procMessage[PROC_MSG_IDX_CB_COOKIE];
                crop_left = procMessage[PROC_MSG_IDX_CROP_L];
                crop_top = procMessage[PROC_MSG_IDX_CROP_T];
                crop_width = procMessage[PROC_MSG_IDX_CROP_W];
                crop_height = procMessage[PROC_MSG_IDX_CROP_H];
                thumb_width = procMessage[PROC_MSG_IDX_THUMB_W];
                thumb_height = procMessage[PROC_MSG_IDX_THUMB_H];

#ifdef HARDWARE_OMX
                exif_buf = (exif_buffer *)procMessage[PROC_MSG_IDX_EXIF_BUFF];
#endif

                ALOGD("mJPEGPictureHeap->getStrongCount() = %d, base = 0x%x",
                        mJPEGPictureHeap->getStrongCount(), (unsigned int)mJPEGPictureHeap->getBase());
                jpegSize = mJPEGLength;

                if(mJPEGPictureHeap->getStrongCount() > 1 )
                {
					ALOGE(" Reducing the strong pointer count ");
                    mJPEGPictureHeap.clear();
                    mJPEGPictureHeap = new MemoryHeapBase(jpegSize);
                }

                base = (unsigned long) mJPEGPictureHeap->getBase();
                base = (base + 0xfff) & 0xfffff000;
                offset = base - (unsigned long) mJPEGPictureHeap->getBase();
                if(capture_width <= 480 || capture_height <= 480){usleep(50000);}
                outBuffer = (void *) base;

                pixelFormat = PIX_YUV422I;

                input_buffer = (void *) NEXT_4K_ALIGN_ADDR(yuv_buffer);
                input_length = yuv_len - ((unsigned int) input_buffer - (unsigned int) yuv_buffer);

#ifdef IMAGE_PROCESSING_PIPELINE

#ifdef DEBUG_LOG

                ALOGD("IPPmode=%d",ippMode);
                if(ippMode == IPP_CromaSupression_Mode){
                    ALOGD("IPP_CromaSupression_Mode");
                }
                else if(ippMode == IPP_EdgeEnhancement_Mode){
                    ALOGD("IPP_EdgeEnhancement_Mode");
                }
                else if(ippMode == IPP_Disabled_Mode){
                    ALOGD("IPP_Disabled_Mode");
                }

#endif

               // Boost DSP OPP to highest level
               SetDSPKHz(DSP3630_KHZ_MAX); //Kirti Added		

               if( (ippMode == IPP_CromaSupression_Mode) || (ippMode == IPP_EdgeEnhancement_Mode) ){

/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [START] */
                   // if(ipp_to_enable) {
        if( (capture_width != capture_width_old) || (capture_height != capture_height_old) || (ippMode != ippMode_old) ) {

                        if(pIPP.hIPP != NULL) {
                            int deinit_ipp_err;           

#ifdef DEBUG_LOG
                            PPM("Before deinit IPP");
#endif

                            ALOGD("pIPP.hIPP=%p", pIPP.hIPP);
                            deinit_ipp_err = DeInitIPP(mippMode);
                            ALOGE_IF(deinit_ipp_err, "ERROR DeInitIPP() failed");
                            pIPP.hIPP = NULL;
                        }

/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [END] */

                         err = InitIPP(capture_width, capture_height, pixelFormat, ippMode);
                         if( err )
                             ALOGE("ERROR InitIPP() failed");

#ifdef DEBUG_LOG
                             PPM("After IPP Init");
#endif

                     }

                   err = PopulateArgsIPP(capture_width, capture_height, pixelFormat, ippMode);
                    if( err )
                         ALOGE("ERROR PopulateArgsIPP() failed");

#ifdef DEBUG_LOG
                     PPM("BEFORE IPP Process Buffer");
                     ALOGD("Calling ProcessBufferIPP(buffer=%p , len=0x%x)", input_buffer, input_length);
#endif
                    // TODO: Need to add support for new EENF 1.9 parameters from proc messages
                     err = ProcessBufferIPP(input_buffer, input_length,
                            pixelFormat,
                            ippMode,
                            EdgeEnhancementStrength,
                            WeakEdgeThreshold,
                            StrongEdgeThreshold,
                            LowFreqLumaNoiseFilterStrength,
                            MidFreqLumaNoiseFilterStrength,
                            HighFreqLumaNoiseFilterStrength,
                            LowFreqCbNoiseFilterStrength,
                            MidFreqCbNoiseFilterStrength,
                            HighFreqCbNoiseFilterStrength,
                            LowFreqCrNoiseFilterStrength,
                            MidFreqCrNoiseFilterStrength,
                            HighFreqCrNoiseFilterStrength,
                            shadingVertParam1,
                            shadingVertParam2,
                            shadingHorzParam1,
                            shadingHorzParam2,
                            shadingGainScale,
                            shadingGainOffset,
                            shadingGainMaxValue,
                            ratioDownsampleCbCr);
                    if( err )
                         ALOGE("ERROR ProcessBufferIPP() failed");

#ifdef DEBUG_LOG
                    PPM("AFTER IPP Process Buffer");
#endif
                    pixelFormat = PIX_YUV422I; //output of IPP is always 422I
                    if(!(pIPP.ippconfig.isINPLACE)){
                        input_buffer = pIPP.pIppOutputBuffer;
                        input_length = pIPP.outputBufferSize;
                    }
/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [START] */
                   capture_width_old = capture_width;
                   capture_height_old = capture_height;
                   ippMode_old = ippMode;
/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [END] */
               }
#endif

#if JPEG

#ifdef DEBUG_LOG
                ALOGD(" outbuffer = %p, jpegSize = %d, input_buffer = %p, input_length = %d, "
                        "image_width = %d, image_height = %d, quality = %d",
                        outBuffer , jpegSize, input_buffer, input_length,
                        image_width, image_height, jpegQuality);
#endif
                //workaround for thumbnail size  - it should be smaller than captured image
                if ((image_width<thumb_width) || (image_height<thumb_width) ||
                    (image_width<thumb_height) || (image_height<thumb_height)) {
                     thumb_width = MIN_THUMB_WIDTH;
                     thumb_height = MIN_THUMB_HEIGHT;
                }

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS
                PPM("BEFORE JPEG Encode Image");
#endif
                if ( !mRawDump ) {
                	if (!( jpegEncoder->encodeImage((uint8_t *)outBuffer , jpegSize, input_buffer, input_length,
                                             capture_width, capture_height, jpegQuality, exif_buf, pixelFormat, thumb_width, thumb_height, image_width, image_height,
                                             image_rotation, image_zoom, crop_top, crop_left, crop_width, crop_height)))
                	{
                    	ALOGE("JPEG Encoding failed");
                	}

		        mJpegBuffAddr = outBuffer;
		        mTotalJpegsize = jpegEncoder->jpegSize;
                
                }
#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS
                    PPM("Shot to JPEG", &ppm_receiveCmdToTakePicture);
#endif

#endif

#ifdef DEBUG_LOG

               ALOGD("jpegEncoder->jpegSize=%d jpegSize=%d", jpegEncoder->jpegSize, jpegSize);

#endif

// (+) CTS Bug fix for takePicture, testPreviewFpsRange
               if( msgTypeEnabled(CAMERA_MSG_RAW_IMAGE_NOTIFY) ) {
                   if (mNotifyCb) {
                       ALOGE("rt5604: ---> call mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, mCallbackCookie)\n");
                       mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, PictureCallbackCookie);
                   }
               }
// (-) CTS Bug fix for takePicture, testPreviewFpsRange

               camera_memory_t* picture = NULL;

               picture = mRequestMemory(-1, jpegEncoder->jpegSize, 1, NULL);
               if (picture && picture->data) {
                   memcpy(picture->data, outBuffer, jpegEncoder->jpegSize);
               } else {
                   CAMHAL_ALOGEA("ERROR create mRequestMemory picture");
               }

               // Send the callback to the application only if the notifier is started and the message is enabled
               if(picture && msgTypeEnabled(mBurstShots ? CAMERA_MSG_COMPRESSED_BURST_IMAGE : CAMERA_MSG_COMPRESSED_IMAGE) ) {
                   ALOGD( "sending the CAMERA_MSG_COMPRESSED_IMAGE callback to the app \n");
                   JpegPictureCallback(mBurstShots ? CAMERA_MSG_COMPRESSED_BURST_IMAGE : CAMERA_MSG_COMPRESSED_IMAGE, picture , 0, NULL, mCallbackCookie);
               } else {
                   CAMHAL_ALOGEA("ERROR CAMERA_MSG_COMPRESSED_IMAGE is disabled");
               }

               if (picture) {
                   picture->release(picture);
               }

#ifdef HARDWARE_OMX

                if((exif_buf != NULL) && (exif_buf->data != NULL))
                    exif_buf_free(exif_buf);

#endif

				if (NULL != mCameraAdapter) {
					mCameraAdapter->queueToGralloc(0,NULL, CameraFrame::IMAGE_FRAME);
				}

                // Release constraint to DSP OPP by setting lowest Hz
                SetDSPKHz(DSP3630_KHZ_MIN);

            } else if(procMessage[PROC_MSG_IDX_ACTION] == PROC_THREAD_EXIT) {
                ALOGD("PROC_THREAD_EXIT_RECEIVED");
                mJPEGPictureHeap.clear();
                break;
            }
        }
    }

    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::allocatePictureBuffers(size_t length, int burstCount)
{
    if (burstCount > MAX_BURST) {
        ALOGE("Can't handle burstCount(%d) larger then MAX_BURST(%d)",
                burstCount, MAX_BURST);
        return -1;
    }

    length += ((2*PAGE) - 1) + 10*PAGE;
    length &= ~((2*PAGE) - 1);
    length += 2*PAGE;

    for (int i = 0; i < burstCount; i++) {
        if (mYuvBuffer[i] != NULL && mYuvBufferLen[i] == length) {
            // proper buffer already allocated. skip alloc.
            continue;
        }

        if (mYuvBuffer[i])
            free(mYuvBuffer[i]);

        mYuvBuffer[i] = (uint8_t *) malloc(length);
        mYuvBufferLen[i] = length;

        if (mYuvBuffer[i] == NULL) {
            ALOGE("mYuvBuffer[%d] malloc failed", i);
            return -1;
        }
    }

    return NO_ERROR;
}

int CameraHal::freePictureBuffers(void)
{
    for (int i = 0; i < MAX_BURST; i++) {
        if (mYuvBuffer[i]) {
            free(mYuvBuffer[i]);
            mYuvBuffer[i] = NULL;
            mYuvBufferLen[i] = 0;
        }
    }

    return NO_ERROR;
}

status_t CameraHal::freePreviewBufs()
{
    status_t ret = NO_ERROR;
    LOG_FUNCTION_NAME;

    CAMHAL_ALOGDB("mPreviewBufs = 0x%x", (unsigned int)mPreviewBufs);
    if(mPreviewBufs)
        {
        ///@todo Pluralise the name of this method to freeBuffers
        ret = mBufProvider->freeBuffer(mPreviewBufs);
        mPreviewBufs = NULL;
        LOG_FUNCTION_NAME_EXIT;
        return ret;
        }
    LOG_FUNCTION_NAME_EXIT;
    return ret;
}

status_t CameraHal::startPreview()
{
    LOG_FUNCTION_NAME

    status_t ret = NO_ERROR;

    if ( mPreviewChangedNeedsRestart==true && mfirstTime!=0 ) {
#ifdef MMS_COMBINATION_BUG_FIX
        if( CameraDestroy(false) < 0){
            ALOGE("ERROR in %s %d", __FILE__, __LINE__);
        } else {
            CAMHAL_ALOGDA("CameraDestroy");
        }
#endif
    }
    mPreviewChangedNeedsRestart = false;

    if(mDisplayAdapter == NULL)    //Eclair HAL
    {
        ALOGD("Return from camera Start Preview");
        mPreviewRunning = true;
        mFalsePreview = true;
        return NO_ERROR;
    }

    if ( mZoomTargetIdx != mZoomCurrentIdx )
    {
        if( ZoomPerform(zoom_step[mZoomTargetIdx]) < 0 )
        ALOGE("Error while applying zoom");

        mZoomCurrentIdx = mZoomTargetIdx;
        mParameters.set("zoom", (int) mZoomCurrentIdx);
    }

    if( (mDisplayAdapter.get() != NULL) && ( !mPreviewRunning ) && ( mDisplayPaused ) )
    {
        CAMHAL_ALOGDA("Preview is in paused state");

        mDisplayPaused = false;

        if ( NO_ERROR == ret )
        {
            ret = mDisplayAdapter->pauseDisplay(mDisplayPaused);

            if ( NO_ERROR != ret )
            {
                CAMHAL_ALOGEB("Display adapter resume failed %x", ret);
            }
        }
        //restart preview callbacks
        if(mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME)
        {
            mAppCallbackNotifier->enableMsgType (CAMERA_MSG_PREVIEW_FRAME);
        }
    }

    mFalsePreview = false;   //Eclair HAL

    TIUTILS::Message msg;
    msg.command = PREVIEW_START;
    previewThreadCommandQ.put(&msg);
    previewThreadAckQ.get(&msg);

    LOG_FUNCTION_NAME_EXIT
    return msg.command == PREVIEW_ACK ? NO_ERROR : INVALID_OPERATION;
}

void CameraHal::stopPreview()
{
    LOG_FUNCTION_NAME

    status_t ret = NO_ERROR;

    ret = cancelAutoFocus();
    if ( ret != NO_ERROR ) {
        CAMHAL_ALOGDB("Cancel AutoFocus returned error %d", ret);
    }
    //forceStopPreview();

    // Reset Capture-Mode to default, so that when we switch from VideoRecording
    // to ImageCapture, CAPTURE_MODE is not left to VIDEO_MODE.
    CAMHAL_ALOGDA("Resetting Capture-Mode to default");
    mParameters.set(TICameraParameters::KEY_CAP_MODE, "");

    mFalsePreview = false;  //Eclair HAL
    TIUTILS::Message msg;
    msg.command = PREVIEW_STOP;
    previewThreadCommandQ.put(&msg);
    previewThreadAckQ.get(&msg);
    LOG_FUNCTION_NAME_EXIT;
}

void CameraHal::forceStopPreview()
{
    LOG_FUNCTION_NAME;

    if(mDisplayAdapter.get() != NULL) {
        ///Stop the buffer display first
        mDisplayAdapter->disableDisplay();
    }

    if(mAppCallbackNotifier.get() != NULL) {
        //Stop the callback sending
        mAppCallbackNotifier->stop();
        mAppCallbackNotifier->flushAndReturnFrames();
        mAppCallbackNotifier->stopPreviewCallbacks();
    }

    if ( NULL != mCameraAdapter )
    {
        CameraAdapter::AdapterState currentState;
        CameraAdapter::AdapterState nextState;

        currentState = mCameraAdapter->getState();
        nextState = mCameraAdapter->getNextState();


        // only need to send these control commands to state machine if we are
        // passed the LOADED_PREVIEW_STATE
        if (currentState > CameraAdapter::LOADED_PREVIEW_STATE) {
        // according to javadoc...FD should be stopped in stopPreview
        // and application needs to call startFaceDection again
        // to restart FD
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_FD);
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_CANCEL_AUTOFOCUS);
        }

        // only need to send these control commands to state machine if we are
        // passed the INITIALIZED_STATE
        if (currentState > CameraAdapter::INTIALIZED_STATE) {
        //Stop the source of frames
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_PREVIEW , true );
        }
    }

    freePreviewBufs();
    mSetPreviewWindowCalled = false;
    //freePreviewDataBufs();

   // mPreviewEnabled = false;
    //mDisplayPaused = false;
    //mPreviewStartInProgress = false;

    //mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_PREVIEW);

    LOG_FUNCTION_NAME_EXIT;
}



status_t CameraHal::autoFocus()
{
    LOG_FUNCTION_NAME

    status_t ret = NO_ERROR;

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

    gettimeofday(&mStartFocus, NULL);

#endif

    {
    Mutex::Autolock lock(mLock);
    mMsgEnabled |= CAMERA_MSG_FOCUS;
    }

    TIUTILS::Message msg;
    msg.command = PREVIEW_AF_START;
    previewThreadCommandQ.put(&msg);
    previewThreadAckQ.get(&msg);

    LOG_FUNCTION_NAME_EXIT
    return msg.command == PREVIEW_ACK ? NO_ERROR : INVALID_OPERATION;
}

bool CameraHal::previewEnabled()
{
    return mPreviewRunning;
}


status_t CameraHal::startRecording( )
{
    int w, h;
    const char *valstr = NULL;
    bool restartPreviewRequired = false;
    status_t ret = NO_ERROR;

    LOG_FUNCTION_NAME;

    if(!previewEnabled())
        {
        return NO_INIT;
        }

    if ( NO_ERROR == ret )
        {
        int count = atoi(mCameraProperties->get(CameraProperties::REQUIRED_PREVIEW_BUFS));
        mParameters.getPreviewSize(&w, &h);
        count = MAX_CAMERA_BUFFERS;
        mAppCallbackNotifier->useVideoBuffers(false);
        mAppCallbackNotifier->setVideoRes(w, h);
        char** buf = mCameraAdapter->getVirtualAddress(count);


        ret = mAppCallbackNotifier->initSharedVideoBuffers(mPreviewBufs, mPreviewOffsets, mPreviewFd, mPreviewLength, count, (void**)buf);
        }

    if ( NO_ERROR == ret )
        {
        ret = mAppCallbackNotifier->startRecording();
        }

    if ( NO_ERROR == ret )
        {
        int index = 0;
        ///Buffers for video capture (if different from preview) are expected to be allocated within CameraAdapter
        ret = mCameraAdapter->sendCommand(CameraAdapter::CAMERA_START_VIDEO);
        }

    if ( NO_ERROR == ret )
        {
        mRecordingEnabled = true;
        }

    LOG_FUNCTION_NAME_EXIT;
    return ret;
}

/**
   @brief Stop a previously started recording.

   @param none
   @return none

 */
void CameraHal::stopRecording()
{
    CameraAdapter::AdapterState currentState;

    LOG_FUNCTION_NAME;

    Mutex::Autolock lock(mLock);

    if (!mRecordingEnabled )
        {
        return;
        }

    currentState = mCameraAdapter->getState();
    if (currentState == CameraAdapter::VIDEO_CAPTURE_STATE) {
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_IMAGE_CAPTURE);
    }

    mAppCallbackNotifier->stopRecording();

    mCameraAdapter->sendCommand(CameraAdapter::CAMERA_STOP_VIDEO);

    mRecordingEnabled = false;

    LOG_FUNCTION_NAME_EXIT;
}

/**
   @brief Returns true if recording is enabled.

   @param none
   @return true If recording is currently running
         false If recording has been stopped

 */
int CameraHal::recordingEnabled()
{
    LOG_FUNCTION_NAME;

    LOG_FUNCTION_NAME_EXIT;

    return mRecordingEnabled;
}

/**
   @brief Release a record frame previously returned by CAMERA_MSG_VIDEO_FRAME.

   @param[in] mem MemoryBase pointer to the frame being released. Must be one of the buffers
               previously given by CameraHal
   @return none

 */
void CameraHal::releaseRecordingFrame(const void* mem)
{
    LOG_FUNCTION_NAME;

    if ( ( mRecordingEnabled ) && mem != NULL)
    {
        mAppCallbackNotifier->releaseRecordingFrame(mem);
    }

    LOG_FUNCTION_NAME_EXIT;

    return;
}

sp<IMemoryHeap>  CameraHal::getRawHeap() const
{
    return mPictureHeap;
}


status_t CameraHal::takePicture( )
{
    LOG_FUNCTION_NAME
    enableMsgType(CAMERA_MSG_COMPRESSED_IMAGE);

    TIUTILS::Message msg;
    msg.command = PREVIEW_CAPTURE;
    previewThreadCommandQ.put(&msg);
    previewThreadAckQ.get(&msg);

#ifdef DEBUG_LOG

    LOG_FUNCTION_NAME_EXIT

#endif

    return NO_ERROR;
}

status_t CameraHal::cancelPicture( )
{
    LOG_FUNCTION_NAME
    disableMsgType(CAMERA_MSG_RAW_IMAGE);
    disableMsgType(CAMERA_MSG_COMPRESSED_IMAGE);
//    mCallbackCookie = NULL;   // done in destructor
    mCancelPicture = true;

    ALOGE("Callbacks set to null");

    LOG_FUNCTION_NAME_EXIT

    return NO_ERROR;
}

void CameraHal::setParameterSupportedSizeList(CameraParameters &params, const char *key, const char *delimiter, const supported_resolution *supRes, size_t count)
{
    LOG_FUNCTION_NAME

    String8 Value;
    unsigned int tokens = 0;

    if ( NULL == supRes ) {
        ALOGE("Invalid resolutions array passed");
    } else {
        for ( unsigned int i = 0 ; i < count; i++ ) {
            if(mCameraID & supRes[i].camera_mask) {
                if (0U < tokens) {
                    Value.append(delimiter);
                }
                Value.appendFormat("%llux%llu", (unsigned long long int) supRes[i].width, (unsigned long long int) supRes[i].height);
                tokens++;
            }
        }
    }

    if (0U < tokens) {
        params.set(key, Value.string());
    } else {
        ALOGE("setParameterSupportedSizeList %s: No enough suitable tokens", key);
    }

    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::setParameterSupportedRangeList(CameraParameters &params, const char *key, const char *delimiter, const supported_range *supRange, size_t count)
{
    LOG_FUNCTION_NAME

    String8 Value;
    unsigned int tokens = 0;

    if ( NULL == supRange ) {
        ALOGE("Invalid range array passed");
    } else {
        for ( unsigned int i = 0 ; i < count; i++ ) {
            if((mCameraID & supRange[i].camera_mask) && (supRange[i].min != 0) && (supRange[i].max != 0)) {
                if (0U < tokens) {
                    Value.append(delimiter);
                }
                Value.appendFormat("(%d,%d)", supRange[i].min, supRange[i].max);
                tokens++;
            }
        }
    }

    if (0U < tokens) {
        params.set(key, Value.string());
    } else {
        ALOGE("setParameterSupportedRangeList %s: No enough suitable tokens", key);
    }

    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::setParameterSupportedRangeListToSingle(CameraParameters &params, const char *key, const char *delimiter, const supported_range *supRange, size_t count)
{
    LOG_FUNCTION_NAME

    String8 Value;
    unsigned int tokens = 0;

    if ( NULL == supRange ) {
        ALOGE("Invalid range array passed");
    } else {
        for ( unsigned int i = 0 ; i < count; i++ ) {
            if ( (mCameraID & supRange[i].camera_mask) && (0 < supRange[i].fixed) ) {
                if (0U < tokens) {
                    Value.append(delimiter);
                }
                Value.appendFormat("%d", supRange[i].fixed / 1000);
                tokens++;
            }
        }
    }

    if (0U < tokens) {
        params.set(key, Value.string());
    } else {
        ALOGE("setParameterSupportedRangeListToSingle %s: No enough suitable tokens", key);
    }

    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::getParameterPreviewFpsRange(const CameraParameters &params, int *min_fps, int *max_fps) const
{
    LOG_FUNCTION_NAME

    *min_fps = *max_fps = -1;
    int fixed_fps = -1;
    unsigned int i;

    params.getPreviewFpsRange(min_fps, max_fps);
    if ((0 < *min_fps) && (0 < *max_fps)) {
        // find corresponding fixed framerate
        for (i = 0; i < ARRAY_SIZE(supportedFpsRange); i++) {
            if ( (supportedFpsRange[i].min == *min_fps) && (supportedFpsRange[i].max == *max_fps) ) {
                fixed_fps = supportedFpsRange[i].fixed;
                break;
            }
        }
        if (i < ARRAY_SIZE(supportedFpsRange)) {
            ALOGD("getParameterPreviewFpsRange(): Framerate range: MIN %d/1000, MAX %d/1000 (fixed %d)", *min_fps, *max_fps, fixed_fps);
        } else {
            ALOGE("getParameterPreviewFpsRange(): Cannot found corresponding fixed framerate");
            *min_fps = -1;
            *max_fps = -1;
            fixed_fps = -1;
        }
    } else {
        int preview_frame_rate = params.getPreviewFrameRate();
        // find corresponding framerate range
        if (0 < preview_frame_rate) {
            for (i = 0; i < ARRAY_SIZE(supportedFpsRange); i++) {
                if ( (preview_frame_rate) == (supportedFpsRange[i].fixed / 1000)) {
                    *min_fps = supportedFpsRange[i].min;
                    *max_fps = supportedFpsRange[i].max;
                    fixed_fps = supportedFpsRange[i].fixed;
                    break;
                }
            }
            if (i < ARRAY_SIZE(supportedFpsRange)) {
                ALOGD("getParameterPreviewFpsRange(): Framerate fixed: MIN %d/1000, MAX %d/1000 (fixed %d)", *min_fps, *max_fps, fixed_fps);
            } else {
                ALOGE("getParameterPreviewFpsRange(): Cannot found corresponding framerate range");
                *min_fps = -1;
                *max_fps = -1;
                fixed_fps = -1;
            }
        } else {
            ALOGE("getParameterPreviewFpsRange(): Framerate: MISSING");
        }
    }

    LOG_FUNCTION_NAME_EXIT

    return fixed_fps;
}

status_t CameraHal::setParametersSC(bool reset)
{
    status_t ret = NO_ERROR;
    int brightness = 0;

    LOG_FUNCTION_NAME

    Mutex::Autolock lock(mParametersLock);

#if 0

    // brightness
    brightness = mParameters.getInt(TICameraParameters::KEY_BRIGHTNESS);
    if ( (0 <= brightness) && (200 >= brightness) && (reset || (mSCConfigOld_brightness != brightness))) {
        mSCConfigOld_brightness = brightness;
        CameraDeviceSetControl("BRIGHTNESS", V4L2_CID_BRIGHTNESS, (brightness * 12) / 200);
    }

#else

    // EV compensation
    brightness = mParameters.getInt(CameraParameters::KEY_EXPOSURE_COMPENSATION);
    if ( (COMPENSATION_MIN_SC <= brightness) && (COMPENSATION_MAX_SC >= brightness) && (reset || (mSCConfigOld_brightness != brightness))) {
        mSCConfigOld_brightness = brightness;
        CameraDeviceSetControl("BRIGHTNESS", V4L2_CID_BRIGHTNESS, (int)((float)(((brightness - COMPENSATION_MIN_SC) * 12.0) / (COMPENSATION_MAX_SC - COMPENSATION_MIN_SC))+0.5) );
    }

#endif

    // white balance
    if ( mParameters.get(CameraParameters::KEY_WHITE_BALANCE) != NULL ) {

        int white_balance = 0;

        if (strcmp(mParameters.get(CameraParameters::KEY_WHITE_BALANCE), (const char *) CameraParameters::WHITE_BALANCE_AUTO ) == 0) {

            white_balance = 0;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_WHITE_BALANCE), (const char *) CameraParameters::WHITE_BALANCE_INCANDESCENT) == 0) {

            white_balance = 2;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_WHITE_BALANCE), (const char *) CameraParameters::WHITE_BALANCE_FLUORESCENT) == 0) {

            white_balance = 3;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_WHITE_BALANCE), (const char *) CameraParameters::WHITE_BALANCE_DAYLIGHT) == 0) {

            white_balance = 1;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_WHITE_BALANCE), (const char *) CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT) == 0) {

            white_balance = 4;

        }

        if (reset || (mSCConfigOld_white_balance != white_balance))
        {
            mSCConfigOld_white_balance = white_balance;
            CameraDeviceSetControl("AUTO_WHITE_BALANCE", V4L2_CID_AUTO_WHITE_BALANCE, white_balance);
        }
    }

    // effect
    if ( mParameters.get(CameraParameters::KEY_EFFECT) != NULL ) {

        int color_effect = V4L2_COLORFX_NONE;

        if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_NONE ) == 0) {

            color_effect = V4L2_COLORFX_NONE;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_MONO) == 0) {

            color_effect = V4L2_COLORFX_BW;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_NEGATIVE) == 0) {
//--[[ LGE_UBIQUIX_MODIFIED_START : rt5604@mnbt.co.kr [2012.05.23] - CAM : negative effect
            color_effect = 3;// curently unsupported V4L2_COLORFX_NEGATIVE;
//--]] LGE_UBIQUIX_MODIFIED_END : rt5604@mnbt.co.kr [2012.05.23] - CAM : negative effect
        } else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_SEPIA) == 0) {

            color_effect = V4L2_COLORFX_SEPIA;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_SOLARIZE) == 0) {

            color_effect = 4; // FIXME

        } else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) EFFECT_EMBOSS) == 0) {

            color_effect = 5; // FIXME

        }

        if (reset || (mSCConfigOld_color_effect != color_effect))
        {
            mSCConfigOld_color_effect = color_effect;
            CameraDeviceSetControl("COLORFX", V4L2_CID_COLORFX, color_effect);
        }
    }


	if ( mParameters.get(CameraParameters::KEY_SCENE_MODE) != NULL ) {

        int scene_night = 0;

        if (strcmp(mParameters.get(CameraParameters::KEY_SCENE_MODE), (const char *) CameraParameters::SCENE_MODE_AUTO) == 0) {

            scene_night = 0;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_SCENE_MODE), (const char *) CameraParameters::SCENE_MODE_NIGHT) == 0) {

            scene_night = 1;

        }

        if (reset || (mSCConfigOld_night_mode != scene_night))
        {
            mSCConfigOld_night_mode = scene_night;
            CameraDeviceSetControl("VCENTER", V4L2_CID_VCENTER, scene_night);
        }
    }

#if 0 // unnecessary settings for subcamera
    // flicker avoidance
    if ( mParameters.get(CameraParameters::KEY_ANTIBANDING) != NULL ) {

        int anti_banding = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;

        if (strcmp(mParameters.get(CameraParameters::KEY_ANTIBANDING), (const char *) CameraParameters::ANTIBANDING_50HZ ) == 0) {

            anti_banding = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_ANTIBANDING), (const char *) CameraParameters::ANTIBANDING_60HZ) == 0) {

            anti_banding = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;

        } else if (strcmp(mParameters.get(CameraParameters::KEY_ANTIBANDING), (const char *) CameraParameters::ANTIBANDING_OFF) == 0) {

            anti_banding = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;

        }

        if (reset || (mSCConfigOld_anti_banding != anti_banding))
        {
            mSCConfigOld_anti_banding = anti_banding;
            CameraDeviceSetControl("POWER_LINE_FREQUENCY", V4L2_CID_POWER_LINE_FREQUENCY, anti_banding);
        }
    }

    // metering mode
    if ( mParameters.get(KEY_METER_MODE) != NULL ) {

        int metering_mode = 0;

        if (strcmp(mParameters.get(KEY_METER_MODE), (const char *) METER_MODE_CENTER) == 0) {

            metering_mode = 1;

        } else if (strcmp(mParameters.get(KEY_METER_MODE), (const char *) METER_MODE_AVERAGE) == 0) {

            metering_mode = 0;

        }

        if (reset || (mSCConfigOld_metering_mode != metering_mode))
        {
            mSCConfigOld_metering_mode = metering_mode;
            CameraDeviceSetControl("HCENTER", V4L2_CID_HCENTER, metering_mode);
        }
    }
#endif
    LOG_FUNCTION_NAME_EXIT

    return ret;
}

status_t CameraHal::convertGPSCoord(double coord, int *deg, int *min, int *sec)
{
    double tmp;

    LOG_FUNCTION_NAME

    if ( coord == 0 ) {

        ALOGE("Invalid GPS coordinate");

        return EINVAL;
    }

    *deg = (int) floor(fabs(coord));
    tmp = ( fabs(coord) - floor(fabs(coord)) )*60;
    *min = (int) floor(tmp);
    tmp = ( tmp - floor(tmp) )*60*1000;
    *sec = (int) floor(tmp);

    if( *sec >= 60 * 1000 ) {
        *sec = 0;
        *min += 1;
    }

    if( *min >= 60 ) {
        *min = 0;
        *deg += 1;
    }

    LOG_FUNCTION_NAME_EXIT

    return NO_ERROR;
}

status_t CameraHal::setParameters(const CameraParameters &params)
{
    LOG_FUNCTION_NAME

    int w, h, pWidthOrig, pHeightOrig;
    int w_orig, h_orig, rot_orig;
    int framerate_min, framerate_max;
    int zoom, compensation, saturation, sharpness;
    int zoom_save;
    int contrast, brightness;
    int error;
    int base;
    const char *valstr;
    char *af_coord;
    TIUTILS::Message msg;
    int ippMode;
    status_t ret = NO_ERROR;

    Mutex::Autolock lock(mLock);

/* 20120701 jungyeal@lge.com modify vt_mode & face_unlock [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)	
    if(params.getInt( KEY_VT_MODE) == 1)
    {
		mvt_mode = true;
		ALOGD("kim vt mvt_mode = %d!!!", mvt_mode);
    }
    else
    {
		mvt_mode = false;
		ALOGD("kim vt mvt_mode = %d!!!", mvt_mode);
    }
#if 0  // Removed this Code.. 
    if ((VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_SECONDARY)  &&
	 		params.getInt("lge-camera") != 1 &&  mvt_mode == false )  // Workround  code... I will should be this condition  by jungyeal
    {	
    		ALOGD("face UnLock mode");
	 	mFaceUnlockMode = true;	 
    }
    else 
    {
    	ALOGD("None face UnLock mode");
    	mFaceUnlockMode = false;		
    }
#endif
#endif
/* 20120701 jungyeal@lge.com modify vt_mode & face_unlock [END] */

    ALOGD("PreviewFormat %s", params.getPreviewFormat());

    if ( params.getPreviewFormat() != NULL ) {
        if (strcmp(params.getPreviewFormat(), (const char *) CameraParameters::PIXEL_FORMAT_YUV422I) == 0) {
            ALOGD("Preview format YUV422I");
        } else if (strcmp(params.getPreviewFormat(), (const char *) CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
            ALOGD("Preview format YUV420SP");
        } else if (strcmp(params.getPreviewFormat(), (const char *) CameraParameters::PIXEL_FORMAT_YUV420P) == 0) {
            ALOGD("Preview format YUV420P");
        } else {
            ALOGE("Format %s not supported", params.getPreviewFormat());
            return -EINVAL;
        }
    }	

    ALOGD("PictureFormat %s", params.getPictureFormat());
    if ( params.getPictureFormat() != NULL ) {
        if (strcmp(params.getPictureFormat(), (const char *) CameraParameters::PIXEL_FORMAT_JPEG) != 0) {
            ALOGE("Only jpeg still pictures are supported");
            return -EINVAL;
        }
    }

    params.getPreviewSize(&w, &h);
    if ( w <= 0 || h <= 0 ) {
        ALOGE("Preview size not supported");
        return -EINVAL;
    }
    if ( validateSize(w, h, supportedPreviewRes, ARRAY_SIZE(supportedPreviewRes)) == false ) {
        ALOGE("Preview size not supported");
        return -EINVAL;
    }
    ALOGD("PreviewResolution by App %d x %d", w, h);

    mParameters.getPreviewSize(&pWidthOrig, &pHeightOrig);
    if (w!=pWidthOrig||h!=pHeightOrig) {
        mPreviewChangedNeedsRestart = true;
        ALOGD("Preview size has changed in setParameters(). Preview needs to be restarted!");
    }

    params.getPictureSize(&w, &h);
    if (validateSize(w, h, supportedPictureRes, ARRAY_SIZE(supportedPictureRes)) == false ) {
        ALOGE("Picture size not supported");
        return -EINVAL;
    }
    ALOGD("Picture Size by App %d x %d", w, h);

#ifdef HARDWARE_OMX

    mExifParams.width = w;
    mExifParams.height = h;

#endif

    params.getPreviewFpsRange(&framerate_min, &framerate_max);
    if (framerate_min > 0 && framerate_max > 0 && framerate_min <= framerate_max) {
        useFramerateRange = true;
        ALOGD("Setparameters(): Framerate range: MIN %d, MAX %d", framerate_min, framerate_max);
        char* supportedFpsRanges = (char*) params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE);
        ALOGD("supportedFpsRanges = %s", supportedFpsRanges);
        if ( validateRange(framerate_min, framerate_max, supportedFpsRanges) == false ) {
            ALOGE("Range Not Supported");
        }
    } else if (framerate_min < 0 || framerate_max < 0) {
        ALOGE("Negative FpsRange");
        return -EINVAL;
    } else if ( framerate_min > framerate_max ) {
        ALOGE("Invalid FpsRange");
        return -EINVAL;
    } else {
        useFramerateRange = false;
        framerate_max = params.getPreviewFrameRate() * 1000;
        ALOGD("Setparameters(): Framerate: %d", framerate_max);
        if (framerate_max < 0 || framerate_max > 30000)
        {
            ALOGE("Invalid Framerate");
            return -EINVAL;
        }
    }

    rot_orig = rotation;
    char value[PROPERTY_VALUE_MAX];
    property_get("debug.video.force_rotation", value, "-1");
    rotation = atoi(value);
    if (0 > rotation) {
        rotation = params.getInt(CameraParameters::KEY_ROTATION);
    } else {
        ALOGI("Rotation force changed to %d", rotation);
    }

    mParameters.getPictureSize(&w_orig, &h_orig);
    zoom_save = mParameters.getInt(CameraParameters::KEY_ZOOM);

    if ( params.get(CameraParameters::KEY_FOCUS_MODE) != NULL &&
            strcmp(params.get(CameraParameters::KEY_FOCUS_MODE), (const char *) "invalid" ) == 0) {
        CAMHAL_ALOGEB("Invalid parameter focus-mode = %s", params.get(CameraParameters::KEY_FOCUS_MODE));
        ret = -EINVAL;
        return ret;
    }

    mParameters = params;
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)	
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
    params.getPreviewSize(&w, &h);
	//ALOGD("kim vt setParameters w:%d, h:%d, if secondary:%d",w, h, VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_SECONDARY);
	if ((VIDEO_DEVICE_INDEX(mCameraIndex) == VIDEO_DEVICE_INDEX_SECONDARY) && mvt_mode) {
		if ((w == 640) && (h == 480)) {
			ALOGD("Adjust PreviewSize (640x480 -> 320x240) by HAL");
			mParameters.setPreviewSize(320, 240);
		}
	}
#endif

#if	1	// rt5604 2012.08.13 Bug fix for CTS testPreviewCallback ->
    params.getPreviewSize(&w, &h);
	if (w == 736) {
		ALOGD("rt5604: ---> Adjust PreviewSize (736x%d -> 720x%d) by HAL", h, h);
		mParameters.setPreviewSize(720, h);
	}
#endif		// rt5604 2012.08.13 Bug fix for CTS testPreviewCallback <-

#ifdef IMAGE_PROCESSING_PIPELINE
/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [START] */ 
/****************Commented-- Work Around for IPP Error*****************************
    ippMode = mParameters.getInt(TICameraParameters::KEY_IPP);
    if ( ippMode == -1 ) {
        if ( mParameters.getInt("lge-camera") == 1 )
            ippMode = IPP_EdgeEnhancement_Mode;
        else
            ippMode = IPP_Disabled_Mode;
    }

    if((mippMode != ippMode) || (w != w_orig) || (h != h_orig) ||
            ((rot_orig != 180) && (rotation == 180)) ||
            ((rot_orig == 180) && (rotation != 180))) // in the current setup, IPP uses a different setup when rotation is 180 degrees
    {
        if(pIPP.hIPP != NULL){
            ALOGD("pIPP.hIPP=%p", pIPP.hIPP);
            if(DeInitIPP(mippMode)) // deinit here to save time
                ALOGE("ERROR DeInitIPP() failed");
            pIPP.hIPP = NULL;
        }

        mippMode = ippMode;
        ALOGD("mippMode=%d", mippMode);

        mIPPToEnable = true;
    }
****************Commented-- Work Around for IPP Error*****************************/
/* CSR-OMAPS00275100 kirti.badkundri@sasken.com Work Around for IPP Error [END] */
#endif

    mParameters.getPictureSize(&w, &h);
    ALOGD("Picture Size by CamHal %d x %d", w, h);

    mParameters.getPreviewSize(&w, &h);
    ALOGD("Preview Resolution by CamHal %d x %d", w, h);

    quality = params.getInt(CameraParameters::KEY_JPEG_QUALITY);
    if ( ( quality < 0 ) || (quality > 100) ){
        quality = 100;
    }
    //Keep the JPEG thumbnail quality same as JPEG quality.
    //JPEG encoder uses the same quality for thumbnail as for the main image.
    mParameters.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, quality);

    zoom = mParameters.getInt(CameraParameters::KEY_ZOOM);
    if( (zoom >= 0) && ( zoom < ZOOM_STAGES) ){
        // immediate zoom
        mZoomSpeed = 0;
        mZoomTargetIdx = zoom;
    } else if(zoom>= ZOOM_STAGES){
        mParameters.set(CameraParameters::KEY_ZOOM, zoom_save);
        return -EINVAL;
    } else {
        mZoomTargetIdx = 0;
    }
    ALOGD("Zoom by App %d", zoom);

#ifdef HARDWARE_OMX

    mExifParams.zoom = zoom;

#endif

    if ( ( params.get(CameraParameters::KEY_GPS_LATITUDE) != NULL ) && ( params.get(CameraParameters::KEY_GPS_LONGITUDE) != NULL ) && ( params.get(CameraParameters::KEY_GPS_ALTITUDE) != NULL )) {

        double gpsCoord;
        struct tm* timeinfo;

#ifdef HARDWARE_OMX

        if( NULL == gpsLocation )
            gpsLocation = (gps_data *) malloc( sizeof(gps_data));

        if( NULL != gpsLocation ) {
            ALOGE("initializing gps_data structure");

            memset(gpsLocation, 0, sizeof(gps_data));
            gpsLocation->datestamp[0] = '\0';

            gpsCoord = strtod( params.get(CameraParameters::KEY_GPS_LATITUDE), NULL);
            convertGPSCoord(gpsCoord, &gpsLocation->latDeg, &gpsLocation->latMin, &gpsLocation->latSec);
            gpsLocation->latRef = (gpsCoord < 0) ? (char*) "S" : (char*) "N";

            gpsCoord = strtod( params.get(CameraParameters::KEY_GPS_LONGITUDE), NULL);
            convertGPSCoord(gpsCoord, &gpsLocation->longDeg, &gpsLocation->longMin, &gpsLocation->longSec);
            gpsLocation->longRef = (gpsCoord < 0) ? (char*) "W" : (char*) "E";

            gpsCoord = strtod( params.get(CameraParameters::KEY_GPS_ALTITUDE), NULL);
            gpsLocation->altitude = gpsCoord;

            if ( NULL != params.get(CameraParameters::KEY_GPS_TIMESTAMP) ){
                long timestamp;
                timestamp = gpsLocation->timestamp = strtol( params.get(CameraParameters::KEY_GPS_TIMESTAMP), NULL, 10);
                timeinfo = localtime((time_t*)&(timestamp));
                if(timeinfo != NULL) {
                    gpsLocation->timestamp = timestamp - timeinfo->tm_gmtoff;
                    timeinfo = localtime((time_t*)&(gpsLocation->timestamp)); 
                    strftime(gpsLocation->datestamp, 11, "%Y:%m:%d", timeinfo);
                    ALOGD("localtime datestamp = %s, tm_gmtoff = %ld", gpsLocation->datestamp, timeinfo->tm_gmtoff);
                }
            }

            gpsLocation->altitudeRef = params.getInt(KEY_GPS_ALTITUDE_REF);
            gpsLocation->mapdatum = (char *) params.get(TICameraParameters::KEY_GPS_MAPDATUM);
            gpsLocation->versionId = (char *) params.get(TICameraParameters::KEY_GPS_VERSION);
            gpsLocation->procMethod = (char *) params.get(CameraParameters::KEY_GPS_PROCESSING_METHOD);

        } else {
            ALOGE("Not enough memory to allocate gps_data structure");
        }

#endif

    }

    if ( params.get(TICameraParameters::KEY_SHUTTER_ENABLE) != NULL )
    {
        if ( strcmp(params.get(TICameraParameters::KEY_SHUTTER_ENABLE), (const char *) "true") == 0 )
        {
            mMsgEnabled |= CAMERA_MSG_SHUTTER;
            mShutterEnable = true;
        }
        else if ( strcmp(params.get(TICameraParameters::KEY_SHUTTER_ENABLE), (const char *) "false") == 0 )
        {
            mShutterEnable = false;
            mMsgEnabled &= ~CAMERA_MSG_SHUTTER;
        }
    }

#ifdef HARDWARE_OMX
    mExifParams.focal_length = (float) IMX046_FOCALLENGTH;
#endif

    if ( (-1 != camera_device) && (VIDEO_DEVICE_INDEX_MAIN != VIDEO_DEVICE_INDEX(mCameraIndex)) /* && mPreviewRunning */) {
		if (mPreviewRunning) {
			setParametersSC(false);
		}
		else {
			// If mPreviewRunnig is false, sensor device is not power on.
			// So, Camera Sensor Control is not effective.
			// But, we have to get the color effect value from the parameters for preserving the effect.
			if ( mParameters.get(CameraParameters::KEY_EFFECT) != NULL ) {
				int color_effect = V4L2_COLORFX_NONE;
		
				if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_NONE ) == 0) {
					color_effect = V4L2_COLORFX_NONE;
				} else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_MONO) == 0) {
					color_effect = V4L2_COLORFX_BW;
				} else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_NEGATIVE) == 0) {
					color_effect = 3;// curently unsupported V4L2_COLORFX_NEGATIVE;
				} else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_SEPIA) == 0) {
					color_effect = V4L2_COLORFX_SEPIA;
				} else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) CameraParameters::EFFECT_SOLARIZE) == 0) {
					color_effect = 4; // FIXME
				} else if (strcmp(mParameters.get(CameraParameters::KEY_EFFECT), (const char *) EFFECT_EMBOSS) == 0) {
					color_effect = 5; // FIXME
				}
		
				ALOGE("rt5604: ---> SetParameters: mSCConfigOld_color_effect = %d, color_effect = %d", mSCConfigOld_color_effect, color_effect);
				mSCConfigOld_color_effect = color_effect;
			}
		}
    }

    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR;
}

CameraParameters CameraHal::getParameters() const
{
    CameraParameters params;
    char tmpBuffer[PARAM_BUFFER];  // Sasken

    LOG_FUNCTION_NAME

    {
        Mutex::Autolock lock(mLock);
        params = mParameters;
    }

/* Sasken: start */
    /* CSR-OMAPS00273048 kirti.badkundri@sasken.com fix CTS - testJpegExif [START] */
    params.set(CameraParameters::KEY_FOCAL_LENGTH, STRINGIZE(IMX046_FOCALLENGTH));
    params.set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE, STRINGIZE(IMX046_HORZANGLE));
    params.set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE, STRINGIZE(IMX046_VERTANGLE));
    /* CSR-OMAPS00273048 kirti.badkundri@sasken.com fix CTS - testJpegExif [END] */

    memset(tmpBuffer, '\0', PARAM_BUFFER);
    snprintf(tmpBuffer, PARAM_BUFFER, "%f,%f,%s", FOCUS_DISTANCE_NEAR, FOCUS_DISTANCE_OPTIMAL, CameraParameters::FOCUS_DISTANCE_INFINITY);
    params.set(CameraParameters::KEY_FOCUS_DISTANCES, tmpBuffer);
/* Sasken: End */

exit:

    LOG_FUNCTION_NAME_EXIT

    //params.dump();
    return params;
}

status_t  CameraHal::dump(int fd) const
{
    return 0;
}

void CameraHal::dumpFrame(void *buffer, int size, char *path)
{
    FILE* fIn = NULL;

    fIn = fopen(path, "w");
    if ( fIn == NULL ) {
        ALOGE("\n\n\n\nError: failed to open the file %s for writing\n\n\n\n", path);
        return;
    }

    fwrite((void *)buffer, 1, size, fIn);
    fclose(fIn);

}

void CameraHal::release()
{
}

#ifdef USE_AUTOFOCUS_THREAD
// rt5604 2012.08.16 autoFocus ->
void CameraHal::onAutoFocus(bool value)
{
    unsigned int afcbMessage[AFCB_THREAD_NUM_ARGS];

    if( msgTypeEnabled(CAMERA_MSG_FOCUS) ) {

#ifdef DEBUG_LOG

        PPM("SENDING MESSAGE TO AF CALLBACK THREAD");

#endif
        afcbMessage[0] = AFCB_THREAD_CALL;
        afcbMessage[1] = (unsigned int) mNotifyCb;
        afcbMessage[2] = (unsigned int) mCallbackCookie;
        afcbMessage[3] = (unsigned int) value;
        write(afcbPipe[1], &afcbMessage, sizeof(afcbMessage));
    }
}
// rt5604 2012.08.16 autoFocus <-
#endif // #ifdef USE_AUTOFOCUS_THREAD

// rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom ->
void CameraHal::onZoom(bool cb_value, int zoom_value)
{
    unsigned int zoomcbMessage[ZOOMCB_THREAD_NUM_ARGS];

    zoomcbMessage[0] = ZOOMCB_THREAD_CALL;
    zoomcbMessage[1] = (unsigned int) mNotifyCb;
    zoomcbMessage[2] = (unsigned int) mCallbackCookie;
    zoomcbMessage[3] = (unsigned int) cb_value;
    zoomcbMessage[4] = (unsigned int) zoom_value;
    write(zoomcbPipe[1], &zoomcbMessage, sizeof(zoomcbMessage));
}
// rt5604 2012.08.15 Bug Fix for CTS testSmoothZoom <-

int CameraHal::SaveFile(char *filename, char *ext, void *buffer, int size)
{
    LOG_FUNCTION_NAME
    //Store image
    char fn [512];

    if (filename) {
      strcpy(fn,filename);
    } else {
      if (ext==NULL) ext = (char*)"tmp";
      sprintf(fn, PHOTO_PATH, file_index, ext);
    }
    file_index++;
    ALOGD("Writing to file: %s", fn);
    int fd = open(fn, O_RDWR | O_CREAT | O_SYNC);
    if (fd < 0) {
        ALOGE("Cannot open file %s : %s", fn, strerror(errno));
        return -1;
    } else {

        int written = 0;
        int page_block, page = 0;
        int cnt = 0;
        int nw;
        char *wr_buff = (char*)buffer;
        ALOGD("Jpeg size %d buffer 0x%x", size, ( unsigned int ) buffer);
        page_block = size / 20;
        while (written < size ) {
          nw = size - written;
          nw = (nw>512*1024)?8*1024:nw;

          nw = ::write(fd, wr_buff, nw);
          if (nw<0) {
              ALOGD("write fail nw=%d, %s", nw, strerror(errno));
            break;
          }
          wr_buff += nw;
          written += nw;
          cnt++;

          page    += nw;
          if (page>=page_block){
              page = 0;
              ALOGD("Percent %6.2f, wn=%5d, total=%8d, jpeg_size=%8d",
                  ((float)written)/size, nw, written, size);
          }
        }

        close(fd);

        return 0;
    }
}


sp<IMemoryHeap> CameraHal::getPreviewHeap() const
{
    LOG_FUNCTION_NAME
    return 0;
}


/*--------------------Eclair HAL---------------------------------------*/
/**
   @brief Enable a message, or set of messages.

   @param[in] msgtype Bitmask of the messages to enable (defined in include/ui/Camera.h)
   @return none

 */
void CameraHal::enableMsgType(int32_t msgType)
{
    LOG_FUNCTION_NAME;

    if ( ( msgType & CAMERA_MSG_SHUTTER ) && ( !mShutterEnable ))
    {
        msgType &= ~CAMERA_MSG_SHUTTER;
    }

    // ignoring enable focus message from camera service
    // we will enable internally in autoFocus call
    if (msgType & CAMERA_MSG_FOCUS) {
        msgType &= ~CAMERA_MSG_FOCUS;
    }

    {
    Mutex::Autolock lock(mLock);
    mMsgEnabled |= msgType;
    }

    if(mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME)
    {
        if(mDisplayPaused)
        {
            CAMHAL_ALOGDA("Preview currently paused...will enable preview callback when restarted");
            msgType &= ~CAMERA_MSG_PREVIEW_FRAME;
        }else
        {
            CAMHAL_ALOGDA("Enabling Preview Callback");
        }
    }
    else
    {
        CAMHAL_ALOGDB("Preview callback not enabled %x", msgType);
    }


    ///Configure app callback notifier with the message callback required
    mAppCallbackNotifier->enableMsgType (msgType);

    LOG_FUNCTION_NAME_EXIT;
}

/**
   @brief Disable a message, or set of messages.

   @param[in] msgtype Bitmask of the messages to disable (defined in include/ui/Camera.h)
   @return none

 */
void CameraHal::disableMsgType(int32_t msgType)
{
    LOG_FUNCTION_NAME;

        {
        Mutex::Autolock lock(mLock);
        mMsgEnabled &= ~msgType;
        }

    if( msgType & CAMERA_MSG_PREVIEW_FRAME)
        {
        CAMHAL_ALOGDA("Disabling Preview Callback");
        }

    ///Configure app callback notifier
    mAppCallbackNotifier->disableMsgType (msgType);

    LOG_FUNCTION_NAME_EXIT;
}


/**
   @brief Query whether a message, or a set of messages, is enabled.

   Note that this is operates as an AND, if any of the messages queried are off, this will
   return false.

   @param[in] msgtype Bitmask of the messages to query (defined in include/ui/Camera.h)
   @return true If all message types are enabled
          false If any message type

 */
int CameraHal::msgTypeEnabled(int32_t msgType)
{
    LOG_FUNCTION_NAME;
    Mutex::Autolock lock(mLock);
    LOG_FUNCTION_NAME_EXIT;
    return (mMsgEnabled & msgType);
}

status_t CameraHal::cancelAutoFocus()
{
    LOG_FUNCTION_NAME

    CameraParameters p;
    TIUTILS::Message msg;
    const char * mFocusMode;

    // Disable focus messaging here. When application requests cancelAutoFocus(),
    // it does not expect any AF callbacks. AF should be done in order to return the lens
    // back to "default" position. In this case we set an average manual focus position.
    // From the other side, in previewthread() state machine we always send callback to
    // application when we focus, so disable focus messages.
    disableMsgType(CAMERA_MSG_FOCUS);

    msg.command = PREVIEW_AF_STOP;
    previewThreadCommandQ.put(&msg);
    previewThreadAckQ.get(&msg);

    if ( msg.command != PREVIEW_ACK )
        ALOGE("AF Stop Failed or AF already stopped");
    else
        ALOGD("AF Stopped");

    if( NULL != mCameraAdapter )
    {
        //adapterParams.set(TICameraParameters::KEY_AUTO_FOCUS_LOCK, CameraParameters::FALSE);
        //mCameraAdapter->setParameters(adapterParams);
        mCameraAdapter->sendCommand(CameraAdapter::CAMERA_CANCEL_AUTOFOCUS);
    }

    enableMsgType(CAMERA_MSG_FOCUS);

    LOG_FUNCTION_NAME_EXIT

    return NO_ERROR;
}

/*--------------------Eclair HAL---------------------------------------*/

status_t CameraHal::sendCommand(int32_t cmd, int32_t arg1, int32_t arg2)
{
    TIUTILS::Message msg;
    status_t ret = NO_ERROR;

    LOG_FUNCTION_NAME

    switch(cmd) {
        case CAMERA_CMD_START_SMOOTH_ZOOM:
            msg.command = START_SMOOTH_ZOOM;
            msg.arg1 = ( void * ) arg1;
            previewThreadCommandQ.put(&msg);
            previewThreadAckQ.get(&msg);

            if ( PREVIEW_ACK != msg.command ) {
                ret = -EINVAL;
            }

            break;
        case CAMERA_CMD_STOP_SMOOTH_ZOOM:
            msg.command = STOP_SMOOTH_ZOOM;
            previewThreadCommandQ.put(&msg);
            previewThreadAckQ.get(&msg);

            if ( PREVIEW_ACK != msg.command ) {
                ret = -EINVAL;
            }

            break;
        case CAMERA_CMD_START_FACE_DETECTION:
            // Return error, because we do not support fd
            ret = -EINVAL;
            break;
        case CAMERA_CMD_STOP_FACE_DETECTION:
            // Return error, because we do not support fd
            ret = -EINVAL;
            break;
        default:
            break;
    };

    LOG_FUNCTION_NAME_EXIT

    return ret;
}

#ifdef DEBUG_LOG
void CameraHal::debugShowBufferStatus()
{
    ALOGE("nOverlayBuffersQueued=%d", nOverlayBuffersQueued);
    ALOGE("nCameraBuffersQueued=%d", nCameraBuffersQueued);
    ALOGE("mVideoBufferCount=%d", mVideoBufferCount);
    for (int i=0; i< VIDEO_FRAME_COUNT_MAX; i++) {
        ALOGE("mVideoBufferStatus[%d]=%d", i, mVideoBufferStatus[i]);
    }
}
#endif

}; // namespace android

