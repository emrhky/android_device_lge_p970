/*
 * Copyright (C) Texas Instruments - http://www.ti.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
* @file V4LCameraAdapter.cpp
*
* This file maps the Camera Hardware Interface to V4L2.
*
*/


#include "V4LCameraAdapter.h"
#include "CameraHal.h"
#include "TICameraParameters.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev.h>

#include <cutils/properties.h>
#define UNLIKELY( exp ) (__builtin_expect( (exp) != 0, false ))
static int mDebugFps = 0;

#define Q16_OFFSET 16

#include <sys/mman.h>
#include <sys/eventfd.h>
#include <ion.h>

#include "../../omap3/hwc/hal_public.h"
#define HERE(Msg) {CAMHAL_ALOGEB("--===line %d, %s===--\n", __LINE__, Msg);}

namespace android {

// 20120628 jungyeal@lge.com sub cam rotation patch from mms
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
extern int YUV422I_rotate(const uint8_t *src, uint8_t *dst, int src_w, int src_h);
extern int YUV422I_rotate_270(const uint8_t *src, uint8_t *dst, int src_w, int src_h);
#endif
#undef LOG_TAG
///Maintain a separate tag for V4LCameraAdapter logs to isolate issues OMX specific
#define LOG_TAG "V4LCameraAdapter"

//frames skipped before recalculating the framerate
#define FPS_PERIOD 30

Mutex gAdapterLock;
const char *device = DEVICE;


/*--------------------Camera Adapter Class STARTS here-----------------------------*/

status_t V4LCameraAdapter::initialize(int CameraHandle)
{
    LOG_FUNCTION_NAME;

    char value[PROPERTY_VALUE_MAX];
    property_get("debug.camera.showfps", value, "0");
    mDebugFps = atoi(value);

    int ret = NO_ERROR;

    // Allocate memory for video info structure
    mVideoInfo = (struct VideoInfo *) calloc (1, sizeof (struct VideoInfo));
    if(!mVideoInfo)
    {
        return NO_MEMORY;
     }
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
    //ALOGI("kim vt  V4LCameraAdapter::initialize");
    mVideoInfo->rotation_buf = (char *) calloc (1, 640 * 480 * 2);
    CAMHAL_ALOGDB("allocated %d bytes at 0x%x for rotation_buf", 640 * 480 * 2, mVideoInfo->rotation_buf);
#endif
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */

    mCameraHandle = CameraHandle;

    ret = ioctl (mCameraHandle, VIDIOC_QUERYCAP, &mVideoInfo->cap);
    if (ret < 0)
        {
        CAMHAL_ALOGEA("Error when querying the capabilities of the V4L Camera");
        return -EINVAL;
        }

    if ((mVideoInfo->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0)
        {
        CAMHAL_ALOGEA("Error while adapter initialization: video capture not supported.");
        return -EINVAL;
        }

    if (!(mVideoInfo->cap.capabilities & V4L2_CAP_STREAMING))
        {
        CAMHAL_ALOGEA("Error while adapter initialization: Capture device does not support streaming i/o");
        return -EINVAL;
        }

    // Initialize flags
    mPreviewing = false;
    mVideoInfo->isStreaming = false;

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

status_t V4LCameraAdapter::fillThisBuffer(void* frameBuf, CameraFrame::FrameType frameType)
{
    status_t ret = NO_ERROR;

    if((NO_ERROR == ret) && ((frameType == CameraFrame::IMAGE_FRAME) || (CameraFrame::RAW_FRAME == frameType)))
    {
    	CAMHAL_ALOGEA(" This is an image capture frame ");
        
        // Signal end of image capture
        if ( NULL != mEndImageCaptureCallback) {
            mEndImageCaptureCallback(mEndCaptureData);
        }

        return NO_ERROR;
    }

    if ( !mVideoInfo->isStreaming )
    {
        return NO_ERROR;
    }

    int i =  mPreviewBufs.valueFor(( unsigned int )frameBuf);

    if(i<0)
    {
        return BAD_VALUE;
    }

    mVideoInfo->buf.index = i;
    mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->buf.memory = V4L2_MEMORY_USERPTR;
    mVideoInfo->buf.m.userptr = (unsigned long)mIonHandle.keyAt(i);

    ret = ioctl(mCameraHandle, VIDIOC_QBUF, &mVideoInfo->buf);
    if (ret < 0) {
       CAMHAL_ALOGEB("Init: VIDIOC_QBUF Failed ret = %d; errno=%d (%s)", ret, errno, strerror(errno));
       return -1;
    }

    nQueued++;

    return ret;

}

status_t V4LCameraAdapter::setParameters(const CameraParameters &params)
{
    LOG_FUNCTION_NAME;

    status_t ret = NO_ERROR;

    // Udpate the current parameter set
    mParams = params;

    LOG_FUNCTION_NAME_EXIT;
    return ret;
}


void V4LCameraAdapter::getParameters(CameraParameters& params)
{
    LOG_FUNCTION_NAME;

    // Return the current parameter set
    params = mParams;

    LOG_FUNCTION_NAME_EXIT;
}


///API to give the buffers to Adapter
status_t V4LCameraAdapter::useBuffers(CameraMode mode, void* bufArr, int num, size_t length, unsigned int queueable)
{
    status_t ret = NO_ERROR;

    LOG_FUNCTION_NAME;

    Mutex::Autolock lock(mLock);

    switch(mode)
        {
        case CAMERA_PREVIEW:
            ret = UseBuffersPreview(bufArr, num);
            break;

        case CAMERA_IMAGE_CAPTURE:
        	ret = UseBuffersImageCapture(bufArr, num);
        	break;

        case CAMERA_VIDEO:
            //@warn Video capture is not fully supported yet
            ret = UseBuffersPreview(bufArr, num);
            break;
        }

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

char** V4LCameraAdapter:: getVirtualAddress(int count)
{
    char** buf = new char*[6];

    for(int i = 0; i < count; i ++)
    {
        buf[i] = (char *)mIonHandle.keyAt(i);
    }
    return buf;
}

status_t V4LCameraAdapter::UseBuffersPreview(void* bufArr, int num)
{
    LOG_FUNCTION_NAME;
    
    int ret = NO_ERROR;
    int width, height;

    if(NULL == bufArr)
    {
        return BAD_VALUE;
    }

    /* Check if camera can handle NB_BUFFER buffers */
    mVideoInfo->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->rb.memory = V4L2_MEMORY_USERPTR;
    mVideoInfo->rb.count = num;

    ret = ioctl(mCameraHandle, VIDIOC_REQBUFS, &mVideoInfo->rb);
    if (ret < 0) {
        CAMHAL_ALOGEB("VIDIOC_REQBUFS failed: %s", strerror(errno));
        return ret;
    }


    for (int i = 0; i < num; i++)
    {
        memset (&mVideoInfo->buf, 0, sizeof (struct v4l2_buffer));

        mVideoInfo->buf.index = i;
        mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        mVideoInfo->buf.memory = V4L2_MEMORY_USERPTR;

        ret = ioctl (mCameraHandle, VIDIOC_QUERYBUF, &mVideoInfo->buf);
        if (ret < 0) {
            CAMHAL_ALOGEB("Unable to query buffer (%s)", strerror(errno));
            return ret;
        }
    }

    if( mIonHandle.isEmpty() )
    {
	    void* buff_t;

	    struct ion_map_gralloc_to_ionhandle_data data;

	    ion_fd = ion_open();
	    CAMHAL_ALOGEB( "ion_fd = %d", ion_fd );
	    if (ion_fd < 0)
	    {
	        ALOGE("ion_open fail !!!");
	        return BAD_VALUE;
	    }

	    for (int i = 0; i < num; i++) {

		uint32_t *ptr = (uint32_t*) bufArr;

		//Associate each Camera internal buffer with the one from Overlay
		mPreviewBufs.add((int)ptr[i], i);

		mParams.getPreviewSize(&width, &height);

		data.gralloc_handle = (int *)((IMG_native_handle_t*)ptr[i])->fd[0];
		CAMHAL_ALOGEB("data.gralloc_handle = %d", data.gralloc_handle);

		if (ion_ioctl(ion_fd, ION_IOC_MAP_GRALLOC, &data)) {
			ALOGE("ion_ioctl fail");
			return BAD_VALUE;
		}

		ALOGE("data.handleY = %x", data.handleY);

		if (ion_map(ion_fd, data.handleY, (width*height*2), PROT_READ | PROT_WRITE,
                    MAP_SHARED, 0, (unsigned char **)&buff_t, &mmap_fd[i]) < 0) {
			ALOGE("ION map failed");
			return BAD_VALUE;
		}

		CAMHAL_ALOGEB(" buff_t is %x ", buff_t);

		mIonHandle.add((void*)buff_t,i);
	   }
    }

    // Update the preview buffer count
    mPreviewBufferCount = num;
    LOG_FUNCTION_NAME_EXIT;

    return ret;
}


status_t V4LCameraAdapter::UseBuffersImageCapture(void* bufArr, int num)
{
    LOG_FUNCTION_NAME;
    
	status_t ret = NO_ERROR;
    mVideoInfo->buf.index = 0;
    mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->buf.memory = V4L2_MEMORY_USERPTR;
    mVideoInfo->buf.m.userptr = (unsigned long)bufArr;

    ret = ioctl(mCameraHandle, VIDIOC_QBUF, &mVideoInfo->buf);
    if (ret < 0) {
        CAMHAL_ALOGEA("VIDIOC_QBUF Failed");
        return -EINVAL;
    }
    nQueued++;
    
    LOG_FUNCTION_NAME_EXIT;
    return ret;
}

status_t V4LCameraAdapter::takePicture()
{
    LOG_FUNCTION_NAME;
    
    status_t ret = NO_ERROR;
    /* turn on streaming */
    enum v4l2_buf_type bufType;
    if (!mVideoInfo->isStreaming)
    {
        bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        ret = ioctl (mCameraHandle, VIDIOC_STREAMON, &bufType);
        if (ret < 0)
        {
            CAMHAL_ALOGEB("StartStreaming: Unable to start capture: %s", strerror(errno));
            return ret;
        }
        CAMHAL_ALOGDA("VIDIOC_STREAMON");
        mVideoInfo->isStreaming = true;
    }

    CAMHAL_ALOGEA("takePicture : De-queue the next avaliable buffer");

    /* De-queue the next avaliable buffer */
    mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->buf.memory = V4L2_MEMORY_USERPTR;

    /* DQ */
    ret = ioctl(mCameraHandle, VIDIOC_DQBUF, &mVideoInfo->buf);
    if (ret < 0) {
        CAMHAL_ALOGEB("GetFrame: VIDIOC_DQBUF Failed ret = %d; errno=%d (%s)", ret, errno, strerror(errno));
        return NULL;
    }
    nDequeued++;

    notifyShutterSubscribers();

    /* turn off streaming */
    bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(mCameraHandle, VIDIOC_STREAMOFF, &bufType) < 0) {
        ALOGE("VIDIOC_STREAMOFF Failed errno=%d (%s)", errno, strerror(errno));
        return -1;
    }
    CAMHAL_ALOGDA("VIDIOC_STREAMOFF");
    mVideoInfo->isStreaming = false;
    
    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

// rt5604 -> to reduce KPI 2012.07.09
int skip_stream_on = 0;
// rt5604 <- to reduce KPI 2012.07.09

status_t V4LCameraAdapter::startPreview(char* indexes)
{
    status_t ret = NO_ERROR;

    Mutex::Autolock lock(mPreviewBufsLock);

    if(mPreviewing)
    {
        return BAD_VALUE;
    }

   for (int i = 0; i < mPreviewBufferCount; i++)
   {
       if (indexes != NULL && indexes[i] == '1') {
           //CAMHAL_ALOGDB("don't QBUF buffer %d", i);
           continue;
       }

       mVideoInfo->buf.index = i;
       mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
       mVideoInfo->buf.memory = V4L2_MEMORY_USERPTR;

       mVideoInfo->buf.m.userptr = (unsigned long)mIonHandle.keyAt(i);

       ret = ioctl(mCameraHandle, VIDIOC_QBUF, &mVideoInfo->buf);
       if (ret < 0) {
           CAMHAL_ALOGEB("VIDIOC_QBUF Failed ret = %d; errno=%d (%s)", ret, errno, strerror(errno));
           return -EINVAL;
       }

       nQueued++;
   }

   enum v4l2_buf_type bufType;
   
// rt5604 -> to reduce KPI 2012.07.09
   if (!skip_stream_on)
// rt5604 <- to reduce KPI 2012.07.09
   if (!mVideoInfo->isStreaming)
   {
       bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;

       ret = ioctl (mCameraHandle, VIDIOC_STREAMON, &bufType);
       if (ret < 0) {
           CAMHAL_ALOGEB("StartStreaming: Unable to start capture: %s", strerror(errno));
           return ret;
       }
       CAMHAL_ALOGDA("VIDIOC_STREAMON");

       mVideoInfo->isStreaming = true;
   }

   //Update the flag to indicate we are previewing
   mPreviewing = true;

   return ret;
}

status_t V4LCameraAdapter::stopPreview(bool check)
{
    enum v4l2_buf_type bufType;
    int ret = NO_ERROR;
    int width, height, i;

    Mutex::Autolock lock(mPreviewBufsLock);

    if(!mPreviewing) {
        return NO_INIT;
    }

// rt5604 -> to reduce KPI 2012.07.09
	if (!skip_stream_on)
// rt5604 <- to reduce KPI 2012.07.09
    if (mVideoInfo->isStreaming) {
        bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        ret = ioctl (mCameraHandle, VIDIOC_STREAMOFF, &bufType);
        if (ret < 0) {
            CAMHAL_ALOGEB("StopStreaming: Unable to stop capture: %s", strerror(errno));
            return ret;
        }
        CAMHAL_ALOGDA("VIDIOC_STREAMOFF");
        mVideoInfo->isStreaming = false;
    }

    mPreviewing = false;

    mParams.getPreviewSize(&width, &height);

#ifdef MMS_COMBINATION_BUG_FIX
    // CTS HACK for stopping preview after capture.
    // This is needed in order the preview resolution to be changed after
    // capture has finished and before startPreview() has been invoked.
    if (1) { //check) {
#else
	if (check) {
#endif
	    for (i = 0; i < 6; i++) {
	        if (munmap(mIonHandle.keyAt(i), (width*height*2)) < 0 )
	            ALOGE("ION Unmap failed");
	        close(mmap_fd[i]);
	    }

	    if (ion_fd >= 0)
	    {
	        ion_close(ion_fd);
	    }
	    mPreviewBufs.clear();
	    mIonHandle.clear();
	}

    return ret;
}

// 20120628 jungyeal@lge.com sub cam rotation patch from mms
// 20121018 daewon1004.kim@lge.com change to int form boolean
char* V4LCameraAdapter::GetFrame(int &index, int rotateVideo)
{
    int ret;
// 20120628 jungyeal@lge.com sub cam rotation patch from mms
    int w, h;

    mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->buf.memory = V4L2_MEMORY_USERPTR;

    /* DQ */
    ret = ioctl(mCameraHandle, VIDIOC_DQBUF, &mVideoInfo->buf);
    if (ret < 0) {
        CAMHAL_ALOGEB("GetFrame: VIDIOC_DQBUF Failed ret = %d; errno=%d (%s)", ret, errno, strerror(errno));
        return NULL;
    }
     nDequeued++;

    index = mVideoInfo->buf.index;
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
   //ALOGI("kim vt  1 V4LCameraAdapter::stopPreview rotateVideo:%d", rotateVideo);
    if ( rotateVideo && mVideoInfo->rotation_buf) {
        mParams.getPreviewSize(&w, &h);
//        CAMHAL_ALOGDB("rotate 0x%x to 0x%x at %dx%d", mVideoInfo->buf.m.userptr, mVideoInfo->rotation_buf, h, w);
        if(rotateVideo ==ROTATION_90)
        {
	        YUV422I_rotate((uint8_t *) mVideoInfo->buf.m.userptr, (uint8_t *) mVideoInfo->rotation_buf, h, w);
        }
	else
	{
	        YUV422I_rotate_270((uint8_t *) mVideoInfo->buf.m.userptr, (uint8_t *) mVideoInfo->rotation_buf, h, w);
	}
        memcpy( (void *) mVideoInfo->buf.m.userptr, (void *) mVideoInfo->rotation_buf, h * w * 2);
        //ALOGI("kim vt  2 V4LCameraAdapter::stopPreview");
    }
#endif
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */

    return (char *)mIonHandle.keyAt(index);
}

//API to get the frame size required to be allocated. This size is used to override the size passed
//by camera service when VSTAB/VNF is turned ON for example
status_t V4LCameraAdapter::getFrameSize(size_t &width, size_t &height)
{
    status_t ret = NO_ERROR;

    // Just return the current preview size, nothing more to do here.
    mParams.getPreviewSize(( int * ) &width,
                           ( int * ) &height);

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

status_t V4LCameraAdapter::getFrameDataSize(size_t &dataFrameSize, size_t bufferCount)
{
    // We don't support meta data, so simply return
    return NO_ERROR;
}

status_t V4LCameraAdapter::getPictureBufferSize(size_t &length, size_t bufferCount)
{
    int ret = NO_ERROR;
    int image_width , image_height ;
    int preview_width, preview_height;
    v4l2_streamparm parm;
    
    mParams.getPictureSize(&image_width, &image_height);
    mParams.getPreviewSize(&preview_width, &preview_height);

    CAMHAL_ALOGEB("Picture Size: Width = %d \tHeight = %d", image_width, image_height);

    mVideoInfo->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->format.fmt.pix.width = image_width;
    mVideoInfo->format.fmt.pix.height = image_height ;
    mVideoInfo->format.fmt.pix.pixelformat = DEFAULT_PIXEL_FORMAT;

    ret = ioctl(mCameraHandle, VIDIOC_S_FMT, &mVideoInfo->format);
    if (ret < 0) {
        CAMHAL_ALOGEB("Open: VIDIOC_S_FMT Failed: %s", strerror(errno));
        return ret;
    }

    //Set 10 fps for 8MP case
	if( ( image_height == CAPTURE_8MP_HEIGHT ) && ( image_width == CAPTURE_8MP_WIDTH ) ) {
		ALOGE("8MP Capture setting framerate to 10");
		parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ret = ioctl(mCameraHandle, VIDIOC_G_PARM, &parm);
		if(ret != 0) {
			ALOGE("VIDIOC_G_PARM ");
			return -1;
		}

		parm.parm.capture.timeperframe.numerator = 1;
		parm.parm.capture.timeperframe.denominator = 10;
		ret = ioctl(mCameraHandle, VIDIOC_S_PARM, &parm);
		if(ret != 0) {
			ALOGE("VIDIOC_S_PARM ");
			return -1;
		}
	}

	/* Check if the camera driver can accept 1 buffer */
    mVideoInfo->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->rb.memory = V4L2_MEMORY_USERPTR;
    mVideoInfo->rb.count = 1;

	if (ioctl(mCameraHandle, VIDIOC_REQBUFS,  &mVideoInfo->rb) < 0){
		ALOGE ("VIDIOC_REQBUFS Failed. errno = %d", errno);
		return -1;
	}

	mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	mVideoInfo->buf.memory =  V4L2_MEMORY_USERPTR;
	mVideoInfo->buf.index = 0;

	if (ioctl(mCameraHandle, VIDIOC_QUERYBUF, &mVideoInfo->buf) < 0) {
		ALOGE("VIDIOC_QUERYBUF Failed");
		return -1;
	}

	length = mVideoInfo->buf.length ;

    return NO_ERROR;
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
        mFps = ((mFrameCount - mLastFrameCount) * float(s2ns(1))) / diff;
        mLastFpsTime = now;
        mLastFrameCount = mFrameCount;
        ALOGD("Camera %d Frames, %f FPS", mFrameCount, mFps);
    }
    // XXX: mFPS has the value we want
}

status_t V4LCameraAdapter::recalculateFPS()
{
    float currentFPS;

    mFrameCount++;

    if ( ( mFrameCount % FPS_PERIOD ) == 0 )
        {
        nsecs_t now = systemTime();
        nsecs_t diff = now - mLastFPSTime;
        currentFPS =  ((mFrameCount - mLastFrameCount) * float(s2ns(1))) / diff;
        mLastFPSTime = now;
        mLastFrameCount = mFrameCount;

        if ( 1 == mIter )
            {
            mFPS = currentFPS;
            }
        else
            {
            //cumulative moving average
            mFPS = mLastFPS + (currentFPS - mLastFPS)/mIter;
            }

        mLastFPS = mFPS;
        mIter++;
        }

    return NO_ERROR;
}

void V4LCameraAdapter::onOrientationEvent(uint32_t orientation, uint32_t tilt)
{
    LOG_FUNCTION_NAME;

    LOG_FUNCTION_NAME_EXIT;
}


V4LCameraAdapter::V4LCameraAdapter(size_t sensor_index)
{
    LOG_FUNCTION_NAME;
    // Nothing useful to do in the constructor

    LOG_FUNCTION_NAME_EXIT;
}

V4LCameraAdapter::~V4LCameraAdapter()
{
    LOG_FUNCTION_NAME;

    // Close the camera handle and free the video info structure
    //close(mCameraHandle);

    mVideoInfo->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mVideoInfo->buf.memory = V4L2_MEMORY_USERPTR;

    nQueued = 0;
    nDequeued = 0;

    mPreviewBufs.clear();
    mIonHandle.clear();

    if (mVideoInfo)
    {
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE)
        //ALOGI("kim vt 1 V4LCameraAdapter::~V4LCameraAdapter");
        if (mVideoInfo->rotation_buf)
        {
            free(mVideoInfo->rotation_buf);
            mVideoInfo->rotation_buf = NULL;
            //ALOGI("kim vt 2 V4LCameraAdapter::~V4LCameraAdapter");
        }
#endif
/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [END] */
        free(mVideoInfo);
        mVideoInfo = NULL;
    }

    LOG_FUNCTION_NAME_EXIT;
}

int V4LCameraAdapter::queueToGralloc(int index, char* fp, int frameType)
{
    status_t ret = NO_ERROR;
    int width, height;
    CameraFrame frame;
    int i;
    VideoInfo* buf;
    uint8_t* grallocPtr;

// ymjun temp : If we use "#ifndef ICAP", main camera's zoom setting in preview does not keep after taking photo.
#if 1//ndef ICAP 	// TI Patch (I067601f1) : add #ifndef ICAP					
    if(!fp )
    	return BAD_VALUE;
#endif         


    mParams.getPreviewSize(&width, &height);

    //Get index corresponding to key, to fetch correct Gralloc Ptr.
    for ( i = 0; i < mPreviewBufs.size(); i++) {
            if(mPreviewBufs.valueAt(i) == index) {
                grallocPtr = (uint8_t*) mPreviewBufs.keyAt(i);
               break;
        }
    }
    
/* CSR-OMAPS00277259 Sasken - Fix for Non-720p recorded videos have low fps [START] */    
	//recalculateFPS();
/* CSR-OMAPS00277259 Sasken - Fix for Non-720p recorded videos have low fps [START] */

    Mutex::Autolock lock(mSubscriberLock);

    if ( CameraFrame::IMAGE_FRAME == frameType ) {
	      int image_width , image_height ;
	      mParams.getPictureSize(&image_width, &image_height);

		  frame.mFrameType = CameraFrame::SNAPSHOT_FRAME;
		  frame.mQuirks |= CameraFrame::ENCODE_RAW_YUV422I_TO_JPEG;
		  frame.mFrameMask = CameraFrame::IMAGE_FRAME;
// ymjun temp : If we use "#ifdef ICAP", main camera's zoom setting in preview does not keep after taking photo.		  
#if 0//def ICAP							// TI Patch (I067601f1)	: add #ifdef ICAP  
          frame.mBuffer = grallocPtr;
#else		
          frame.mBuffer = fp;
#endif
          frame.mLength = image_width * image_height * 2;
	      frame.mAlignment = image_width * 2;
	      frame.mWidth = image_width;
	      frame.mHeight = image_height;
	}
    else if ( CameraFrame::VIDEO_FRAME_SYNC == frameType ) {
        frame.mFrameType = CameraFrame::VIDEO_FRAME_SYNC;
        frame.mFrameMask = CameraFrame::VIDEO_FRAME_SYNC;
        frame.mBuffer = grallocPtr;
        frame.mLength = width*height*2;
        frame.mAlignment = width*2;
        frame.mWidth = width;
        frame.mHeight = height;
		
#if defined(LGE_JUSTIN_DEVICE)
/* CSR-OMAPS00277259 syam.reddy@sasken.com - Fix for JUSTIN Sub camera thumbnail broken image issue  [START] */
		if(width == 1280 && height == 720)
			mHDRecording = true;
/* CSR-OMAPS00277259 syam.reddy@sasken.com - Fix for JUSTIN Sub camera thumbnail broken image issue  [END] */
#else
/* CSR-OMAPS00277259 Sasken - Fix for Non-720p recorded videos have low fps [START] */
        if(width != 1280 && height != 720)
        	mRecording = false;
/* CSR-OMAPS00277259 Sasken - Fix for Non-720p recorded videos have low fps [End] */
#endif
    }
    else {
		frame.mFrameType = CameraFrame::PREVIEW_FRAME_SYNC;
		frame.mFrameMask = CameraFrame::PREVIEW_FRAME_SYNC;
		frame.mBuffer = grallocPtr;
		frame.mLength = width*height*2;
	    frame.mAlignment = width*2;
	    frame.mWidth = width;
	    frame.mHeight = height;
	}

	frame.mOffset = 0;
	frame.mYuv[0] = NULL;
	frame.mYuv[1] = NULL;
	frame.mTimestamp = systemTime(SYSTEM_TIME_MONOTONIC);

	ret = setInitFrameRefCount(frame.mBuffer, frame.mFrameMask);
	ret = sendFrameToSubscribers(&frame);

    return ret;
}

extern "C" CameraAdapter* CameraAdapter_Factory(size_t sensor_index)
{
    CameraAdapter *adapter = NULL;
    Mutex::Autolock lock(gAdapterLock);

    LOG_FUNCTION_NAME;

    adapter = new V4LCameraAdapter(sensor_index);
    if ( adapter ) {
        CAMHAL_ALOGDB("New OMX Camera adapter instance created for sensor %d",sensor_index);
    } else {
        CAMHAL_ALOGEA("Camera adapter create failed!");
    }

    LOG_FUNCTION_NAME_EXIT;

    return adapter;
}

extern "C" int CameraAdapter_Capabilities(CameraProperties::Properties* properties_array,
                                          const unsigned int starting_camera,
                                          const unsigned int max_camera) {
    int num_cameras_supported = 0;
    CameraProperties::Properties* properties = NULL;

    LOG_FUNCTION_NAME;

    if(!properties_array)
    {
        return -EINVAL;
    }

    // TODO: Need to tell camera properties what other cameras we can support
    if (starting_camera + num_cameras_supported < max_camera) {
        num_cameras_supported++;
        properties = properties_array + starting_camera;
        properties->set(CameraProperties::CAMERA_NAME, "USBCamera");

    }

    LOG_FUNCTION_NAME_EXIT;

    return num_cameras_supported;
}

/* 20120628 jungyeal@lge.com sub cam rotation patch from mms [START] */
// 20121003 daewon1004.kim@lge.com Front camera display -90 on the vt call (blackg_open_ics)
#if defined(LGE_JUSTIN_DEVICE) || defined(BLACKG_OPEN_COM_DEVICE) 
int YUV422I_rotate(const uint8_t *src, uint8_t *dst, int src_w, int src_h)
{
#if 1   // improve performance
    register int i, j;
    register const uint8_t *a;
    register uint8_t *b;
    const uint8_t *re_src;
    uint8_t *re_dst;
    int stride, stride2, stride_out, stride_out2;

#ifdef DEBUG_LOG

    struct timeval conv_before;

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

    gettimeofday(&conv_before, 0);

#endif
#endif

   //ALOGI("kim vt  YUV422I_rotate src_w:%d, src_w:%d",src_w, src_h);

    stride = src_w * 2;
    stride2 = src_w * 4;
    stride_out = src_h * 2;
    stride_out2 = src_h * 4;

    re_src = src + 1;
    re_dst = dst + stride_out - 1; // dst + 1 + (2 * (src_h - 1))

    j = 0;
    do {
        a = re_src;
        b = re_dst;

        i = 0;
        do {
            // Y plane
            *b = *a;
            *(b + stride_out) = *(a + 2);
            *(b - 2)= *(a + stride);
            *(b + stride_out - 2) = *(a + stride + 2);
            // U plane
            *(b - 3 + stride_out) = *(b - 3) = *(a - 1);
            // V plane
            *(b - 1 + stride_out) = *(b - 1) = *(a + 1);

            a += 4;
            b += stride_out2;

            i += 2;
        } while ( i != src_w );
        re_src += stride2;
        re_dst -= 4;

        j+=2;
    }while (j < src_h);
#else
    const uint8_t *a;
    uint8_t *b;
    uint8_t *c;
    int i, j, stride, stride_out, stride_out2;

#ifdef DEBUG_LOG

    struct timeval conv_before;

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

    gettimeofday(&conv_before, 0);

#endif
#endif

    memset(dst, 0x80, src_w * src_h * 2);

    stride = src_w * 2;
    stride_out = src_h * 2;
    stride_out2 = src_h * 4;

    // Convert Y plane
    for (j = 0; j < src_h; j ++) {
        a = src + 1 + (stride * j);
        b = dst + 1 + (2 * (src_h - 1 - j));
        for (i = 0; i != src_w; i ++) {
            *b = *a;
            a += 2;
            b += stride_out;
        }
    }

    // Convert U plane
    for (j = 0; j < src_h; j += 2) {
        a = src + 0 + (stride * j);
        b = dst + 0 + (4 * (((src_h - j) / 2) -1));
        c = b + stride_out;
        for (i = 0; i != src_w; i +=2 ) {
            *c = *b = *a;
            a += 4;
            b += stride_out2;
            c += stride_out2;
        }
    }

    // Convert V plane
    for (j = 0; j < src_h; j += 2) {
        a = src + 2 + (stride * j);
        b = dst + 2 + (4 * (((src_h - j) / 2) -1));
        c = b + stride_out;
        for (i = 0; i != src_w; i +=2 ) {
            *c = *b = *a;
            a += 4;
            b += stride_out2;
            c += stride_out2;
        }
    }
#endif

    return 0;
}
/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [START] */

int YUV422I_rotate_270(const uint8_t *src, uint8_t *dst, int src_w, int src_h)
{
#if 1   // improve performance
    register int i, j;
    register const uint8_t *a;
    register uint8_t *b;
    const uint8_t *re_src;
    uint8_t *re_dst;
    int stride, stride2, stride_out, stride_out2;

#ifdef DEBUG_LOG

    struct timeval conv_before;

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

    gettimeofday(&conv_before, 0);

#endif
#endif

   //ALOGI("kim vt  YUV422I_rotate_270 src_w:%d, src_w:%d",src_w, src_h);

    stride = src_w * 2;
    stride2 = src_w * 4;
    stride_out = src_h * 2;
    stride_out2 = src_h * 4;

    re_src = src + (stride - 2) + 1;
    re_dst = dst + 1; // dst + 1 + (2 * (src_h - 1))

    j = 0;
    do {
        a = re_src;
        b = re_dst;

        i = 0;
        do {
            // Y plane
            *b = *a;
            *(b + stride_out) = *(a - 2);
            *(b + 2)= *(a + stride);
            *(b + stride_out + 2) = *(a + stride - 2);
            // U plane
            *(b - 1 + stride_out) = *(b - 1) = *(a - 3);
            // V plane
            *(b + 1 + stride_out) = *(b + 1) = *(a - 1);

            a -= 4;
            b += stride_out2;

            i += 2;
        } while ( i != src_w );
        re_src += stride2;
        re_dst += 4;

        j+=2;
    }while (j < src_h);
#else
    const uint8_t *a;
    uint8_t *b;
    uint8_t *c;
    int i, j, stride, stride_out, stride_out2;

#ifdef DEBUG_LOG

    struct timeval conv_before;

#if PPM_INSTRUMENTATION || PPM_INSTRUMENTATION_ABS

    gettimeofday(&conv_before, 0);

#endif
#endif

//    memset(dst, 0x80, src_w * src_h * 2);

    stride = src_w * 2;
    stride_out = src_h * 2;
    stride_out2 = src_h * 4;

    // Convert Y plane
    for (j = 0; j < src_h; j ++) {
        a = src + (stride - 2) + 1 + (stride * j);
        b = dst + 1 + 2 * j;
        for (i = 0; i != src_w; i ++) {
            *b = *a;
            a -= 2;
            b += stride_out;
        }
    }

    // Convert U plane
    for (j = 0; j < src_h; j += 2) {
        a = src + (stride - 2) + 0 + (stride * j);
        b = dst + 0 + (4 * j);
        c = b + stride_out;
        for (i = 0; i != src_w; i +=2 ) {
            *c = *b = *a;
            a -= 4;
            b += stride_out2;
            c += stride_out2;
        }
    }

    // Convert V plane
    for (j = 0; j < src_h; j += 2) {
        a = src + (stride - 2) + 2 + (stride * j);
        b = dst + 2 + (4 * j);
        c = b + stride_out;
        for (i = 0; i != src_w; i +=2 ) {
            *c = *b = *a;
            a -= 4;
            b += stride_out2;
            c += stride_out2;
        }
    }
#endif

    return 0;
}
#endif
/* CSR-OMAPS00273421 kirti.badkundri@sasken.com fix for sub cam rotation [START] */
};


/*--------------------Camera Adapter Class ENDS here-----------------------------*/

