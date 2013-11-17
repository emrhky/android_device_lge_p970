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




#define LOG_TAG "AppCallbackNotifier"


#include "CameraHal.h"
#include "VideoMetadata.h"
#include "Encoder_libjpeg.h"
#include <MetadataBufferType.h>
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferMapper.h>
#include "NV12_resize.h"

namespace android {

const int AppCallbackNotifier::NOTIFIER_TIMEOUT = -1;
KeyedVector<void*, sp<Encoder_libjpeg> > gEncoderQueue;

// rt5604 2012.08.13 BUG Fix for CTS testPreviewCallback ->
#define DVD_736     736
#define DVD_720     720
// rt5604 2012.08.13 BUG Fix for CTS testPreviewCallback <-
void AppCallbackNotifierEncoderCallback(void* main_jpeg,
                                        void* thumb_jpeg,
                                        CameraFrame::FrameType type,
                                        void* cookie1,
                                        void* cookie2,
                                        void* cookie3)
{
    if (cookie1) {
        AppCallbackNotifier* cb = (AppCallbackNotifier*) cookie1;
        cb->EncoderDoneCb(main_jpeg, thumb_jpeg, type, cookie2, cookie3);
    }
}

/*--------------------NotificationHandler Class STARTS here-----------------------------*/

void AppCallbackNotifier::EncoderDoneCb(void* main_jpeg, void* thumb_jpeg, CameraFrame::FrameType type, void* cookie1, void* cookie2)
{
    camera_memory_t* encoded_mem = NULL;
    Encoder_libjpeg::params *main_param = NULL, *thumb_param = NULL;
    size_t jpeg_size;
    uint8_t* src = NULL;
    sp<Encoder_libjpeg> encoder = NULL;

    LOG_FUNCTION_NAME;

    camera_memory_t* picture = NULL;

    {
    Mutex::Autolock lock(mLock);

    if (!main_jpeg) {
        goto exit;
    }

    encoded_mem = (camera_memory_t*) cookie1;
    main_param = (Encoder_libjpeg::params *) main_jpeg;
    jpeg_size = main_param->jpeg_size;
    src = main_param->src;

    if(encoded_mem && encoded_mem->data && (jpeg_size > 0)) {
        if (cookie2) {
            ExifElementsTable* exif = (ExifElementsTable*) cookie2;
            Section_t* exif_section = NULL;

            exif->insertExifToJpeg((unsigned char*) encoded_mem->data, jpeg_size);

            if(thumb_jpeg) {
                thumb_param = (Encoder_libjpeg::params *) thumb_jpeg;
                exif->insertExifThumbnailImage((const char*)thumb_param->dst,
                                               (int)thumb_param->jpeg_size);
            }

            exif_section = FindSection(M_EXIF);

            if (exif_section) {
                picture = mRequestMemory(-1, jpeg_size + exif_section->Size, 1, NULL);
                if (picture && picture->data) {
                    exif->saveJpeg((unsigned char*) picture->data, jpeg_size + exif_section->Size);
                }
            }
            delete exif;
            cookie2 = NULL;
        } else {
            picture = mRequestMemory(-1, jpeg_size, 1, NULL);
            if (picture && picture->data) {
                memcpy(picture->data, encoded_mem->data, jpeg_size);
            }
        }
    }
    } // scope for mutex lock

    if (!mRawAvailable) {
        dummyRaw();
    } else {
        mRawAvailable = false;
    }

    // Send the callback to the application only if the notifier is started and the message is enabled
    if(picture && (mNotifierState==AppCallbackNotifier::NOTIFIER_STARTED) &&
                  (mCameraHal->msgTypeEnabled(CAMERA_MSG_COMPRESSED_IMAGE)))
    {
        Mutex::Autolock lock(mBurstLock);
        {
            mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, picture, 0, NULL, mCallbackCookie);
        }
    }

 exit:

    if (main_jpeg) {
        free(main_jpeg);
    }

    if (thumb_jpeg) {
       if (((Encoder_libjpeg::params *) thumb_jpeg)->dst) {
           free(((Encoder_libjpeg::params *) thumb_jpeg)->dst);
       }
       free(thumb_jpeg);
    }

    if (encoded_mem) {
        encoded_mem->release(encoded_mem);
    }

    if (picture) {
        picture->release(picture);
    }

    if (cookie2) {
        delete (ExifElementsTable*) cookie2;
    }

    if (mNotifierState == AppCallbackNotifier::NOTIFIER_STARTED) {
        encoder = gEncoderQueue.valueFor(src);
        if (encoder.get()) {
            gEncoderQueue.removeItem(src);
            encoder.clear();
        }
        mFrameProvider->returnFrame(src, type);
    }

    LOG_FUNCTION_NAME_EXIT;
}

/**
  * NotificationHandler class
  */

///Initialization function for AppCallbackNotifier
status_t AppCallbackNotifier::initialize()
{
    LOG_FUNCTION_NAME;

    mMeasurementEnabled = false;

    ///Create the app notifier thread
    mNotificationThread = new NotificationThread(this);
    if(!mNotificationThread.get())
        {
        CAMHAL_ALOGEA("Couldn't create Notification thread");
        return NO_MEMORY;
        }

    ///Start the display thread
    status_t ret = mNotificationThread->run("NotificationThread", PRIORITY_URGENT_DISPLAY);
    if(ret!=NO_ERROR)
        {
        CAMHAL_ALOGEA("Couldn't run NotificationThread");
        mNotificationThread.clear();
        return ret;
        }

    mUseMetaDataBufferMode = true;
    mRawAvailable = false;

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

void AppCallbackNotifier::setCallbacks(CameraHal* cameraHal,
                                        camera_notify_callback notify_cb,
                                        camera_data_callback data_cb,
                                        camera_data_timestamp_callback data_cb_timestamp,
                                        camera_request_memory get_memory,
                                        void *user)
{
    Mutex::Autolock lock(mLock);

    LOG_FUNCTION_NAME;

    mCameraHal = cameraHal;
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mRequestMemory = get_memory;
    mCallbackCookie = user;

    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::setMeasurements(bool enable)
{
    Mutex::Autolock lock(mLock);

    LOG_FUNCTION_NAME;

    mMeasurementEnabled = enable;

    if (  enable  )
    {
        mFrameProvider->enableFrameNotification(CameraFrame::FRAME_DATA_SYNC);
    }

    LOG_FUNCTION_NAME_EXIT;
}


//All sub-components of Camera HAL call this whenever any error happens
void AppCallbackNotifier::errorNotify(int error)
{
    LOG_FUNCTION_NAME;

    CAMHAL_ALOGEB("AppCallbackNotifier received error %d", error);

    // If it is a fatal error abort here!
    if((error == CAMERA_ERROR_FATAL) || (error == CAMERA_ERROR_HARD)) {
        //We kill media server if we encounter these errors as there is
        //no point continuing and apps also don't handle errors other
        //than media server death always.
        abort();
        return;
    }

    if (  ( NULL != mCameraHal ) &&
          ( NULL != mNotifyCb ) &&
          ( mCameraHal->msgTypeEnabled(CAMERA_MSG_ERROR) ) )
      {
        CAMHAL_ALOGEB("AppCallbackNotifier mNotifyCb %d", error);
        mNotifyCb(CAMERA_MSG_ERROR, CAMERA_ERROR_UNKNOWN, 0, mCallbackCookie);
      }

    LOG_FUNCTION_NAME_EXIT;
}

bool AppCallbackNotifier::notificationThread()
{
    bool shouldLive = true;
    status_t ret;

    LOG_FUNCTION_NAME;

    //CAMHAL_ALOGDA("Notification Thread waiting for message");
    ret = TIUTILS::MessageQueue::waitForMsg(&mNotificationThread->msgQ(),
                                            &mEventQ,
                                            &mFrameQ,
                                            AppCallbackNotifier::NOTIFIER_TIMEOUT);

    //CAMHAL_ALOGDA("Notification Thread received message");

    if (mNotificationThread->msgQ().hasMsg()) {
        ///Received a message from CameraHal, process it
        CAMHAL_ALOGDA("Notification Thread received message from Camera HAL");
        shouldLive = processMessage();
        if(!shouldLive) {
                CAMHAL_ALOGDA("Notification Thread exiting.");
        }
    }

    if(mEventQ.hasMsg()) {
        ///Received an event from one of the event providers
        CAMHAL_ALOGDA("Notification Thread received an event from event provider (CameraAdapter)");
        notifyEvent();
     }

    if(mFrameQ.hasMsg()) {
       ///Received a frame from one of the frame providers
       //CAMHAL_ALOGDA("Notification Thread received a frame from frame provider (CameraAdapter)");
       notifyFrame();
    }

    LOG_FUNCTION_NAME_EXIT;
    return shouldLive;
}

void AppCallbackNotifier::notifyEvent()
{
    ///Receive and send the event notifications to app
    TIUTILS::Message msg;
    LOG_FUNCTION_NAME;
    mEventQ.get(&msg);
    bool ret = true;
    CameraHalEvent *evt = NULL;
    CameraHalEvent::FocusEventData *focusEvtData;
    CameraHalEvent::ZoomEventData *zoomEvtData;
    CameraHalEvent::FaceEventData faceEvtData;

    if(mNotifierState != AppCallbackNotifier::NOTIFIER_STARTED)
    {
        return;
    }

    switch(msg.command)
        {
        case AppCallbackNotifier::NOTIFIER_CMD_PROCESS_EVENT:

            evt = ( CameraHalEvent * ) msg.arg1;

            if ( NULL == evt )
                {
                CAMHAL_ALOGEA("Invalid CameraHalEvent");
                return;
                }

            switch(evt->mEventType)
                {
                case CameraHalEvent::EVENT_SHUTTER:

                    if ( ( NULL != mCameraHal ) &&
                          ( NULL != mNotifyCb ) &&
                          ( mCameraHal->msgTypeEnabled(CAMERA_MSG_SHUTTER) ) )
                        {
                            mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);
                        }
                    mRawAvailable = false;

                    break;

                case CameraHalEvent::EVENT_FOCUS_LOCKED:
                case CameraHalEvent::EVENT_FOCUS_ERROR:

                    focusEvtData = &evt->mEventData->focusEvent;
                    if ( ( focusEvtData->focusLocked ) &&
                          ( NULL != mCameraHal ) &&
                          ( NULL != mNotifyCb ) &&
                          ( mCameraHal->msgTypeEnabled(CAMERA_MSG_FOCUS) ) )
                        {
                         mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
                         mCameraHal->disableMsgType(CAMERA_MSG_FOCUS);
                        }
                    else if ( focusEvtData->focusError &&
                                ( NULL != mCameraHal ) &&
                                ( NULL != mNotifyCb ) &&
                                ( mCameraHal->msgTypeEnabled(CAMERA_MSG_FOCUS) ) )
                        {
                         mNotifyCb(CAMERA_MSG_FOCUS, false, 0, mCallbackCookie);
                         mCameraHal->disableMsgType(CAMERA_MSG_FOCUS);
                        }

                    break;

                case CameraHalEvent::EVENT_ZOOM_INDEX_REACHED:

                    zoomEvtData = &evt->mEventData->zoomEvent;

                    if ( ( NULL != mCameraHal ) &&
                         ( NULL != mNotifyCb) &&
                         ( mCameraHal->msgTypeEnabled(CAMERA_MSG_ZOOM) ) )
                        {
                        mNotifyCb(CAMERA_MSG_ZOOM, zoomEvtData->currentZoomIndex, zoomEvtData->targetZoomIndexReached, mCallbackCookie);
                        }

                    break;

                case CameraHalEvent::EVENT_FACE:

                    faceEvtData = evt->mEventData->faceEvent;

                    if ( ( NULL != mCameraHal ) &&
                         ( NULL != mNotifyCb) &&
                         ( mCameraHal->msgTypeEnabled(CAMERA_MSG_PREVIEW_METADATA) ) )
                        {
                        // WA for an issue inside CameraService
                        camera_memory_t *tmpBuffer = mRequestMemory(-1, 1, 1, NULL);

                        mDataCb(CAMERA_MSG_PREVIEW_METADATA,
                                tmpBuffer,
                                0,
                                faceEvtData->getFaceResult(),
                                mCallbackCookie);

                        faceEvtData.clear();

                        if ( NULL != tmpBuffer ) {
                            tmpBuffer->release(tmpBuffer);
                        }

                        }

                    break;

                case CameraHalEvent::ALL_EVENTS:
                    break;
                default:
                    break;
                }

            break;
        }

    if ( NULL != evt )
        {
        delete evt;
        }


    LOG_FUNCTION_NAME_EXIT;

}

//--[[ LGE_UBIQUIX_MODIFIED_START : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot
#if	0
int YUV422I_to_YUV420SP(const uint8_t *src, uint8_t *dst, int w, int h)
{
    const uint8_t *a;
    uint8_t *b;
    int i, j, total_steps, stride;

    // Convert Y plane
    stride = w * 2;
    a = src + 1;
    b = dst;
    total_steps = w * h;

    for (i = 0; i < total_steps; i++) {
        *b = *a;
        a += 2;
        b++;
    }

    // Convert U plane
    stride = w * 2;
    a = src + 2;
    b = dst + (w * h);
    for (j = 0; j < h; j += 2) {
        for (i = 0; i < stride; i += 4) {
            *b = *a;
            a += 4;
            b += 2;
        }
        a += stride;
    }

    // Convert V plane
    stride = w * 2;
    a = src;
    b = dst + (w * h) + 1;
    for (j = 0; j < h; j += 2) {
        for (i = 0; i < stride; i += 4) {
            *b = *a;
            a += 4;
            b += 2;
        }
        a += stride;
    }

    return 0;
}
#endif

#include <yuv422yuv420.h>

int YUV422I_to_YUV420SP(const uint8_t *src, uint8_t *dst, int w, int h)
{
    YUV422_BUF_t i_buf;
    YUV420_BUF_t o_buf;
    YUVWINCONV_PARAMS_t win;
    YUV422YUV420_CONV_STATUS_t conv_res;

    // input buffer
    i_buf.buf = (uint32_t *)src;
    i_buf.w = w;
    i_buf.h = h;
    i_buf.bytes = w * h * 2;
    i_buf.fmt = YUV422_FORMAT_UYVY;

    // output buffer
    o_buf.buf = (uint16_t *) dst;
    o_buf.w = w;
    o_buf.h = h;
    o_buf.bytes = w * h * 3 / 2;
    o_buf.fmt = YUV420_FORMAT_NV21;

    // window
    win.d.x = 0;
    win.d.y = 0;
    win.s.x = 0;
    win.s.y = 0;
    win.w = w;
    win.h = h;

    conv_res = yuv422yuv420_neon (&o_buf, &i_buf, &win);

    switch (conv_res) {
        case YUV422YUV420_CONV_STATUS_OK:
//--rt5604            ALOGD("Conversion OK");
            break;
        case YUV422YUV420_CONV_STATUS_ERROR:
            ALOGE("Conversion ERROR");
            memset( (void *) dst, 0x80, w * h * 3 / 2);
            break;
    }

    return 0;
}
//--]] LGE_UBIQUIX_MODIFIED_END : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot

/* 20120614 jungyeal@lge.com Workaround 192x144 -> 176x144 Crop  for VT [START]*/ 
// rt5604 2012.08.13 BUG Fix for CTS testPreviewCallback
unsigned char my_buf[720*576*2];	 	// 640*480
unsigned char *bufferTemp;
/* 20120614 jungyeal@lge.com Workaround 192x144 -> 176x144 Crop  for VT [END]*/ 
static void copy2Dto1D(void *dst,
                       void *src,
                       int width,
                       int height,
                       size_t stride,
                       uint32_t offset,
                       unsigned int bytesPerPixel,
                       size_t length,
                       const char *pixelFormat)
{
    unsigned int alignedRow, row;
    unsigned char *bufferDst, *bufferSrc;
    unsigned char *bufferDstEnd, *bufferSrcEnd;
    uint16_t *bufferSrc_UV;

    unsigned int *y_uv = (unsigned int *)src;

//    CAMHAL_ALOGVB("copy2Dto1D() y= %p ; uv=%p.",y_uv[0], y_uv[1]);
//    CAMHAL_ALOGVB("pixelFormat,= %d; offset=%d",*pixelFormat,offset);

    if (pixelFormat!=NULL) {
        if (strcmp(pixelFormat, CameraParameters::PIXEL_FORMAT_YUV422I) == 0) {
            bytesPerPixel = 2;
        } else if (strcmp(pixelFormat, CameraParameters::PIXEL_FORMAT_YUV420SP) == 0 ||
                   strcmp(pixelFormat, CameraParameters::PIXEL_FORMAT_YUV420P) == 0) {

//--[[ LGE_UBIQUIX_MODIFIED_START : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot
//			ALOGD("rt5604: copy2Dto1D: pixelFormat=%s!!!", pixelFormat);
#if	1
			bytesPerPixel = 2;
#else
            bytesPerPixel = 1;
            bufferDst = ( unsigned char * ) dst;
            bufferDstEnd = ( unsigned char * ) dst + width*height*bytesPerPixel;
            bufferSrc = ( unsigned char * ) y_uv[0] + offset;
            bufferSrcEnd = ( unsigned char * ) ( ( size_t ) y_uv[0] + length + offset);
            row = width*bytesPerPixel;
            alignedRow = stride-width;
            int stride_bytes = stride / 8;
            uint32_t xOff = offset % stride;
            uint32_t yOff = offset / stride;

            // going to convert from NV12 here and return
            // Step 1: Y plane: iterate through each row and copy
            for ( int i = 0 ; i < height ; i++) {
                memcpy(bufferDst, bufferSrc, row);
                bufferSrc += stride;
                bufferDst += row;
                if ( ( bufferSrc > bufferSrcEnd ) || ( bufferDst > bufferDstEnd ) ) {
                    break;
                }
            }

            bufferSrc_UV = ( uint16_t * ) ((uint8_t*)y_uv[1] + (stride/2)*yOff + xOff);

            if (strcmp(pixelFormat, CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
                 uint16_t *bufferDst_UV;

                // Step 2: UV plane: convert NV12 to NV21 by swapping U & V
                bufferDst_UV = (uint16_t *) (((uint8_t*)dst)+row*height);

                for (int i = 0 ; i < height/2 ; i++, bufferSrc_UV += alignedRow/2) {
                    int n = width;
                    asm volatile (
                    "   pld [%[src], %[src_stride], lsl #2]                         \n\t"
                    "   cmp %[n], #32                                               \n\t"
                    "   blt 1f                                                      \n\t"
                    "0: @ 32 byte swap                                              \n\t"
                    "   sub %[n], %[n], #32                                         \n\t"
                    "   vld2.8  {q0, q1} , [%[src]]!                                \n\t"
                    "   vswp q0, q1                                                 \n\t"
                    "   cmp %[n], #32                                               \n\t"
                    "   vst2.8  {q0,q1},[%[dst]]!                                   \n\t"
                    "   bge 0b                                                      \n\t"
                    "1: @ Is there enough data?                                     \n\t"
                    "   cmp %[n], #16                                               \n\t"
                    "   blt 3f                                                      \n\t"
                    "2: @ 16 byte swap                                              \n\t"
                    "   sub %[n], %[n], #16                                         \n\t"
                    "   vld2.8  {d0, d1} , [%[src]]!                                \n\t"
                    "   vswp d0, d1                                                 \n\t"
                    "   cmp %[n], #16                                               \n\t"
                    "   vst2.8  {d0,d1},[%[dst]]!                                   \n\t"
                    "   bge 2b                                                      \n\t"
                    "3: @ Is there enough data?                                     \n\t"
                    "   cmp %[n], #8                                                \n\t"
                    "   blt 5f                                                      \n\t"
                    "4: @ 8 byte swap                                               \n\t"
                    "   sub %[n], %[n], #8                                          \n\t"
                    "   vld2.8  {d0, d1} , [%[src]]!                                \n\t"
                    "   vswp d0, d1                                                 \n\t"
                    "   cmp %[n], #8                                                \n\t"
                    "   vst2.8  {d0[0],d1[0]},[%[dst]]!                             \n\t"
                    "   bge 4b                                                      \n\t"
                    "5: @ end                                                       \n\t"
#ifdef NEEDS_ARM_ERRATA_754319_754320
                    "   vmov s0,s0  @ add noop for errata item                      \n\t"
#endif
                    : [dst] "+r" (bufferDst_UV), [src] "+r" (bufferSrc_UV), [n] "+r" (n)
                    : [src_stride] "r" (stride_bytes)
                    : "cc", "memory", "q0", "q1"
                    );
                }
            } else if (strcmp(pixelFormat, CameraParameters::PIXEL_FORMAT_YUV420P) == 0) {
                 uint16_t *bufferDst_U;
                 uint16_t *bufferDst_V;

                // Step 2: UV plane: convert NV12 to YV12 by de-interleaving U & V
                // TODO(XXX): This version of CameraHal assumes NV12 format it set at
                //            camera adapter to support YV12. Need to address for
                //            USBCamera

                bufferDst_V = (uint16_t *) (((uint8_t*)dst)+row*height);
                bufferDst_U = (uint16_t *) (((uint8_t*)dst)+row*height+row*height/4);

                for (int i = 0 ; i < height/2 ; i++, bufferSrc_UV += alignedRow/2) {
                    int n = width;
                    asm volatile (
                    "   pld [%[src], %[src_stride], lsl #2]                         \n\t"
                    "   cmp %[n], #32                                               \n\t"
                    "   blt 1f                                                      \n\t"
                    "0: @ 32 byte swap                                              \n\t"
                    "   sub %[n], %[n], #32                                         \n\t"
                    "   vld2.8  {q0, q1} , [%[src]]!                                \n\t"
                    "   cmp %[n], #32                                               \n\t"
                    "   vst1.8  {q1},[%[dst_v]]!                                    \n\t"
                    "   vst1.8  {q0},[%[dst_u]]!                                    \n\t"
                    "   bge 0b                                                      \n\t"
                    "1: @ Is there enough data?                                     \n\t"
                    "   cmp %[n], #16                                               \n\t"
                    "   blt 3f                                                      \n\t"
                    "2: @ 16 byte swap                                              \n\t"
                    "   sub %[n], %[n], #16                                         \n\t"
                    "   vld2.8  {d0, d1} , [%[src]]!                                \n\t"
                    "   cmp %[n], #16                                               \n\t"
                    "   vst1.8  {d1},[%[dst_v]]!                                    \n\t"
                    "   vst1.8  {d0},[%[dst_u]]!                                    \n\t"
                    "   bge 2b                                                      \n\t"
                    "3: @ Is there enough data?                                     \n\t"
                    "   cmp %[n], #8                                                \n\t"
                    "   blt 5f                                                      \n\t"
                    "4: @ 8 byte swap                                               \n\t"
                    "   sub %[n], %[n], #8                                          \n\t"
                    "   vld2.8  {d0, d1} , [%[src]]!                                \n\t"
                    "   cmp %[n], #8                                                \n\t"
                    "   vst1.8  {d1[0]},[%[dst_v]]!                                 \n\t"
                    "   vst1.8  {d0[0]},[%[dst_u]]!                                 \n\t"
                    "   bge 4b                                                      \n\t"
                    "5: @ end                                                       \n\t"
#ifdef NEEDS_ARM_ERRATA_754319_754320
                    "   vmov s0,s0  @ add noop for errata item                      \n\t"
#endif
                    : [dst_u] "+r" (bufferDst_U), [dst_v] "+r" (bufferDst_V),
                      [src] "+r" (bufferSrc_UV), [n] "+r" (n)
                    : [src_stride] "r" (stride_bytes)
                    : "cc", "memory", "q0", "q1"
                    );
                }
            }
            return ;
#endif
//--]] LGE_UBIQUIX_MODIFIED_END : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot
        } else if(strcmp(pixelFormat, CameraParameters::PIXEL_FORMAT_RGB565) == 0) {
            bytesPerPixel = 2;
        }
    }

    bufferDst = ( unsigned char * ) dst;
    bufferSrc = ( unsigned char * ) y_uv[0];
    row = width*bytesPerPixel;
    alignedRow = ( row + ( stride -1 ) ) & ( ~ ( stride -1 ) );

	// TI WA 0425
	alignedRow = row;

//iterate through each row
//--[[ LGE_UBIQUIX_MODIFIED_START : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot
#if	1
/* 20120614 jungyeal@lge.com Workaround 192x144 -> 176x144 Crop  for VT [START] */ 
	if (width == 192) // for only VT
	{
		unsigned int src_offset=0, dest_offset=0;
		width=176;
		row = 176*bytesPerPixel;
		alignedRow = 192*2; 
		
		bufferTemp = ( unsigned char *) my_buf;

		ALOGD("Crop for VT  from 192x144  to 176x144");	
		for ( int i = 0 ; i < height ; i++,  src_offset += alignedRow, dest_offset += row) {
			memcpy( (unsigned char *) (bufferTemp+dest_offset), (unsigned char *) (bufferSrc+src_offset), row);
		} 
		YUV422I_to_YUV420SP(bufferTemp, bufferDst, width, height);
	} 
/* 20120614 jungyeal@lge.com Workaround 192x144 -> 176x144 Crop  for VT [END] */ 	
	else if (width == DVD_736) {		// CTS
		unsigned int src_offset=0, dest_offset=0;
		
		width=DVD_720;
		row = DVD_720*bytesPerPixel;
		alignedRow = DVD_736*2; 

		bufferTemp = ( unsigned char *) my_buf;
		
		ALOGD("Crop for DVD_736  from 736x576  to 720x576");	
		for ( int i = 0 ; i < height ; i++,  src_offset += alignedRow, dest_offset += row) {
			memcpy( (unsigned char *) (bufferTemp+dest_offset), (unsigned char *) (bufferSrc+src_offset), row);
		} 
		YUV422I_to_YUV420SP(bufferTemp, bufferDst, width, height);
	} 
	else {
		YUV422I_to_YUV420SP(bufferSrc, bufferDst, width, height);
	}
#else
    for ( int i = 0 ; i < height ; i++,  bufferSrc += alignedRow, bufferDst += row) {
        memcpy(bufferDst, bufferSrc, row);
    }
#endif
//--]] LGE_UBIQUIX_MODIFIED_END : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot
}

void AppCallbackNotifier::copyAndSendPictureFrame(CameraFrame* frame, int32_t msgType)
{
    camera_memory_t* picture = NULL;
    void *dest = NULL, *src = NULL;

    // scope for lock
    {
        Mutex::Autolock lock(mLock);

        if(mNotifierState != AppCallbackNotifier::NOTIFIER_STARTED) {
            goto exit;
        }

        picture = mRequestMemory(-1, frame->mLength, 1, NULL);

        if (NULL != picture) {
            dest = picture->data;
            if (NULL != dest) {
                src = (void *) ((unsigned int) frame->mBuffer + frame->mOffset);
                memcpy(dest, src, frame->mLength);
            }
        }
    }

 exit:
    mFrameProvider->returnFrame(frame->mBuffer, (CameraFrame::FrameType) frame->mFrameType);

    if(picture) {
        if((mNotifierState == AppCallbackNotifier::NOTIFIER_STARTED) &&
           mCameraHal->msgTypeEnabled(msgType)) {
            mDataCb(msgType, picture, 0, NULL, mCallbackCookie);
        }
        picture->release(picture);
    }
}

void AppCallbackNotifier::copyAndSendPreviewFrame(CameraFrame* frame, int32_t msgType)
{
    camera_memory_t* picture = NULL;
    void* dest = NULL;

    // scope for lock
    {
        Mutex::Autolock lock(mLock);

        if(mNotifierState != AppCallbackNotifier::NOTIFIER_STARTED) {
            goto exit;
        }

        if (!mPreviewMemory || !frame->mBuffer) {
            CAMHAL_ALOGDA("Error! One of the buffer is NULL");
            goto exit;
        }


        dest = (void*) mPreviewBufs[mPreviewBufCount];

        /*CAMHAL_ALOGVB("%d:copy2Dto1D(%p, %p, %d, %d, %d, %d, %d,%s)",
                     __LINE__,
                      buf,
                      frame->mBuffer,
                      frame->mWidth,
                      frame->mHeight,
                      frame->mAlignment,
                      2,
                      frame->mLength,
                      mPreviewPixelFormat);*/

        if ( NULL != dest ) {
            // data sync frames don't need conversion
            if (CameraFrame::FRAME_DATA_SYNC == frame->mFrameType) {
                if ( (mPreviewMemory->size / MAX_BUFFERS) >= frame->mLength ) {
                    memcpy(dest, (void*) frame->mBuffer, frame->mLength);
                } else {
                    memset(dest, 0, (mPreviewMemory->size / MAX_BUFFERS));
                }
            } else {
//--[[ LGE_UBIQUIX_MODIFIED_START : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot (don't modify below code)
#if 1
              if ((NULL == frame->mYuv[0]) || (NULL == frame->mYuv[1])){
                CAMHAL_ALOGEA("Error! One of the YUV Pointer is NULL");
                goto exit;
              }
              else{
                copy2Dto1D(dest,
                           frame->mYuv,
                           frame->mWidth,
                           frame->mHeight,
                           frame->mAlignment,
                           frame->mOffset,
                           2,
                           frame->mLength,
                           mPreviewPixelFormat);
              }
#else
              /*
              if ((NULL == frame->mYuv[0]) || (NULL == frame->mYuv[1])){
                CAMHAL_ALOGEA("Error! One of the YUV Pointer is NULL");
                goto exit;
              }
              else{
                copy2Dto1D(dest,
                           frame->mYuv,
                           frame->mWidth,
                           frame->mHeight,
                           frame->mAlignment,
                           frame->mOffset,
                           2,
                           frame->mLength,
                           mPreviewPixelFormat);
              }
              */
              dest = (void*) frame->mBuffer;
#endif
//--]] LGE_UBIQUIX_MODIFIED_END : rt5604@mnbt.co.kr [2012.05.23] - CAM : beauty/panorama shot
            }
        }
    }

 exit:
    mFrameProvider->returnFrame(frame->mBuffer, (CameraFrame::FrameType) frame->mFrameType);

    if((mNotifierState == AppCallbackNotifier::NOTIFIER_STARTED) &&
       mCameraHal->msgTypeEnabled(msgType) &&
       (dest != NULL)) {
        mDataCb(msgType, mPreviewMemory, mPreviewBufCount, NULL, mCallbackCookie);
    }

    // increment for next buffer
    mPreviewBufCount = (mPreviewBufCount + 1) % AppCallbackNotifier::MAX_BUFFERS;
}

status_t AppCallbackNotifier::dummyRaw()
{
    LOG_FUNCTION_NAME;

    if ( NULL == mRequestMemory ) {
        CAMHAL_ALOGEA("Can't allocate memory for dummy raw callback!");
        return NO_INIT;
    }

    if ( ( NULL != mCameraHal ) &&
         ( NULL != mDataCb) &&
         ( NULL != mNotifyCb ) ){

        if ( mCameraHal->msgTypeEnabled(CAMERA_MSG_RAW_IMAGE) ) {
            camera_memory_t *dummyRaw = mRequestMemory(-1, 1, 1, NULL);

            if ( NULL == dummyRaw ) {
                CAMHAL_ALOGEA("Dummy raw buffer allocation failed!");
                return NO_MEMORY;
            }

            mDataCb(CAMERA_MSG_RAW_IMAGE, dummyRaw, 0, NULL, mCallbackCookie);

            dummyRaw->release(dummyRaw);
        } else if ( mCameraHal->msgTypeEnabled(CAMERA_MSG_RAW_IMAGE_NOTIFY) ) {
            mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, mCallbackCookie);
        }
    }

    LOG_FUNCTION_NAME_EXIT;

    return NO_ERROR;
}

void AppCallbackNotifier::notifyFrame()
{
    ///Receive and send the frame notifications to app
    TIUTILS::Message msg;
    CameraFrame *frame;
    MemoryHeapBase *heap;
    MemoryBase *buffer = NULL;
    sp<MemoryBase> memBase;
    void *buf = NULL;

    LOG_FUNCTION_NAME;

    {
        Mutex::Autolock lock(mLock);
        if(!mFrameQ.isEmpty()) {
            mFrameQ.get(&msg);
        } else {
            return;
        }
    }

    bool ret = true;

    frame = NULL;
    switch(msg.command)
        {
        case AppCallbackNotifier::NOTIFIER_CMD_PROCESS_FRAME:

                frame = (CameraFrame *) msg.arg1;
                if(!frame)
                    {
                    break;
                    }

                if ( (CameraFrame::RAW_FRAME == frame->mFrameType )&&
                    ( NULL != mCameraHal ) &&
                    ( NULL != mDataCb) &&
                    ( NULL != mNotifyCb ) )
                    {

                    if ( mCameraHal->msgTypeEnabled(CAMERA_MSG_RAW_IMAGE) )
                        {
#ifdef COPY_IMAGE_BUFFER
                        copyAndSendPictureFrame(frame, CAMERA_MSG_RAW_IMAGE);
#else
                        //TODO: Find a way to map a Tiler buffer to a MemoryHeapBase
#endif
                        }
                    else {
                        if ( mCameraHal->msgTypeEnabled(CAMERA_MSG_RAW_IMAGE_NOTIFY) ) {
                            mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, mCallbackCookie);
                        }
                        mFrameProvider->returnFrame(frame->mBuffer,
                                                    (CameraFrame::FrameType) frame->mFrameType);
                    }

                    mRawAvailable = true;

                    }
                else if ( (CameraFrame::IMAGE_FRAME == frame->mFrameType) &&
                          (NULL != mCameraHal) &&
                          (NULL != mDataCb) &&
                          (CameraFrame::ENCODE_RAW_YUV422I_TO_JPEG & frame->mQuirks) )
                    {
#if (!JPEG)
                    int encode_quality = 100, tn_quality = 100;
                    int tn_width, tn_height;
                    unsigned int current_snapshot = 0;
                    Encoder_libjpeg::params *main_jpeg = NULL, *tn_jpeg = NULL;
                    void* exif_data = NULL;
                    camera_memory_t* raw_picture = mRequestMemory(-1, frame->mLength, 1, NULL);

                    if(raw_picture) {
                        buf = raw_picture->data;
                    }

                    encode_quality = mParameters.getInt(CameraParameters::KEY_JPEG_QUALITY);
                    if (encode_quality < 0 || encode_quality > 100) {
                        encode_quality = 100;
                    }

                    tn_quality = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY);
                    if (tn_quality < 0 || tn_quality > 100) {
                        tn_quality = 100;
                    }

                    if (CameraFrame::HAS_EXIF_DATA & frame->mQuirks) {
                        exif_data = frame->mCookie2;
                    }

                    main_jpeg = (Encoder_libjpeg::params*)
                                    malloc(sizeof(Encoder_libjpeg::params));
                    if (main_jpeg) {
                        main_jpeg->src = (uint8_t*) frame->mBuffer;
                        main_jpeg->src_size = frame->mLength;
                        main_jpeg->dst = (uint8_t*) buf;
                        main_jpeg->dst_size = frame->mLength;
                        main_jpeg->quality = encode_quality;
                        main_jpeg->in_width = frame->mWidth;
                        main_jpeg->in_height = frame->mHeight;
                        main_jpeg->out_width = frame->mWidth;
                        main_jpeg->out_height = frame->mHeight;
                        main_jpeg->format = CameraParameters::PIXEL_FORMAT_YUV422I;
                    }

                    tn_width = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH);
                    tn_height = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT);

                    if ((tn_width > 0) && (tn_height > 0)) {
                        tn_jpeg = (Encoder_libjpeg::params*)
                                      malloc(sizeof(Encoder_libjpeg::params));
                        // if malloc fails just keep going and encode main jpeg
                        if (!tn_jpeg) {
                            tn_jpeg = NULL;
                        }
                    }

                    if (tn_jpeg) {
                        int width, height;
                        mParameters.getPreviewSize(&width,&height);
                        current_snapshot = (mPreviewBufCount + MAX_BUFFERS - 1) % MAX_BUFFERS;
                        tn_jpeg->src = (uint8_t*) mPreviewBufs[current_snapshot];
                        tn_jpeg->src_size = mPreviewMemory->size / MAX_BUFFERS;
                        tn_jpeg->dst = (uint8_t*) malloc(tn_jpeg->src_size);
                        tn_jpeg->dst_size = tn_jpeg->src_size;
                        tn_jpeg->quality = tn_quality;
                        tn_jpeg->in_width = width;
                        tn_jpeg->in_height = height;
                        tn_jpeg->out_width = tn_width;
                        tn_jpeg->out_height = tn_height;
                        tn_jpeg->format = CameraParameters::PIXEL_FORMAT_YUV420SP;;
                    }

                    sp<Encoder_libjpeg> encoder = new Encoder_libjpeg(main_jpeg,
                                                      tn_jpeg,
                                                      AppCallbackNotifierEncoderCallback,
                                                      (CameraFrame::FrameType)frame->mFrameType,
                                                      this,
                                                      raw_picture,
                                                      exif_data);
                    encoder->run();
                    gEncoderQueue.add(frame->mBuffer, encoder);
                    encoder.clear();
#else
					int jpegSize = 0;
					mCameraHal->getCaptureSize(&jpegSize);

					camera_memory_t* picture = NULL;

					picture = mRequestMemory(-1, jpegSize, 1, NULL);
					if (picture && picture->data) {
						mCameraHal->copyJpegImage( picture->data );
					}

#if defined(TARGET_OMAP3) && defined(OMAP_ENHANCEMENT)
					if ( mBurst )
					{
						mDataCb(CAMERA_MSG_COMPRESSED_BURST_IMAGE, picture, 0, NULL, mCallbackCookie);
					}
					else if(picture && (mNotifierState==AppCallbackNotifier::NOTIFIER_STARTED) &&
							(mCameraHal->msgTypeEnabled(CAMERA_MSG_COMPRESSED_IMAGE)))
#endif
					{
						mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, picture, 0, NULL, mCallbackCookie);
					}

					// Send the callback to the application only if the notifier is started and the message is enabled
					if (picture) {
						picture->release(picture);
					}

					if (mNotifierState == AppCallbackNotifier::NOTIFIER_STARTED)
					{
						mFrameProvider->returnFrame(frame->mBuffer,
                                                ( CameraFrame::FrameType ) frame->mFrameType);
					}
#endif
                    }
                else if ( ( CameraFrame::IMAGE_FRAME == frame->mFrameType ) &&
                             ( NULL != mCameraHal ) &&
                             ( NULL != mDataCb) )
                    {

                    // CTS, MTS requirements: Every 'takePicture()' call
                    // who registers a raw callback should receive one
                    // as well. This is  not always the case with
                    // CameraAdapters though.
                    if (!mRawAvailable) {
                        dummyRaw();
                    } else {
                        mRawAvailable = false;
                    }

#ifdef COPY_IMAGE_BUFFER
                    {
                        Mutex::Autolock lock(mBurstLock);
#if 0 //TODO: enable burst mode later
                        if ( mBurst )
                        {
                            mDataCb(CAMERA_MSG_COMPRESSED_BURST_IMAGE, JPEGPictureMemBase, mCallbackCookie);
                        }
                        else
#endif
                        {
                            copyAndSendPictureFrame(frame, CAMERA_MSG_COMPRESSED_IMAGE);
                        }
                    }
#else
                     //TODO: Find a way to map a Tiler buffer to a MemoryHeapBase
#endif
                    }
                else if ( ( CameraFrame::VIDEO_FRAME_SYNC == frame->mFrameType ) &&
                             ( NULL != mCameraHal ) &&
                             ( NULL != mDataCb) &&
                             ( mCameraHal->msgTypeEnabled(CAMERA_MSG_VIDEO_FRAME)  ) )
                    {
                    mRecordingLock.lock();
                    if(mRecording)
                        {
                        if(mUseMetaDataBufferMode)
                            {
                            camera_memory_t *videoMedatadaBufferMemory =
                                             (camera_memory_t *) mVideoMetadataBufferMemoryMap.valueFor((uint32_t) frame->mBuffer);
                            video_metadata_t *videoMetadataBuffer = (video_metadata_t *) videoMedatadaBufferMemory->data;

                            if( (NULL == videoMedatadaBufferMemory) || (NULL == videoMetadataBuffer) || (NULL == frame->mBuffer) )
                                {
                                CAMHAL_ALOGEA("Error! One of the video buffers is NULL");
                                break;
                                }

                            if ( mUseVideoBuffers )
                              {
                                int vBuf = mVideoMap.valueFor((uint32_t) frame->mBuffer);
                                GraphicBufferMapper &mapper = GraphicBufferMapper::get();
                                Rect bounds;
                                bounds.left = 0;
                                bounds.top = 0;
                                bounds.right = mVideoWidth;
                                bounds.bottom = mVideoHeight;

                                void *y_uv[2];
                                mapper.lock((buffer_handle_t)vBuf, CAMHAL_GRALLOC_USAGE, bounds, y_uv);

                                structConvImage input =  {frame->mWidth,
                                                          frame->mHeight,
                                                          4096,
                                                          IC_FORMAT_YCbCr420_lp,
                                                          (mmByte *)frame->mYuv[0],
                                                          (mmByte *)frame->mYuv[1],
                                                          frame->mOffset};

                                structConvImage output = {mVideoWidth,
                                                          mVideoHeight,
                                                          4096,
                                                          IC_FORMAT_YCbCr420_lp,
                                                          (mmByte *)y_uv[0],
                                                          (mmByte *)y_uv[1],
                                                          0};

                                VT_resizeFrame_Video_opt2_lp(&input, &output, NULL, 0);
                                mapper.unlock((buffer_handle_t)vBuf);
                                videoMetadataBuffer->metadataBufferType = (int) kMetadataBufferTypeCameraSource;
                                videoMetadataBuffer->handle = (void *)vBuf;
                                videoMetadataBuffer->offset = 0;
                              }
                            else
                              {
                                videoMetadataBuffer->metadataBufferType = (int) kMetadataBufferTypeCameraSource;
								int index = mPreviewBufIndex.valueFor((uint32_t) frame->mBuffer);

								videoMetadataBuffer->handle = (char*)mVirtualBuffer.keyAt(index); //frame->mBuffer;
                                videoMetadataBuffer->offset = frame->mOffset;
                              }

                            CAMHAL_ALOGVB("mDataCbTimestamp : frame->mBuffer=0x%x, videoMetadataBuffer=0x%x, videoMedatadaBufferMemory=0x%x",
                                            frame->mBuffer, videoMetadataBuffer, videoMedatadaBufferMemory);

                            mDataCbTimestamp(frame->mTimestamp, CAMERA_MSG_VIDEO_FRAME,
                                                videoMedatadaBufferMemory, 0, mCallbackCookie);
                            }
                        else
                            {
                            //TODO: Need to revisit this, should ideally be mapping the TILER buffer using mRequestMemory
                            camera_memory_t* fakebuf = mRequestMemory(-1, 4, 1, NULL);
                            if( (NULL == fakebuf) || ( NULL == fakebuf->data) || ( NULL == frame->mBuffer))
                                {
                                CAMHAL_ALOGEA("Error! One of the video buffers is NULL");
                                break;
                                }

							int index = mPreviewBufIndex.valueFor((uint32_t) frame->mBuffer);
							fakebuf->data = (char*)mVirtualBuffer.keyAt(index);
                            mDataCbTimestamp(frame->mTimestamp, CAMERA_MSG_VIDEO_FRAME, fakebuf, 0, mCallbackCookie);
                            fakebuf->release(fakebuf);
                            }
                        }
                    mRecordingLock.unlock();

                    }
                else if(( CameraFrame::SNAPSHOT_FRAME == frame->mFrameType ) &&
                             ( NULL != mCameraHal ) &&
                             ( NULL != mDataCb) &&
                             ( NULL != mNotifyCb)) {
                    //When enabled, measurement data is sent instead of video data
                    if ( !mMeasurementEnabled ) {
                        copyAndSendPreviewFrame(frame, CAMERA_MSG_POSTVIEW_FRAME);
                    } else {
                        mFrameProvider->returnFrame(frame->mBuffer,
                                                    (CameraFrame::FrameType) frame->mFrameType);
                    }
                }
                else if ( ( CameraFrame::PREVIEW_FRAME_SYNC== frame->mFrameType ) &&
                            ( NULL != mCameraHal ) &&
                            ( NULL != mDataCb) &&
                            ( mCameraHal->msgTypeEnabled(CAMERA_MSG_PREVIEW_FRAME)) ) {
                    //When enabled, measurement data is sent instead of video data
                    if ( !mMeasurementEnabled ) {
                        copyAndSendPreviewFrame(frame, CAMERA_MSG_PREVIEW_FRAME);
                    } else {
                         mFrameProvider->returnFrame(frame->mBuffer,
                                                     (CameraFrame::FrameType) frame->mFrameType);
                    }
                }
                else if ( ( CameraFrame::FRAME_DATA_SYNC == frame->mFrameType ) &&
                            ( NULL != mCameraHal ) &&
                            ( NULL != mDataCb) &&
                            ( mCameraHal->msgTypeEnabled(CAMERA_MSG_PREVIEW_FRAME)) ) {
                    copyAndSendPreviewFrame(frame, CAMERA_MSG_PREVIEW_FRAME);
                } else {
                    mFrameProvider->returnFrame(frame->mBuffer,
                                                ( CameraFrame::FrameType ) frame->mFrameType);
                    CAMHAL_ALOGDB("Frame type 0x%x is still unsupported!", frame->mFrameType);
                }

                break;

        default:

            break;

        };

exit:

    if ( NULL != frame )
        {
        delete frame;
        }

    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::frameCallbackRelay(CameraFrame* caFrame)
{
    LOG_FUNCTION_NAME;
    AppCallbackNotifier *appcbn = (AppCallbackNotifier*) (caFrame->mCookie);
    appcbn->frameCallback(caFrame);
    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::frameCallback(CameraFrame* caFrame)
{
    ///Post the event to the event queue of AppCallbackNotifier
    TIUTILS::Message msg;
    CameraFrame *frame;

    LOG_FUNCTION_NAME;

    if ( NULL != caFrame )
        {

        frame = new CameraFrame(*caFrame);
        if ( NULL != frame )
            {
              msg.command = AppCallbackNotifier::NOTIFIER_CMD_PROCESS_FRAME;
              msg.arg1 = frame;
              mFrameQ.put(&msg);
            }
        else
            {
            CAMHAL_ALOGEA("Not enough resources to allocate CameraFrame");
            }

        }

    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::flushAndReturnFrames()
{
    TIUTILS::Message msg;
    CameraFrame *frame;

    Mutex::Autolock lock(mLock);
    while (!mFrameQ.isEmpty()) {
        mFrameQ.get(&msg);
        frame = (CameraFrame*) msg.arg1;
        if (frame) {
            mFrameProvider->returnFrame(frame->mBuffer,
                                        (CameraFrame::FrameType) frame->mFrameType);
        }
    }

    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::eventCallbackRelay(CameraHalEvent* chEvt)
{
    LOG_FUNCTION_NAME;
    AppCallbackNotifier *appcbn = (AppCallbackNotifier*) (chEvt->mCookie);
    appcbn->eventCallback(chEvt);
    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::eventCallback(CameraHalEvent* chEvt)
{

    ///Post the event to the event queue of AppCallbackNotifier
    TIUTILS::Message msg;
    CameraHalEvent *event;


    LOG_FUNCTION_NAME;

    if ( NULL != chEvt )
        {

        event = new CameraHalEvent(*chEvt);
        if ( NULL != event )
            {
            msg.command = AppCallbackNotifier::NOTIFIER_CMD_PROCESS_EVENT;
            msg.arg1 = event;
            mEventQ.put(&msg);
            }
        else
            {
            CAMHAL_ALOGEA("Not enough resources to allocate CameraHalEvent");
            }

        }

    LOG_FUNCTION_NAME_EXIT;
}


bool AppCallbackNotifier::processMessage()
{
    ///Retrieve the command from the command queue and process it
    TIUTILS::Message msg;

    LOG_FUNCTION_NAME;

    CAMHAL_ALOGDA("+Msg get...");
    mNotificationThread->msgQ().get(&msg);
    CAMHAL_ALOGDA("-Msg get...");
    bool ret = true;

    switch(msg.command)
      {
        case NotificationThread::NOTIFIER_EXIT:
          {
            CAMHAL_ALOGEA("Received NOTIFIER_EXIT command from Camera HAL");
            mNotifierState = AppCallbackNotifier::NOTIFIER_EXITED;
            ret = false;
            break;
          }
        default:
          {
            CAMHAL_ALOGEA("Error: ProcessMsg() command from Camera HAL");
            break;
          }
      }

    LOG_FUNCTION_NAME_EXIT;

    return ret;


}

AppCallbackNotifier::~AppCallbackNotifier()
{
    LOG_FUNCTION_NAME;

    ///Stop app callback notifier if not already stopped
    stop();

    ///Unregister with the frame provider
    if ( NULL != mFrameProvider )
        {
        mFrameProvider->disableFrameNotification(CameraFrame::ALL_FRAMES);
        }

    //unregister with the event provider
    if ( NULL != mEventProvider )
        {
        mEventProvider->disableEventNotification(CameraHalEvent::ALL_EVENTS);
        }

    TIUTILS::Message msg = {0,0,0,0,0,0};
    msg.command = NotificationThread::NOTIFIER_EXIT;

    ///Post the message to display thread
    mNotificationThread->msgQ().put(&msg);

    //Exit and cleanup the thread
    mNotificationThread->requestExit();
    mNotificationThread->join();

    //Delete the display thread
    mNotificationThread.clear();


    ///Free the event and frame providers
    if ( NULL != mEventProvider )
        {
        ///Deleting the event provider
        CAMHAL_ALOGDA("Stopping Event Provider");
        delete mEventProvider;
        mEventProvider = NULL;
        }

    if ( NULL != mFrameProvider )
        {
        ///Deleting the frame provider
        CAMHAL_ALOGDA("Stopping Frame Provider");
        delete mFrameProvider;
        mFrameProvider = NULL;
        }

    releaseSharedVideoBuffers();

    LOG_FUNCTION_NAME_EXIT;
}

//Free all video heaps and buffers
void AppCallbackNotifier::releaseSharedVideoBuffers()
{
    LOG_FUNCTION_NAME;

    if(mUseMetaDataBufferMode)
    {
        camera_memory_t* videoMedatadaBufferMemory;
        for (unsigned int i = 0; i < mVideoMetadataBufferMemoryMap.size();  i++)
            {
            videoMedatadaBufferMemory = (camera_memory_t*) mVideoMetadataBufferMemoryMap.valueAt(i);
            if(NULL != videoMedatadaBufferMemory)
                {
                videoMedatadaBufferMemory->release(videoMedatadaBufferMemory);
                CAMHAL_ALOGDB("Released  videoMedatadaBufferMemory=0x%x", videoMedatadaBufferMemory);
                }
            }

        mVideoMetadataBufferMemoryMap.clear();
        mVideoMetadataBufferReverseMap.clear();
        if (mUseVideoBuffers)
		{
            mVideoMap.clear();
		}
		if ( !mVirtualBuffer.isEmpty() )
		{
			mVirtualBuffer.clear();
		}

		if ( !mPreviewBufIndex.isEmpty() )
		{
			mPreviewBufIndex.clear();
		}
    }

    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::setEventProvider(int32_t eventMask, MessageNotifier * eventNotifier)
{

    LOG_FUNCTION_NAME;
    ///@remarks There is no NULL check here. We will check
    ///for NULL when we get start command from CameraHal
    ///@Remarks Currently only one event provider (CameraAdapter) is supported
    ///@todo Have an array of event providers for each event bitmask
    mEventProvider = new EventProvider(eventNotifier, this, eventCallbackRelay);
    if ( NULL == mEventProvider )
        {
        CAMHAL_ALOGEA("Error in creating EventProvider");
        }
    else
        {
        mEventProvider->enableEventNotification(eventMask);
        }

    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::setFrameProvider(FrameNotifier *frameNotifier)
{
    LOG_FUNCTION_NAME;
    ///@remarks There is no NULL check here. We will check
    ///for NULL when we get the start command from CameraAdapter
    mFrameProvider = new FrameProvider(frameNotifier, this, frameCallbackRelay);
    if ( NULL == mFrameProvider )
        {
        CAMHAL_ALOGEA("Error in creating FrameProvider");
        }
    else
        {
        //Register only for captured images and RAW for now
        //TODO: Register for and handle all types of frames
        mFrameProvider->enableFrameNotification(CameraFrame::IMAGE_FRAME);
        mFrameProvider->enableFrameNotification(CameraFrame::RAW_FRAME);
        }

    LOG_FUNCTION_NAME_EXIT;
}

status_t AppCallbackNotifier::startPreviewCallbacks(CameraParameters &params, void *buffers, uint32_t *offsets, int fd, size_t length, size_t count)
{
    sp<MemoryHeapBase> heap;
    sp<MemoryBase> buffer;
    unsigned int *bufArr;
    size_t size = 0;

    LOG_FUNCTION_NAME;

    Mutex::Autolock lock(mLock);

    if ( NULL == mFrameProvider )
        {
        CAMHAL_ALOGEA("Trying to start video recording without FrameProvider");
        return -EINVAL;
        }

    if ( mPreviewing )
        {
        CAMHAL_ALOGDA("+Already previewing");
        return NO_INIT;
        }

    int w,h;
    ///Get preview size
    params.getPreviewSize(&w, &h);

    //Get the preview pixel format
    mPreviewPixelFormat = params.getPreviewFormat();

     if(strcmp(mPreviewPixelFormat, (const char *) CameraParameters::PIXEL_FORMAT_YUV422I) == 0)
        {
        size = w*h*2;
        mPreviewPixelFormat = CameraParameters::PIXEL_FORMAT_YUV422I;
        }
    else if(strcmp(mPreviewPixelFormat, (const char *) CameraParameters::PIXEL_FORMAT_YUV420SP) == 0 ||
            strcmp(mPreviewPixelFormat, (const char *) CameraParameters::PIXEL_FORMAT_YUV420P) == 0)
        {
        size = (w*h*3)/2;
        mPreviewPixelFormat = CameraParameters::PIXEL_FORMAT_YUV420SP;
        }
    else if(strcmp(mPreviewPixelFormat, (const char *) CameraParameters::PIXEL_FORMAT_RGB565) == 0)
        {
        size = w*h*2;
        mPreviewPixelFormat = CameraParameters::PIXEL_FORMAT_RGB565;
        }

 /* 20120614 jungyeal@lge.com Workaround 192x144 -> 176x144 Crop  for VT [START] */	
#ifdef LGE_JUSTIN_DEVICE
    if (w == 192)		//for VT
    {
        size = 176 * 144 * 3 / 2;
        ALOGD("mPreviewMemory force size =%d changed for VT",size);
    }
   else if (w == 320 && mCameraHal->mFaceUnlockMode) //for face_UnLock 
   {
		size = 640 * 480 * 3 / 2;	
		ALOGD("mPreviewMemory force size =%d changed for face unlock ",size);
   }
/* 20120614 jungyeal@lge.com Workaround 192x144 -> 176x144 Crop	for VT [END] */
#else
// 2012.08.13 rt5604 BUG fix for CTS testPreviewCallback ->
    if (w == 192)		//for VT
    {
        size = 176 * h * 3 / 2;
        ALOGD("rt5604: ---> mPreviewMemory force size =%d changed for CTS",size);
    }
// 2012.08.13 rt5604 BUG fix for CTS testPreviewCallback <-
#endif
/* 20120614 jungyeal@lge.com Workaround 192x144 -> 176x144 Crop  for VT [END] */

// 2012.08.13 rt5604 BUG fix for CTS testPreviewCallback ->
    if (w == 736)		//for CTS
	{
		size = 720 * h * 3 / 2;
		ALOGD("rt5604: ---> mPreviewMemory force size =%d changed for CTS",size);
	}
// 2012.08.13 rt5604 BUG fix for CTS testPreviewCallback <-

    mPreviewMemory = mRequestMemory(-1, size, AppCallbackNotifier::MAX_BUFFERS, NULL);
    if (!mPreviewMemory) {
        return NO_MEMORY;
    }

    for (int i=0; i < AppCallbackNotifier::MAX_BUFFERS; i++) {
        mPreviewBufs[i] = (unsigned char*) mPreviewMemory->data + (i*size);
    }

    if ( mCameraHal->msgTypeEnabled(CAMERA_MSG_PREVIEW_FRAME ) ) {
         mFrameProvider->enableFrameNotification(CameraFrame::PREVIEW_FRAME_SYNC);
    }

    mPreviewBufCount = 0;

    mPreviewing = true;

    LOG_FUNCTION_NAME;

    return NO_ERROR;
}

void AppCallbackNotifier::setBurst(bool burst)
{
    LOG_FUNCTION_NAME;

    Mutex::Autolock lock(mBurstLock);

    mBurst = burst;

    LOG_FUNCTION_NAME_EXIT;
}

void AppCallbackNotifier::useVideoBuffers(bool useVideoBuffers)
{
  LOG_FUNCTION_NAME;

  mUseVideoBuffers = useVideoBuffers;

  LOG_FUNCTION_NAME_EXIT;
}

bool AppCallbackNotifier::getUesVideoBuffers()
{
    return mUseVideoBuffers;
}

void AppCallbackNotifier::setVideoRes(int width, int height)
{
  LOG_FUNCTION_NAME;

  mVideoWidth = width;
  mVideoHeight = height;

  LOG_FUNCTION_NAME_EXIT;
}

int AppCallbackNotifier::setParameters(const CameraParameters& params)
{
    LOG_FUNCTION_NAME;

    mParameters = params;

    LOG_FUNCTION_NAME_EXIT;
    return NO_ERROR;
}

status_t AppCallbackNotifier::stopPreviewCallbacks()
{
    sp<MemoryHeapBase> heap;
    sp<MemoryBase> buffer;

    LOG_FUNCTION_NAME;

    if ( NULL == mFrameProvider )
        {
        CAMHAL_ALOGEA("Trying to stop preview callbacks without FrameProvider");
        return -EINVAL;
        }

    if ( !mPreviewing )
        {
        return NO_INIT;
        }

    mFrameProvider->disableFrameNotification(CameraFrame::PREVIEW_FRAME_SYNC);

    {
    Mutex::Autolock lock(mLock);
    mPreviewMemory->release(mPreviewMemory);
    }

    mPreviewing = false;

    LOG_FUNCTION_NAME_EXIT;

    return NO_ERROR;

}

status_t AppCallbackNotifier::useMetaDataBufferMode(bool enable)
{
    mUseMetaDataBufferMode = enable;

    return NO_ERROR;
}


status_t AppCallbackNotifier::startRecording()
{
    status_t ret = NO_ERROR;

    LOG_FUNCTION_NAME;

   Mutex::Autolock lock(mRecordingLock);

    if ( NULL == mFrameProvider )
        {
        CAMHAL_ALOGEA("Trying to start video recording without FrameProvider");
        ret = -1;
        }

    if(mRecording)
        {
        return NO_INIT;
        }

    if ( NO_ERROR == ret )
        {
         mFrameProvider->enableFrameNotification(CameraFrame::VIDEO_FRAME_SYNC);
        }

    mRecording = true;

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

//Allocate metadata buffers for video recording
status_t AppCallbackNotifier::initSharedVideoBuffers(void *buffers, uint32_t *offsets, int fd, size_t length, size_t count, void **vidBufs)
{
    status_t ret = NO_ERROR;
    LOG_FUNCTION_NAME;

    if(mUseMetaDataBufferMode)
        {
        uint32_t *bufArr = NULL;
        camera_memory_t* videoMedatadaBufferMemory = NULL;

        if(NULL == buffers)
            {
            CAMHAL_ALOGEA("Error! Video buffers are NULL");
            return BAD_VALUE;
            }
        bufArr = (uint32_t *) buffers;

        for (uint32_t i = 0; i < count; i++)
            {
            videoMedatadaBufferMemory = mRequestMemory(-1, sizeof(video_metadata_t), 1, NULL);
            if((NULL == videoMedatadaBufferMemory) || (NULL == videoMedatadaBufferMemory->data))
                {
                CAMHAL_ALOGEA("Error! Could not allocate memory for Video Metadata Buffers");
                return NO_MEMORY;
                }

            mVideoMetadataBufferMemoryMap.add(bufArr[i], (uint32_t)(videoMedatadaBufferMemory));
            mVideoMetadataBufferReverseMap.add((uint32_t)(videoMedatadaBufferMemory->data), bufArr[i]);
            CAMHAL_ALOGDB("bufArr[%d]=0x%x, videoMedatadaBufferMemory=0x%x, videoMedatadaBufferMemory->data=0x%x",
                    i, bufArr[i], videoMedatadaBufferMemory, videoMedatadaBufferMemory->data);

			mPreviewBufIndex.add(bufArr[i],i);

            if (vidBufs != NULL)
              {
                char** vBufArr = (char**) vidBufs;
                mVirtualBuffer.add(vBufArr[i],i);
                mVideoMap.add(bufArr[i], (unsigned int)vBufArr[i]);
              }
            }
        }
		else
        {
            uint32_t *bufArr = (uint32_t *) buffers;
            char** vBufArr = (char**) vidBufs;

            for (uint32_t i = 0; i < 6; i++)
            {
                mPreviewBufIndex.add(bufArr[i],i);
                mVirtualBuffer.add(vBufArr[i],i);
            }
        }

exit:
    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

status_t AppCallbackNotifier::stopRecording()
{
    status_t ret = NO_ERROR;

    LOG_FUNCTION_NAME;

    Mutex::Autolock lock(mRecordingLock);

    if ( NULL == mFrameProvider )
        {
        CAMHAL_ALOGEA("Trying to stop video recording without FrameProvider");
        ret = -1;
        }

    if(!mRecording)
        {
        return NO_INIT;
        }

    if ( NO_ERROR == ret )
        {
		 mFrameProvider->disableFrameNotification(CameraFrame::PREVIEW_FRAME_SYNC);
         mFrameProvider->disableFrameNotification(CameraFrame::VIDEO_FRAME_SYNC);
        }

    ///Release the shared video buffers
    releaseSharedVideoBuffers();

    mRecording = false;

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

status_t AppCallbackNotifier::releaseRecordingFrame(const void* mem)
{
    status_t ret = NO_ERROR;
    void *frame = NULL;

    LOG_FUNCTION_NAME;
    if ( NULL == mFrameProvider )
        {
        CAMHAL_ALOGEA("Trying to stop video recording without FrameProvider");
        ret = -1;
        }

    if ( NULL == mem )
        {
        CAMHAL_ALOGEA("Video Frame released is invalid");
        ret = -1;
        }

    if( NO_ERROR != ret )
        {
        return ret;
        }

    if(mUseMetaDataBufferMode)
        {
        video_metadata_t *videoMetadataBuffer = (video_metadata_t *) mem ;
        frame = (void*) mVideoMetadataBufferReverseMap.valueFor((uint32_t) videoMetadataBuffer);
        CAMHAL_ALOGVB("Releasing frame with videoMetadataBuffer=0x%x, videoMetadataBuffer->handle=0x%x & frame handle=0x%x\n",
                       videoMetadataBuffer, videoMetadataBuffer->handle, frame);
        }
    else
        {
        frame = (void*)(*((uint32_t *)mem));
        }

    if ( NO_ERROR == ret )
        {
         ret = mFrameProvider->returnFrame(frame, CameraFrame::VIDEO_FRAME_SYNC);
        }

    LOG_FUNCTION_NAME_EXIT;

    return ret;
}

status_t AppCallbackNotifier::enableMsgType(int32_t msgType)
{
    if( msgType & (CAMERA_MSG_POSTVIEW_FRAME | CAMERA_MSG_PREVIEW_FRAME) ) {
        mFrameProvider->enableFrameNotification(CameraFrame::PREVIEW_FRAME_SYNC);
    }

    return NO_ERROR;
}

status_t AppCallbackNotifier::disableMsgType(int32_t msgType)
{
    if(!mCameraHal->msgTypeEnabled(CAMERA_MSG_PREVIEW_FRAME | CAMERA_MSG_POSTVIEW_FRAME)) {
        mFrameProvider->disableFrameNotification(CameraFrame::PREVIEW_FRAME_SYNC);
    }

    return NO_ERROR;

}

status_t AppCallbackNotifier::start()
{
    LOG_FUNCTION_NAME;
    if(mNotifierState==AppCallbackNotifier::NOTIFIER_STARTED)
        {
        CAMHAL_ALOGDA("AppCallbackNotifier already running");
        LOG_FUNCTION_NAME_EXIT;
        return ALREADY_EXISTS;
        }

    ///Check whether initial conditions are met for us to start
    ///A frame provider should be available, if not return error
    if(!mFrameProvider)
        {
        ///AppCallbackNotifier not properly initialized
        CAMHAL_ALOGEA("AppCallbackNotifier not properly initialized - Frame provider is NULL");
        LOG_FUNCTION_NAME_EXIT;
        return NO_INIT;
        }

    ///At least one event notifier should be available, if not return error
    ///@todo Modify here when there is an array of event providers
    if(!mEventProvider)
        {
        CAMHAL_ALOGEA("AppCallbackNotifier not properly initialized - Event provider is NULL");
        LOG_FUNCTION_NAME_EXIT;
        ///AppCallbackNotifier not properly initialized
        return NO_INIT;
        }

    mNotifierState = AppCallbackNotifier::NOTIFIER_STARTED;
    CAMHAL_ALOGDA(" --> AppCallbackNotifier NOTIFIER_STARTED \n");

    gEncoderQueue.clear();

    LOG_FUNCTION_NAME_EXIT;

    return NO_ERROR;

}

status_t AppCallbackNotifier::stop()
{
    LOG_FUNCTION_NAME;

    if(mNotifierState!=AppCallbackNotifier::NOTIFIER_STARTED)
        {
        CAMHAL_ALOGDA("AppCallbackNotifier already in stopped state");
        LOG_FUNCTION_NAME_EXIT;
        return ALREADY_EXISTS;
        }
    {
    Mutex::Autolock lock(mLock);

    mNotifierState = AppCallbackNotifier::NOTIFIER_STOPPED;
    CAMHAL_ALOGDA(" --> AppCallbackNotifier NOTIFIER_STOPPED \n");
    }

    while(!gEncoderQueue.isEmpty()) {
        sp<Encoder_libjpeg> encoder = gEncoderQueue.valueAt(0);
        if(encoder.get()) {
            encoder->cancel();
            encoder->join();
            encoder.clear();
        }
        gEncoderQueue.removeItemsAt(0);
    }

    LOG_FUNCTION_NAME_EXIT;
    return NO_ERROR;
}


/*--------------------NotificationHandler Class ENDS here-----------------------------*/



};
