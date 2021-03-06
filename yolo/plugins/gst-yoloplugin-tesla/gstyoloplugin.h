/**
MIT License

Copyright (c) 2018 NVIDIA CORPORATION. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*
*/

#ifndef __GST_YOLOPLUGIN_H__
#define __GST_YOLOPLUGIN_H__

#include <gst/base/gstbasetransform.h>
#include <gst/video/video.h>

/* Open CV headers */
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "gst-nvquery.h"
#include "gstnvdsmeta.h"
#include "gstnvstreammeta.h"
#include "nvbuffer.h"
#include "yoloplugin_lib.h"
#include <cuda.h>
#include <cuda_runtime.h>

/* Package and library details required for plugin_init */
#define PACKAGE "nvyolo"
#define VERSION "1.0"
#define LICENSE "Proprietary"
#define DESCRIPTION "NVIDIA example plugin for integration with DeepStream on DGPU"
#define BINARY_PACKAGE "NVIDIA DeepStream 3rdparty IP integration example plugin"
#define URL "http://nvidia.com/"

G_BEGIN_DECLS
/* Standard boilerplate stuff */
typedef struct _GstYoloPlugin GstYoloPlugin;
typedef struct _GstYoloPluginClass GstYoloPluginClass;

/* Standard boilerplate stuff */
#define GST_TYPE_YOLOPLUGIN (gst_yoloplugin_get_type())
#define GST_YOLOPLUGIN(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_YOLOPLUGIN, GstYoloPlugin))
#define GST_YOLOPLUGIN_CLASS(klass)                                                                \
    (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_YOLOPLUGIN, GstYoloPluginClass))
#define GST_YOLOPLUGIN_GET_CLASS(obj)                                                              \
    (G_TYPE_INSTANCE_GET_CLASS((obj), GST_TYPE_YOLOPLUGIN, GstYoloPluginClass))
#define GST_IS_YOLOPLUGIN(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_YOLOPLUGIN))
#define GST_IS_YOLOPLUGIN_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_YOLOPLUGIN))
#define GST_YOLOPLUGIN_CAST(obj) ((GstYoloPlugin*) (obj))

struct _GstYoloPlugin
{
  GstBaseTransform base_trans;

  // Context of the custom algorithm library
  YoloPluginCtx *yolopluginlib_ctx;

  // Unique ID of the element. The labels generated by the element will be
  // updated at index `unique_id` of attr_info array in NvDsObjectParams.
  guint unique_id;

  // Frame number of the current input buffer
  guint64 frame_num;

  // NPP Stream used for allocating the CUDA task
  cudaStream_t npp_stream;

  // the scratch conversion host buffer for DGPU
  void *hconv_buf;

  // OpenCV mat to remove padding and convert RGBA to RGB
    std::vector < cv::Mat * >cvmats;

  // Input video info (resolution, color format, framerate, etc)
  GstVideoInfo video_info;

  // Resolution at which frames/objects should be processed
  gint processing_width;
  gint processing_height;

  // Amount of objects processed in single call to algorithm
  guint batch_size;

  // GPU ID on which we expect to execute the task
  guint gpu_id;

  // Boolean indicating if entire frame or cropped objects should be processed
  gboolean process_full_frame;

  //plugin config file path
  gchar *config_file_path;
};

// Boiler plate stuff
struct _GstYoloPluginClass
{
  GstBaseTransformClass parent_class;
};

GType gst_yoloplugin_get_type (void);

G_END_DECLS
#endif /* __GST_YOLOPLUGIN_H__ */
