/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <unistd.h>
#include "gstnvdsmeta.h"
//for camera
#include "ip_cam.h"
#include "simple_cache.h"

#define MAX_DISPLAY_LEN 64

#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1080

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

gint frame_number = 0;
gchar pgie_classes_str[4][32] = { "Vehicle", "TwoWheeler", "Person",
  "Roadsign"
};
// APP_Source
AppSrcData* mAppSrcData;
// End APP_Source

#ifdef ENABLE_RECORDING
FILE *vRawIn;
//~ FILE *vRawOut;
#endif

static gboolean forceStop = FALSE;

static void sighandler(int signum)
{
	/*close files*/
#ifdef ENABLE_RECORDING
	fclose(vRawIn);
	//~ fclose(vRawOut);
#endif
    g_print ("sighandler: %d\n", signum);
    forceStop = TRUE;
    // gst_app_src_end_of_stream ((GstAppSrc *) mAppSrcData->app_source);
    g_main_loop_quit (mAppSrcData->loop);
}

/* osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */

static GstPadProbeReturn
osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{
    GstBuffer *buf = (GstBuffer *) info->data;
    guint num_rects = 0; 
    NvDsObjectMeta *obj_meta = NULL;
    guint vehicle_count = 0;
    guint person_count = 0;
    NvDsMetaList * l_frame = NULL;
    NvDsMetaList * l_obj = NULL;
    NvDsDisplayMeta *display_meta = NULL;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
        int offset = 0;
        for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
                l_obj = l_obj->next) {
            obj_meta = (NvDsObjectMeta *) (l_obj->data);
            if (obj_meta->class_id == PGIE_CLASS_ID_VEHICLE) {
                vehicle_count++;
                num_rects++;
            }
            if (obj_meta->class_id == PGIE_CLASS_ID_PERSON) {
                person_count++;
                num_rects++;
            }
        }
        display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
        NvOSD_TextParams *txt_params  = &display_meta->text_params[0];
        display_meta->num_labels = 1;
        txt_params->display_text = (char*)g_malloc0 (MAX_DISPLAY_LEN);
        offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Person = %d ", person_count);
        offset = snprintf(txt_params->display_text + offset , MAX_DISPLAY_LEN, "Vehicle = %d ", vehicle_count);

        /* Now set the offsets where the string should appear */
        txt_params->x_offset = 10;
        txt_params->y_offset = 12;

        /* Font , font-color and font-size */
        txt_params->font_params.font_name = (char*)"Serif";
        txt_params->font_params.font_size = 10;
        txt_params->font_params.font_color.red = 1.0;
        txt_params->font_params.font_color.green = 1.0;
        txt_params->font_params.font_color.blue = 1.0;
        txt_params->font_params.font_color.alpha = 1.0;

        /* Text background color */
        txt_params->set_bg_clr = 1;
        txt_params->text_bg_clr.red = 0.0;
        txt_params->text_bg_clr.green = 0.0;
        txt_params->text_bg_clr.blue = 0.0;
        txt_params->text_bg_clr.alpha = 1.0;

        nvds_add_display_meta_to_frame(frame_meta, display_meta);
    }

    //~ g_print ("Frame Number = %d Number of objects = %d "
            //~ "Vehicle Count = %d Person Count = %d\n",
            //~ frame_number, num_rects, vehicle_count, person_count);
    frame_number++;
    return GST_PAD_PROBE_OK;
}

static gboolean
bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      if(forceStop) {
        g_print ("End of stream\n");
        g_main_loop_quit (loop);
      }
      break;
    case GST_MESSAGE_ERROR:{
      gchar *debug;
      GError *error;
      gst_message_parse_error (msg, &error, &debug);
      g_printerr ("ERROR from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), error->message);
      if (debug)
        g_printerr ("Error details: %s\n", debug);
      g_free (debug);
      g_error_free (error);
      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

// For camera
#ifdef IP_CAM_INPUT
static void search_device_cb(KHJ::searchDeviceInfo *info, int count)
{
	g_print("----Search------%d\n", count);
	for (int i = 0; i < count; ++i)
		g_print("Search %s\n", info[i].UID);
	g_print("----Search------%d\n", count);
}

static void video_recv_cb(char *buffer, int size, uint64_t pts, bool is_key)
{
	//~ g_print("video: %p %d %ld %d\n", buffer, size, pts, is_key);
    //~ g_print("mAppSrcData->cache: %p\n", mAppSrcData->cache);
#ifdef ENABLE_RECORDING
    fwrite (buffer , sizeof(char), size, vRawIn);
#endif
    simple_cache_write(mAppSrcData->cache, buffer, size);
}

static void connect_cb(std::shared_ptr<KHJ::CameraBase> c, int status)
{
    g_print("connect_cb: UID: %s, status=%d\n", c->getUID().c_str(), status);
    mAppSrcData->cam_status = status;
	if (0 == status) {
		g_print("Connect success...recv video...\n");
#ifdef ENABLE_RECORDING
        vRawIn = fopen("videoraw.h265", "wb");
#endif
        mAppSrcData->is_offline = FALSE;
		c->startRecvVideo(true, video_recv_cb);
	}
}

static void offline_cb(std::shared_ptr<KHJ::CameraBase> c)
{
    g_print("offline_cb: [UID: %s]\n", c->getUID().c_str());
    mAppSrcData->is_offline = TRUE;
}
#endif // IP_CAM_INPUT
// end Camera

int
main (int argc, char *argv[])
{
  signal(SIGINT, sighandler);

  GstElement *pipeline = NULL, *source = NULL, *h265parser = NULL,
      *decoder = NULL, *streammux = NULL, *sink = NULL, *pgie = NULL, *nvvidconv = NULL,
      *nvosd = NULL;
  GstElement *transform = NULL;
  GstBus *bus = NULL;
  guint bus_watch_id;
  GstPad *osd_sink_pad = NULL;
  gboolean add_bus_watch_id = FALSE;

  GstPad *sinkpad, *srcpad;
  gchar pad_name_sink[16] = "sink_0";
  gchar pad_name_src[16] = "src";

  /* Check input arguments */
  if (argc != 3) {
#ifdef IP_CAM_INPUT
    g_printerr ("Usage: %s <Camera UUID> <Config filename>\n", argv[0]);
#else
    g_printerr ("Usage: %s <H265 filename> <Config filename>\n", argv[0]);
#endif
    return -1;
  }
  // APP_Source
  guint cache_size_max = 1024*1024*10; //10M
  mAppSrcData = ip_cam_init_source(cache_size_max);
  if(mAppSrcData == NULL) {
    g_printerr("Can not init App Source\n");
    return -1;
  }
  /* Standard GStreamer initialization */
  gst_init (&argc, &argv);
  mAppSrcData->loop = g_main_loop_new (NULL, FALSE);
start_app:
  add_bus_watch_id = FALSE;
#ifdef IP_CAM_INPUT
  std::shared_ptr<KHJ::CameraBase> camera = nullptr;
  while(camera == nullptr){
      if(forceStop){
        goto out;
      }
      camera = ip_cam_connect(mAppSrcData, std::string(argv[1]), "admin", "888888", search_device_cb, connect_cb, offline_cb);
      if(camera ==  nullptr){
        g_printerr ("Can not connect camera. Retry after 30 seconds\n");
        sleep(30);
      }
  }
#else
  mAppSrcData->file = fopen (argv[1], "r");
#endif
  /* Create gstreamer elements */
  /* Create Pipeline element that will form a connection of other elements */
  pipeline = gst_pipeline_new ("dstest-appsrc-pipeline");
  if (!pipeline) {
    g_printerr ("Pipeline could not be created. Exiting.\n");
    goto disconnect_cam; // free GMainLoop and disconnect Camera
  }

  /* App Source element for reading from raw video file */
  mAppSrcData->app_source = gst_element_factory_make ("appsrc", "app-source");
  if (!mAppSrcData->app_source) {
    g_printerr ("Appsrc element could not be created. Exiting.\n");
    goto disconnect_cam;
  }

  /* Since the data format in the input file is elementary h265 stream,
   * we need a h265parser */
  h265parser = gst_element_factory_make ("h265parse", "h265-parser");

  /* Use nvdec_h265 for hardware accelerated decode on GPU */
  decoder = gst_element_factory_make ("nvv4l2decoder", "nvv4l2-decoder");

  /* Create nvstreammux instance to form batches from one or more sources. */
  streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");

  if (!streammux) {
    g_printerr ("One element could not be created. Exiting.\n");
    goto stop_playback;
  }

  /* Use nvinfer to run inferencing on decoder's output,
   * behaviour of inferencing is set through config file */
  pgie = gst_element_factory_make ("nvinfer", "primary-nvinference-engine");

  /* Use convertor to convert from NV12 to RGBA as required by nvosd */
  nvvidconv = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");

  /* Create OSD to draw on the converted RGBA buffer */
  nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");

  /* Finally render the osd output */
  transform = gst_element_factory_make ("nvegltransform", "nvegl-transform");
  sink = gst_element_factory_make ("nveglglessink", "nvvideo-renderer");

  if (!h265parser || !decoder || !pgie
      || !nvvidconv || !nvosd || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    goto stop_playback;
  }

  if(!transform) {
    g_printerr ("One tegra element could not be created. Exiting.\n");
    goto stop_playback;
  }

  /* APP_Source
   * Configure APP SOurce */
  g_object_set (mAppSrcData->app_source, "caps",
      gst_caps_new_simple ("video/x-h265",
          "format", G_TYPE_STRING, "byte-stream", NULL), NULL);
          
  g_signal_connect (mAppSrcData->app_source, "need-data", G_CALLBACK (start_feed),
      mAppSrcData);
  g_signal_connect (mAppSrcData->app_source, "enough-data", G_CALLBACK (stop_feed),
      mAppSrcData);
  /* End APP_Source */

  g_object_set (G_OBJECT (streammux), "batch-size", 1, NULL);

  g_object_set (G_OBJECT (streammux), "width", MUXER_OUTPUT_WIDTH, "height",
      MUXER_OUTPUT_HEIGHT,
      "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);

  /* Set all the necessary properties of the nvinfer element,
   * the necessary ones are : */
  g_object_set (G_OBJECT (pgie),
      "config-file-path", argv[2], NULL);

  /* we add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, mAppSrcData->loop);
  add_bus_watch_id = TRUE;
  gst_object_unref (bus);

  /* Set up the pipeline */
  /* we add all elements into the pipeline */
  gst_bin_add_many (GST_BIN (pipeline),
      mAppSrcData->app_source, h265parser, decoder, streammux, pgie,
      nvvidconv, nvosd, transform, sink, NULL);

  sinkpad = gst_element_get_request_pad (streammux, pad_name_sink);
  if (!sinkpad) {
    g_printerr ("Streammux request sink pad failed. Exiting.\n");
    goto stop_playback;
  }

  srcpad = gst_element_get_static_pad (decoder, pad_name_src);
  if (!srcpad) {
    g_printerr ("Decoder request src pad failed. Exiting.\n");
    goto stop_playback;
  }

  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
      g_printerr ("Failed to link decoder to stream muxer. Exiting.\n");
      goto stop_playback;
  }

  gst_object_unref (sinkpad);
  gst_object_unref (srcpad);

  /* we link the elements together */
  /* app-source -> h265-parser -> nvh265-decoder ->
   * nvinfer -> nvvidconv -> nvosd -> video-renderer */

  if (!gst_element_link_many (mAppSrcData->app_source, h265parser, decoder, NULL)) {
    g_printerr ("Elements could not be linked: 1. Exiting.\n");
    goto stop_playback;
  }

  if (!gst_element_link_many (streammux, pgie,
      nvvidconv, nvosd, transform, sink, NULL)) {
    g_printerr ("Elements could not be linked: 2. Exiting.\n");
    goto stop_playback;
  }

  /* Lets add probe to get informed of the meta data generated, we add probe to
   * the sink pad of the osd element, since by that time, the buffer would have
   * had got all the metadata. */
  osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
  if (!osd_sink_pad)
    g_print ("Unable to get sink pad\n");
  else
    gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
        osd_sink_pad_buffer_probe, NULL, NULL);
  gst_object_unref (osd_sink_pad);

  /* Set the pipeline to "playing" state */
  g_print ("Now playing: %s\n", argv[1]);
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print ("Running...\n");
  g_main_loop_run (mAppSrcData->loop);

stop_playback:
  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  if(add_bus_watch_id) {
    g_print ("Deleting bus watch\n");
    g_source_remove (bus_watch_id);
  }

disconnect_cam:
#ifdef IP_CAM_INPUT
  ip_cam_disconnect(camera);
#endif
  /* restart service if the network has any issue and it is not receive stop command from main controller */
  if(!forceStop)
    goto start_app;
out:
  g_main_loop_unref (mAppSrcData->loop);
  ip_cam_free_source(mAppSrcData);
  return 0;
}
