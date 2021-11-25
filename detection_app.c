#include <gst/gst.h>
#include <time.h>
#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <cuda_runtime_api.h>

#include "gstnvdsmeta.h"

#ifndef PLATFORM_TEGRA
#include "gst-nvmessage.h"
#endif

#define MAX_DISPLAY_LEN 64
#define BATCH_SIZE 4

#define PGIE_CLASS_ID_FALL 0
#define PGIE_CLASS_ID_STAND 1
/*Please define differet configs path.
 */
#define FIRSTCONFIG "configs/config_infer_1_yoloV5.txt"
#define SECONDCONFIG "configs/config_infer_2_yoloV5.txt"
#define THIRDCONFIG "configs/config_infer_3_yoloV5.txt"
#define TRACKER_CONFIG_FILE "configs/dstest2_tracker_config.txt"
#define MAX_TRACKING_ID_LEN 16

/*Please define drop-frame-interval.*/
#define DFI 0  //FROM 1 TO 30

/*Please define the buffer size of queue*/
#define BUFFER_SIZE 80000

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 800
#define MUXER_OUTPUT_HEIGHT 450
#define BUFFER_POOL_SIZE 4
#define LIVE_SOURCE 1

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

/* NVIDIA Decoder source pad memory feature. This feature signifies that source
 * pads having this capability will push GstBuffers containing cuda buffers. */
#define GST_CAPS_FEATURES_NVMM "memory:NVMM"

#define DISPLAY_ON 1 //1 for on, 0 for off
#define FILE_PATH "./out.h264"


/* By default, OSD process-mode is set to CPU_MODE. To change mode, set as:
 * 1: GPU mode (for Tesla only)
 * 2: HW mode (For Jetson only)
 */
/* 0 = CPU  1 = GPU I'd like to use CPU to deal with OSD group. */
#define OSD_PROCESS_MODE 0

/* By default, OSD will not display text. To display text, change this to 1 */
#define OSD_DISPLAY_TEXT 0

#define TILED_OUTPUT_WIDTH 1600
#define TILED_OUTPUT_HEIGHT 450

/* global vars for batch and time cost */
gint total_batch_no = 1;
static clock_t start_time = 0;
static clock_t cost = 0;

/* hook function to get the time cost */
static GstPadProbeReturn
tiler_src_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
    gpointer u_data)
{
  GstBuffer *buf = (GstBuffer *) info->data;
  guint num_rects = 0; 
  NvDsObjectMeta *obj_meta = NULL;
  guint stand_count = 0;
  guint fall_count = 0;
  guint obj_num = 1;
  float left = 0;
  float top = 0;
  float width = 0;
  float height = 0;
  NvDsMetaList * l_frame = NULL;
  NvDsMetaList * l_obj = NULL;   
  NvDsComp_BboxInfo bbox; 

  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

  gchar* label;
  FILE* pf;

  if(start_time==0)
    start_time = clock();

  if (total_batch_no%1 == 0){

    cost = (clock() - start_time);
    cost = cost*1000/CLOCKS_PER_SEC;
    g_print("batch no: %d, time cost per batch: %lds\n", total_batch_no, cost);
    start_time = clock();

  }
  total_batch_no++;

  // for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;l_frame = l_frame->next) 
  // {
  //   NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);

    // obj_num = 1;
    // pf = fopen("result.txt","a+");

    // // check every object in a frame
    // for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;l_obj = l_obj->next) 
    // { 
    //   obj_meta = (NvDsObjectMeta *) (l_obj->data);
      
    //   // get bbox properties
    //   bbox = obj_meta->detector_bbox_info;
    //   left = bbox.org_bbox_coords.left;
    //   top = bbox.org_bbox_coords.top;
    //   width = bbox.org_bbox_coords.width;
    //   height = bbox.org_bbox_coords.height;
    //   label = obj_meta->obj_label;

    //   g_print ("Source ID = %d Frame Number = %d object_num = %d letf = %f top = %f "
    //         "width = %f height = %f\n Object label = %s\n", frame_meta->source_id,
    //         frame_meta->frame_num, obj_num, left, top, width, height, label);

    //   // write result into the result.txt file
    //   fprintf(pf,"Source ID = %d Frame Number = %d object_num = %d letf = %f top = %f "
    //         "width = %f height = %f\n Object label = %s\n", frame_meta->source_id,
    //         frame_meta->frame_num, obj_num, left, top, width, height, label);

    //   obj_num++;

    //   obj_meta = (NvDsObjectMeta *) (l_obj->data);
    //   if (obj_meta->class_id == PGIE_CLASS_ID_FALL) {
    //       fall_count++;
    //       num_rects++;
    //   }
    //   if (obj_meta->class_id == PGIE_CLASS_ID_STAND) {
    //       stand_count++;
    //       num_rects++;
    //   }
    // }
    // fclose(pf);

    // g_print ("Source ID = %d Frame Number = %d Number of objects = %d "
    //         "Stand Count = %d Fall Count = %d\n", frame_meta->source_id,
    //         frame_meta->frame_num, num_rects, stand_count, fall_count);
  }
  return GST_PAD_PROBE_OK;
}


static gboolean
bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_WARNING:
    {
      gchar *debug;
      GError *error;
      gst_message_parse_warning (msg, &error, &debug);
      g_printerr ("WARNING from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), error->message);
      g_free (debug);
      g_printerr ("Warning: %s\n", error->message);
      g_error_free (error);
      break;
    }
    case GST_MESSAGE_ERROR:
    {
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
#ifndef PLATFORM_TEGRA
    case GST_MESSAGE_ELEMENT:
    {
      if (gst_nvmessage_is_stream_eos (msg)) {
        guint stream_id;
        if (gst_nvmessage_parse_stream_eos (msg, &stream_id)) {
          g_print ("Got EOS from stream %d\n", stream_id);
        }
      }
      break;
    }
#endif
    default:
      break;
  }
  return TRUE;
}

static void
cb_newpad (GstElement * decodebin, GstPad * decoder_src_pad, gpointer data)
{
  g_print ("In cb_newpad\n");
  GstCaps *caps = gst_pad_get_current_caps (decoder_src_pad);
  const GstStructure *str = gst_caps_get_structure (caps, 0);
  const gchar *name = gst_structure_get_name (str);
  GstElement *source_bin = (GstElement *) data;
  GstCapsFeatures *features = gst_caps_get_features (caps, 0);

  /* Need to check if the pad created by the decodebin is for video and not
   * audio. */
  if (!strncmp (name, "video", 5)) {
    /* Link the decodebin pad only if decodebin has picked nvidia
     * decoder plugin nvdec_*. We do this by checking if the pad caps contain
     * NVMM memory features. */
    if (gst_caps_features_contains (features, GST_CAPS_FEATURES_NVMM)) {
      /* Get the source bin ghost pad */
      GstPad *bin_ghost_pad = gst_element_get_static_pad (source_bin, "src");
      if (!gst_ghost_pad_set_target (GST_GHOST_PAD (bin_ghost_pad),
              decoder_src_pad)) {
        g_printerr ("Failed to link decoder src pad to source bin ghost pad\n");
      }
      gst_object_unref (bin_ghost_pad);
    } else {
      g_printerr ("Error: Decodebin did not pick nvidia decoder plugin.\n");
    }
  }
}

static void
decodebin_child_added (GstChildProxy * child_proxy, GObject * object,
    gchar * name, gpointer user_data)
{
  g_print ("Decodebin child added: %s\n", name);
  if (g_strrstr (name, "decodebin") == name) {
    g_signal_connect (G_OBJECT (object), "child-added",
        G_CALLBACK (decodebin_child_added), user_data);
  }
  if (g_strstr_len (name, -1, "nvv4l2decoder") == name) {
    //g_print ("Seting bufapi_version: %s\n", name);
    //g_object_set (object, "bufapi-version", TRUE, NULL);
    gboolean src_res = g_str_has_suffix(name, "0");
    
      g_object_set(object, "drop-frame-interval", DFI, "skip-frames", 0, NULL);
      g_print ("\ndrop-frame-interval: %d\n", DFI);   
    
  }
}

static GstElement *
create_source_bin (guint index, gchar * uri)
{
  GstElement *bin = NULL, *uri_decode_bin = NULL;
  gchar bin_name[16] = { };

  g_snprintf (bin_name, 15, "source-bin-%02d", index);
  /* Create a source GstBin to abstract this bin's content from the rest of the
   * pipeline */
  bin = gst_bin_new (bin_name);

  /* Source element for reading from the uri.
   * We will use decodebin and let it figure out the container format of the
   * stream and the codec and plug the appropriate demux and decode plugins. */
  uri_decode_bin = gst_element_factory_make ("uridecodebin", "uri-decode-bin");

  if (!bin || !uri_decode_bin) {
    g_printerr ("One element in source bin could not be created.\n");
    return NULL;
  }

  /* We set the input uri to the source element */
  g_object_set (G_OBJECT (uri_decode_bin), "uri", uri, NULL);

  /* Connect to the "pad-added" signal of the decodebin which generates a
   * callback once a new pad for raw data has beed created by the decodebin */
  g_signal_connect (G_OBJECT (uri_decode_bin), "pad-added",
      G_CALLBACK (cb_newpad), bin);
  g_signal_connect (G_OBJECT (uri_decode_bin), "child-added",
      G_CALLBACK (decodebin_child_added), bin);

  gst_bin_add (GST_BIN (bin), uri_decode_bin);

  /* We need to create a ghost pad for the source bin which will act as a proxy
   * for the video decoder src pad. The ghost pad will not have a target right
   * now. Once the decode bin creates the video decoder and generates the
   * cb_newpad callback, we will set the ghost pad target to the video decoder
   * src pad. */
  if (!gst_element_add_pad (bin, gst_ghost_pad_new_no_target ("src",
              GST_PAD_SRC))) {
    g_printerr ("Failed to add ghost pad in source bin\n");
    return NULL;
  }

  return bin;
}


/* Tracker config parsing */

#define CHECK_ERROR(error) \
    if (error) { \
        g_printerr ("Error while parsing config file: %s\n", error->message); \
        goto done; \
    }

#define CONFIG_GROUP_TRACKER "tracker"
#define CONFIG_GROUP_TRACKER_WIDTH "tracker-width"
#define CONFIG_GROUP_TRACKER_HEIGHT "tracker-height"
#define CONFIG_GROUP_TRACKER_LL_CONFIG_FILE "ll-config-file"
#define CONFIG_GROUP_TRACKER_LL_LIB_FILE "ll-lib-file"
#define CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS "enable-batch-process"
#define CONFIG_GPU_ID "gpu-id"

static gchar *
get_absolute_file_path (gchar *cfg_file_path, gchar *file_path)
{
  gchar abs_cfg_path[PATH_MAX + 1];
  gchar *abs_file_path;
  gchar *delim;

  if (file_path && file_path[0] == '/') {
    return file_path;
  }

  if (!realpath (cfg_file_path, abs_cfg_path)) {
    g_free (file_path);
    return NULL;
  }

  // Return absolute path of config file if file_path is NULL.
  if (!file_path) {
    abs_file_path = g_strdup (abs_cfg_path);
    return abs_file_path;
  }

  delim = g_strrstr (abs_cfg_path, "/");
  *(delim + 1) = '\0';

  abs_file_path = g_strconcat (abs_cfg_path, file_path, NULL);
  g_free (file_path);

  return abs_file_path;
}

static gboolean
set_tracker_properties (GstElement *nvtracker)
{
  gboolean ret = FALSE;
  GError *error = NULL;
  gchar **keys = NULL;
  gchar **key = NULL;
  GKeyFile *key_file = g_key_file_new ();

  if (!g_key_file_load_from_file (key_file, TRACKER_CONFIG_FILE, G_KEY_FILE_NONE,
          &error)) {
    g_printerr ("Failed to load config file: %s\n", error->message);
    return FALSE;
  }

  keys = g_key_file_get_keys (key_file, CONFIG_GROUP_TRACKER, NULL, &error);
  CHECK_ERROR (error);

  for (key = keys; *key; key++) {
    if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_WIDTH)) {
      gint width =
          g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
          CONFIG_GROUP_TRACKER_WIDTH, &error);
      CHECK_ERROR (error);
      g_object_set (G_OBJECT (nvtracker), "tracker-width", width, NULL);
    } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_HEIGHT)) {
      gint height =
          g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
          CONFIG_GROUP_TRACKER_HEIGHT, &error);
      CHECK_ERROR (error);
      g_object_set (G_OBJECT (nvtracker), "tracker-height", height, NULL);
    } else if (!g_strcmp0 (*key, CONFIG_GPU_ID)) {
      guint gpu_id =
          g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
          CONFIG_GPU_ID, &error);
      CHECK_ERROR (error);
      g_object_set (G_OBJECT (nvtracker), "gpu_id", gpu_id, NULL);
    } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_LL_CONFIG_FILE)) {
      char* ll_config_file = get_absolute_file_path (TRACKER_CONFIG_FILE,
                g_key_file_get_string (key_file,
                    CONFIG_GROUP_TRACKER,
                    CONFIG_GROUP_TRACKER_LL_CONFIG_FILE, &error));
      CHECK_ERROR (error);
      g_object_set (G_OBJECT (nvtracker), "ll-config-file", ll_config_file, NULL);
    } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_LL_LIB_FILE)) {
      char* ll_lib_file = get_absolute_file_path (TRACKER_CONFIG_FILE,
                g_key_file_get_string (key_file,
                    CONFIG_GROUP_TRACKER,
                    CONFIG_GROUP_TRACKER_LL_LIB_FILE, &error));
      CHECK_ERROR (error);
      g_object_set (G_OBJECT (nvtracker), "ll-lib-file", ll_lib_file, NULL);
    } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS)) {
      gboolean enable_batch_process =
          g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
          CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS, &error);
      CHECK_ERROR (error);
      g_object_set (G_OBJECT (nvtracker), "enable_batch_process",
                    enable_batch_process, NULL);
    } else {
      g_printerr ("Unknown key '%s' for group [%s]", *key,
          CONFIG_GROUP_TRACKER);
    }
  }

  ret = TRUE;
done:
  if (error) {
    g_error_free (error);
  }
  if (keys) {
    g_strfreev (keys);
  }
  if (!ret) {
    g_printerr ("%s failed", __func__);
  }
  return ret;
}

/* Build the deepstream pipeline */ 
int
main (int argc, char *argv[])
{
  GMainLoop *loop = NULL;

  GstElement *pipeline = NULL, *streammux = NULL, *video_sink = NULL, *file_sink = NULL, *tee = NULL,
      *first_detector = NULL, *second_detector = NULL, *third_detector = NULL,*tracker = NULL,
      *queue_input, *queue_file, *queue_video, 
      *nvvideoconvert = NULL, *nvtransform = NULL, *nvosd = NULL, *enc = NULL,
      *tiler = NULL;

  GstBus *bus = NULL;
  guint bus_watch_id;
  GstPad *tiler_src_pad = NULL;
  guint i, num_sources;
  guint tiler_rows, tiler_columns;
  guint pgie_batch_size;

  int current_device = -1;
  cudaGetDevice(&current_device);
  struct cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, current_device);

  /* Check input arguments */
  if (argc < 2) {
    g_printerr ("Usage: %s <uri1> [uri2] ... [uriN] \n", argv[0]);
    return -1;
  }
  num_sources = argc - 1;

  /* Standard GStreamer initialization */
  gst_init (&argc, &argv);
  loop = g_main_loop_new (NULL, FALSE);

  /* Create gstreamer elements */
  /* Create Pipeline element that will form a connection of other elements */
  pipeline = gst_pipeline_new ("detection-pipeline");

  /* Create nvstreammux instance to form batches from one or more sources. */
  streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");

  if (!pipeline || !streammux) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }
  gst_bin_add (GST_BIN (pipeline), streammux);

  for (i = 0; i < num_sources; i++) {
    GstPad *sinkpad, *srcpad;
    gchar pad_name[16] = { };
    GstElement *source_bin = create_source_bin (i, argv[i + 1]);

    if (!source_bin) {
      g_printerr ("Failed to create source bin. Exiting.\n");
      return -1;
    }

    gst_bin_add (GST_BIN (pipeline), source_bin);

    g_snprintf (pad_name, 15, "sink_%u", i);
    sinkpad = gst_element_get_request_pad (streammux, pad_name);
    if (!sinkpad) {
      g_printerr ("Streammux request sink pad failed. Exiting.\n");
      return -1;
    }

    srcpad = gst_element_get_static_pad (source_bin, "src");
    if (!srcpad) {
      g_printerr ("Failed to get src pad of source bin. Exiting.\n");
      return -1;
    }

    if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
      g_printerr ("Failed to link source bin to stream muxer. Exiting.\n");
      return -1;
    }
    gst_object_unref (srcpad);
    gst_object_unref (sinkpad);
  }

  /* Use nvinfer to infer on batched frame. */
  first_detector = gst_element_factory_make ("nvinfer", "primary-nvinference-engine1");
  second_detector = gst_element_factory_make ("nvinfer", "primary-nvinference-engine2");
  third_detector = gst_element_factory_make ("nvinfer", "primary-nvinference-engine3");
  tracker = gst_element_factory_make("nvtracker","tracker");


  /* Add queue elements between every two elements */
  queue_input = gst_element_factory_make ("queue", "queue_input");
  queue_file = gst_element_factory_make ("queue", "queue_file");
  queue_video = gst_element_factory_make ("queue", "queue_video");

  g_object_set (G_OBJECT (queue_input), "max-size-buffers", BUFFER_SIZE, NULL);
  g_object_set (G_OBJECT (queue_file), "max-size-buffers", BUFFER_SIZE, NULL);
  g_object_set (G_OBJECT (queue_video), "max-size-buffers", BUFFER_SIZE, NULL);
 
  tee = gst_element_factory_make ("tee","tee");

  file_sink = gst_element_factory_make ("filesink", "file-sink");
  enc = gst_element_factory_make ("nvv4l2h264enc", "h264-enc");
  g_object_set (G_OBJECT (file_sink), "location", FILE_PATH, NULL);


  if (DISPLAY_ON==1) {

    /* Use nvtiler to composite the batched frames into a 2D tiled array based
    * on the source of the frames. */
    tiler = gst_element_factory_make ("nvmultistreamtiler", "nvtiler");

    /* Use convertor to convert from NV12 to RGBA as required by nvosd */
    nvvideoconvert = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");
    nvtransform = gst_element_factory_make ("nvegltransform", "nvegltransform");

    /* Create OSD to draw on the converted RGBA buffer */
    nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");

    /* Finally render the osd output */
    video_sink = gst_element_factory_make ("nveglglessink", "nveglglessink");

    if (!first_detector || !second_detector || !third_detector || !tiler|| !nvvideoconvert || !nvtransform|| !nvosd|| !video_sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;

    /* we set the tiler properties here */
    tiler_rows = (guint) sqrt (num_sources);
    tiler_columns = (guint) ceil (1.0 * num_sources / tiler_rows);
    g_object_set (G_OBJECT (tiler), "rows", tiler_rows, "columns", tiler_columns,
        "width", TILED_OUTPUT_WIDTH, "height", TILED_OUTPUT_HEIGHT, NULL);

    g_object_set (G_OBJECT(nvosd), "process-mode", 2, NULL);
    }
  }
  else{
    
    /* Finally fakesin for output */
    video_sink = gst_element_factory_make ("fakesink", "nvvideo-renderer");

    if (!first_detector || !second_detector || !third_detector || !video_sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
    }
  }

  if(!prop.integrated) {
    g_printerr ("One tegra element could not be created. Exiting.\n");
    return -1;
  }

  
  g_object_set (G_OBJECT (streammux), "batch-size", BATCH_SIZE, NULL);

  g_object_set (G_OBJECT (streammux), "width", MUXER_OUTPUT_WIDTH, "height",
      MUXER_OUTPUT_HEIGHT,
      "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC,
      "live-source", LIVE_SOURCE, 
      "buffer-pool-size", BUFFER_POOL_SIZE,
      "enable-padding", 1,
       NULL);

  /* Configure the nvinfer element using the nvinfer config file. */
  g_object_set (G_OBJECT (first_detector), "config-file-path", FIRSTCONFIG, "unique-id", 1, "process-mode", 1, NULL);
  g_object_set (G_OBJECT (second_detector), "config-file-path", SECONDCONFIG, "unique-id", 1, "process-mode", 1, NULL);
  g_object_set (G_OBJECT (third_detector), "config-file-path", THIRDCONFIG, "unique-id", 1, "process-mode", 1, NULL);

  /* Override the batch-size set in the config file with the number of sources. */
  g_object_set (G_OBJECT (first_detector), "batch-size", BATCH_SIZE, NULL);
  g_object_set (G_OBJECT (second_detector), "batch-size", BATCH_SIZE, NULL);
  g_object_set (G_OBJECT (third_detector), "batch-size", BATCH_SIZE, NULL);

  g_object_get (G_OBJECT (first_detector), "batch-size", &pgie_batch_size, NULL);
  g_print("the input sourece num is %d, the detector batch size: %d.\n", num_sources ,pgie_batch_size);

  g_object_set (G_OBJECT (video_sink), "qos", 0, "sync", FALSE, NULL);
  g_object_set (G_OBJECT (file_sink), "qos", 0, "sync", FALSE, NULL);


    /* Set necessary properties of the tracker element. */
  if (!set_tracker_properties(tracker)) {
    g_printerr ("Failed to set tracker properties. Exiting.\n");
    return -1;
  }

  /* we add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  /* Set up the pipeline */
  /* we add all elements into the pipeline */
  

  if(DISPLAY_ON==1){
    gst_bin_add_many (GST_BIN (pipeline), 
      queue_input,
      first_detector, 
      second_detector, 
      third_detector, 
      tracker,
      tiler, nvvideoconvert, nvosd,
      queue_video, nvtransform, video_sink, 
      NULL);

    /* we link the elements together
    * nvstreammux -> nvinfer -> nvtiler -> nvvidconv -> nvosd -> video-renderer */
    if (!gst_element_link_many (streammux, queue_input,
          first_detector, 
          second_detector, 
          third_detector, 
          tracker,
          tiler, nvvideoconvert, nvosd, queue_video, nvtransform, video_sink,
          NULL)) {
      g_printerr ("Source and inference elements could not be linked. Exiting.\n");
      return -1;
    }

  }else{
    gst_bin_add_many (GST_BIN (pipeline), queue_input,
      first_detector, second_detector, third_detector,
      video_sink, NULL);
    /* we link the elements together
    * nvstreammux -> nvinfer -> fake_sink */
    if (!gst_element_link_many (streammux, queue_input,
          first_detector, second_detector, third_detector,
          video_sink, NULL)) {
      g_printerr ("Elements could not be linked. Exiting.\n");
      return -1;
    }
  }

  
  /* Lets add probe to get informed of the meta data generated, we add probe to
   * the sink pad of the osd element, since by that time, the buffer would have
   * had got all the metadata. */
  tiler_src_pad = gst_element_get_static_pad (third_detector, "src");
  if (!tiler_src_pad)
    g_print ("Unable to get src pad\n");
  else
    gst_pad_add_probe (tiler_src_pad, GST_PAD_PROBE_TYPE_BUFFER,
        tiler_src_pad_buffer_probe, NULL, NULL);
  gst_object_unref (tiler_src_pad);

  /* Set the pipeline to "playing" state */
  g_print ("Now playing:");
  for (i = 0; i < num_sources; i++) {
    g_print (" %s,", argv[i + 1]);
  }
  g_print ("\n");
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print ("Running...\n");
  g_main_loop_run (loop);

  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  g_source_remove (bus_watch_id);
  g_main_loop_unref (loop);
  return 0;
}
