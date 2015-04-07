#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gst/gst.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

using namespace cv;
VideoCapture *cam;

typedef struct gst_app {
  GstPipeline *pipeline;
  GstElement *src;
  GstElement *sink;
  GstElement *decoder;
  GstElement *ffmpeg;
  GstCaps *ccaps;
  GstElement *xvimagesink;
  GMainLoop *loop;
  guint sourceid;
  FILE *file;
} gst_app_t;

static gst_app_t gst_app;

#define BUFF_SIZE (1024)

void init_video() {
  cam->open(0); // get camera
  if (!cam->isOpened()) {
    printf("could not open %s\n","/dev/video0");
  }

  Size s = Size((int)cam->get(CV_CAP_PROP_FRAME_WIDTH),
                (int)cam->get(CV_CAP_PROP_FRAME_HEIGHT));
  int fps = (int)cam->get(CV_CAP_PROP_FPS);
  int ex = static_cast<int>(cam->get(CV_CAP_PROP_FOURCC));

  printf("opened video\n");
  printf("video specs: size: %d %d, fps: %d, ex: %d\n", s.width, s.height, fps, ex);
}

static gboolean bus_callback(GstBus *bus, GstMessage *message, gpointer *ptr) {
  gst_app_t *app = (gst_app_t *)ptr;

  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
      gchar *debug;
      GError *err;
      
      gst_message_parse_error(message, &err, &debug);
      g_print("Error: %s\n", err->message);
      g_error_free(err);
      g_free(debug);
      g_main_loop_quit(app->loop);
    }
    break;
    case GST_MESSAGE_WARNING: {
      gchar *debug;
      GError *err;
      gchar *name;

      gst_message_parse_warning(message, &err, &debug);
      g_print("Name of src %s\n", name ? name : "nil");
      g_error_free(err);
      g_free(debug);
    }
    break;
    case GST_MESSAGE_EOS: {
      g_print("End of Stream\n");
      g_main_loop_quit(app->loop);
    }
    break;
    case GST_MESSAGE_STATE_CHANGED: {
    }
    break;
    default: {
      g_print("got message %s\n", gst_message_type_get_name(GST_MESSAGE_TYPE(message)));
    }
    break;
  }
  return TRUE;
}

static void start_video_buffer(char const *fname) {
  // filesrc ! decodebin ! ffmpegcolorspace ! xvimagesink
  gst_app_t *app = &gst_app;
  GstBus *bus;
  gboolean link_ok;
  GstStateChangeReturn state_ret;

//  app->file = fopen(fname, "r");
//  g_assert(app->file);

  gst_init(NULL, NULL);

  // SET UP EVERYTHING
  app->pipeline = (GstPipeline *)gst_pipeline_new("mypipeline");
  bus = gst_pipeline_get_bus(app->pipeline);
  gst_bus_add_watch(bus, (GstBusFunc)bus_callback, app);
  gst_object_unref(bus);

  app->src = gst_element_factory_make("v4l2src", "mysrc");
  app->ccaps = gst_caps_new_simple("video/x-raw",
      "width", G_TYPE_INT, 640,
      "height", G_TYPE_INT, 480,
      NULL);
  app->xvimagesink = gst_element_factory_make("xvimagesink", "myvsink");

  g_assert(app->src);
  g_assert(app->ccaps);
  g_assert(app->xvimagesink);

  g_object_set(G_OBJECT(app->src), "device", "/dev/video0", NULL);

  // build pipeline

  gst_bin_add_many(GST_BIN(app->pipeline),
      app->src,
      app->xvimagesink,
      NULL);

  link_ok = gst_element_link_filtered(
      app->src,
      app->xvimagesink,
      app->ccaps);
  
  gst_caps_unref(app->ccaps);

  if (link_ok != TRUE) {
    g_printerr("Cannot link src and sink\n");
    return;
  }

  state_ret = gst_element_set_state((GstElement *)app->pipeline, GST_STATE_PLAYING);
  g_warning("set state returned %d\n", state_ret);

  // here is the loop
  app->loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(app->loop);

  state_ret = gst_element_set_state((GstElement *)app->pipeline, GST_STATE_NULL);
  g_warning("set state null returned %d\n", state_ret);

  return;
}

int main(int argc, char *argv[]) {
  printf("opening video source...\n");
  start_video_buffer("icarus.mp4");
  printf("done with program\n");
  return 0;
}
