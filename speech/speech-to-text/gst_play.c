#include <gst/gst.h>

int main(int argc, char *argv[]) {
  GstElement *pipeline, *source, *sink;
  GstCaps *caps;
  GstBus *bus;
  GstMessage *msg;
  GstStateChangeReturn ret;

  // init
  gst_init(&argc, &argv);

  // create elements
  source = gst_element_factory_make("alsasrc", "audsrc");
  sink = gst_element_factory_make("alsasink", "audsink");

  // create capabilities
  caps = gst_caps_new_simple("audio/x-raw-int",
      "endianness", G_TYPE_INT, 1234,
      "signed", G_TYPE_BOOLEAN, TRUE,
      "width", G_TYPE_INT, 16,
      "height", G_TYPE_INT, 16,
      "rate", G_TYPE_INT, 16000,
      "channels", G_TYPE_INT, 1,
      "depth", G_TYPE_INT, 16,
      NULL);

  // create pipeline
  pipeline = gst_pipeline_new("audpipeline");
  if (!source || ! sink || !pipeline) {
    g_printerr("Not all elements created\n");
    return -1;
  }

  // build pipeline
  gst_bin_add_many(GST_BIN(pipeline), source, sink, NULL);
  if (gst_element_link_filtered(source, sink, caps) != TRUE) {
    g_printerr("Elements could not be linked.\n");
    gst_caps_unref(caps);
    gst_object_unref(pipeline);
    return -1;
  }
  gst_caps_unref(caps);

  // modify the source's properties
  //g_object_set(source, "pattern", 0, NULL);

  // start playing
  ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the pipeline to the playing state\n");
    gst_object_unref(pipeline);
    return -1;
  }

  // wait until error or EOS
  bus = gst_element_get_bus(pipeline);
  msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

  // parse message
  if (msg != NULL) {
    GError *err;
    gchar *debug_info;

    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR:
        gst_message_parse_error(msg, &err, &debug_info);
        g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
        g_clear_error(&err);
        g_free(debug_info);
        break;
      case GST_MESSAGE_EOS:
        g_print("End-Of-Stream reached\n");
        break;
      default:
        g_printerr("Unexpected message received\n");
        break;
    }
    gst_message_unref(msg);
  }

  // free
  gst_object_unref(bus);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  return 0;
}
