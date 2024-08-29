#include "vicon_unlabeled_marker_receiver/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

namespace ViconReceiver {
namespace UnlabeledMarker {

Communicator::Communicator() : Node("vicon_unlabeled_markers") {
  // get parameters
  this->declare_parameter<std::string>("hostname", "127.0.0.1");
  this->declare_parameter<int>("buffer_size", 200);
  this->declare_parameter<std::string>("namespace", "vicon_unlabeled_markers");
  this->get_parameter("hostname", hostname);
  this->get_parameter("buffer_size", buffer_size);
  this->get_parameter("namespace", ns_name);
}

bool Communicator::connect() {
  // connect to server
  string msg = "Connecting to " + hostname + " ...";
  cout << msg << endl;
  int counter = 0;
  while (!vicon_client.IsConnected().Connected) {
    bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
    if (!ok) {
      counter++;
      msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
      cout << msg << endl;
      sleep(1);
    }
  }
  msg = "Connection successfully established with " + hostname;
  cout << msg << endl;

  // perform further initialization
  // vicon_client.EnableSegmentData();
  // vicon_client.EnableMarkerData();
  vicon_client.EnableUnlabeledMarkerData();
  // vicon_client.EnableMarkerRayData();
  vicon_client.EnableDeviceData();
  vicon_client.EnableDebugData();

  vicon_client.SetStreamMode(StreamMode::ClientPull);
  vicon_client.SetBufferSize(buffer_size);

  msg = "Initialization complete";
  cout << msg << endl;

  return true;
}

bool Communicator::disconnect() {
  if (!vicon_client.IsConnected().Connected)
    return true;
  sleep(1);
  vicon_client.DisableSegmentData();
  vicon_client.DisableMarkerData();
  vicon_client.DisableUnlabeledMarkerData();
  vicon_client.DisableDeviceData();
  vicon_client.DisableCentroidData();
  string msg = "Disconnecting from " + hostname + "...";
  cout << msg << endl;
  vicon_client.Disconnect();
  msg = "Successfully disconnected";
  cout << msg << endl;
  if (!vicon_client.IsConnected().Connected)
    return true;
  return false;
}

void Communicator::get_frame() {
  if (!vicon_client.IsConnected().Connected) {
    cout << "Not connected to server" << endl;
    // wait 1 sec to try next
    sleep(1);
    return;
  }

  vicon_client.GetFrame();
  Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();
  std::size_t marker_count = vicon_client.GetUnlabeledMarkerCount().MarkerCount;

  MarkersStruct current_markers(marker_count, frame_number.FrameNumber);
  for (std::size_t i = 0; i < marker_count; i++) {
    auto translation =
        vicon_client.GetUnlabeledMarkerGlobalTranslation(i).Translation;
    current_markers.x[i] = translation[0];
    current_markers.y[i] = translation[1];
    current_markers.z[i] = translation[2];
    current_markers.indices[i] = i;
  }

  // send position to publisher
  map<string, Publisher>::iterator pub_it;
  boost::mutex::scoped_try_lock lock(mutex);
  const string subject_name = "unlabeled_markers";
  const string segment_name = "positions";

  if (lock.owns_lock()) {
    // get publisher
    pub_it = pub_map.find(subject_name + "/" + segment_name);
    if (pub_it != pub_map.end()) {
      Publisher &pub = pub_it->second;

      if (pub.is_ready) {
        pub.publish(current_markers);
      }
    } else {
      // create publisher if not already available
      lock.unlock();
      create_publisher(subject_name, segment_name);
    }
  }
}

void Communicator::create_publisher(const string subject_name,
                                    const string segment_name) {
  boost::thread(&Communicator::create_publisher_thread, this, subject_name,
                segment_name);
}

void Communicator::create_publisher_thread(const string subject_name,
                                           const string segment_name) {
  std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
  std::string key = subject_name + "/" + segment_name;

  string msg = "Creating publisher for segment " + segment_name +
               " from subject " + subject_name;
  cout << msg << endl;

  // create publisher
  boost::mutex::scoped_lock lock(mutex);
  pub_map.insert(std::map<std::string, Publisher>::value_type(
      key, Publisher(topic_name, this)));

  // we don't need the lock anymore, since rest is protected by is_ready
  lock.unlock();
}

} // namespace UnlabeledMarker
} // namespace ViconReceiver

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconReceiver::UnlabeledMarker::Communicator>();
  node->connect();

  while (rclcpp::ok()) {
    node->get_frame();
  }

  node->disconnect();
  rclcpp::shutdown();
  return 0;
}