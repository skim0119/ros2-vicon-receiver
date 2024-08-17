#include "vicon_receiver_mock/communicator.hpp"

Communicator::Communicator() : Node("vicon") {
  // get parameters
  this->declare_parameter<int>("buffer_size", 200);
  this->declare_parameter<std::string>("namespace", "vicon_mock");
  this->get_parameter("buffer_size", buffer_size);
  this->get_parameter("namespace", ns_name);
}

bool Communicator::connect() {
  // connect to server
  string msg = "Connecting to vicon-mock ...";
  cout << msg << endl;
  msg = "Connection successfully established with vicon-mock";
  cout << msg << endl;
  msg = "Initialization complete";
  cout << msg << endl;

  return true;
}

bool Communicator::disconnect() {
  sleep(1);
  string msg = "Disconnecting from vicon-mock";
  cout << msg << endl;
  msg = "Successfully disconnected";
  cout << msg << endl;
  return true;
}

void Communicator::get_frame() {

  unsigned int subject_count = 5UL;

  map<string, Publisher>::iterator pub_it;

  for (unsigned int subject_index = 0; subject_index < subject_count;
       ++subject_index) {
    // get the subject name
    string subject_name = "TestSubject_" + std::to_string(subject_index);

    // count the number of segments
    unsigned int segment_count = 3UL;

    for (unsigned int segment_index = 0; segment_index < segment_count;
         ++segment_index) {
      // get the segment name
      string segment_name = "TestSegment_" + std::to_string(subject_index) + "_" + std::to_string(segment_index);

      // get position of segment
      PositionStruct current_position;

      for (size_t i = 0; i < 4; i++) {
        if (i < 3)
          current_position.translation[i] = 0.0;
        current_position.rotation[i] = 10.0;
      }
      current_position.segment_name = segment_name;
      current_position.subject_name = subject_name;
      current_position.translation_type = "Global";
      current_position.frame_number = 0UL;

      // send position to publisher
      boost::mutex::scoped_try_lock lock(mutex);

      if (lock.owns_lock()) {
        // get publisher
        pub_it = pub_map.find(subject_name + "/" + segment_name);
        if (pub_it != pub_map.end()) {
          Publisher &pub = pub_it->second;

          if (pub.is_ready) {
            pub.publish(current_position);
          }
        } else {
          // create publisher if not already available
          lock.unlock();
          create_publisher(subject_name, segment_name);
        }
      }
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Communicator>();
  node->connect();

  while (rclcpp::ok()) {
    node->get_frame();
  }

  node->disconnect();
  rclcpp::shutdown();
  return 0;
}
