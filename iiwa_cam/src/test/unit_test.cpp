

#include <iiwa.hpp>

void test1() {
  // cam::Frame frame(7);
  // auto cart = frame.get_cartesian_pos();

  // cart.push_back(1);
  // auto &in = cart;
  // cam::print_vec(in);
  // cam::print_vec(frame.get_joint_pos());

  // frame.set_joint_pos(in);

  // cam::print_vec(frame.get_joint_pos());

  // in.push_back(22);
  // cam::print_vec(frame.get_joint_pos());
}

void test2(int argc, char *argv[]) {
  ros::init(argc, argv, "cam_unit_test2");

  cam::Kuka kuka;

  geometry_msgs::Pose pose;
  pose.position.x = -0.52;
  pose.position.y = 0;
  pose.position.z = 0.15;
  pose.orientation.w = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;

  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();

  pose.position.z = 0.1;
  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();

  kuka.move_joint_ptp_drop(std::vector<double>{0, 0, 0, 0, 0, 0, 0});

  ros::spin();

  ros::shutdown();
}

void test3(int argc, char *argv[]) {
  ros::init(argc, argv, "cam_unit_test3");
  ros::NodeHandle nh;

    cam::Kuka kuka;
    kuka.get_recorded_frames();


  ros::spin();
  ros::shutdown();
}

void csv_reader_test(int argc, char *argv[]) {
  csv2::Reader<csv2::delimiter<','>, csv2::quote_character<'"'>,
               csv2::first_row_is_header<true>,
               csv2::trim_policy::trim_whitespace>
      csv;

  if (csv.mmap(argv[1])) {
    const auto header = csv.header();
    std::string head;
    header.read_raw_value(head);
    std::cout << head << std::endl;
    for (const auto row : csv) {
      for (const auto cell : row) {
        // Do something with cell value
        std::string value;
        cell.read_value(value);
        std::cout << value << " ";
      }
      std::cout << std::endl;
    }
  }
}

void csv_writer_test() {
  
  // std::ofstream stream("foo.csv");
  std::ofstream stream;
  stream.open("foo.csv",std::ios::out);
  
  csv2::Writer<csv2::delimiter<','>> writer(stream);

  std::vector<std::vector<std::string>> rows = {
      {"a", "b", "c"}, {"1", "2", "3"}, {"4", "5", "6"}};

  writer.write_rows(rows);
  // writer.write_rows(rows);

  stream.close();
}

#include <cstdio>
#include <thread>
void fun1() {
  for (int i = 0; i < 20; i++) printf("fun1: %d\n", i + 1);
}

void mt_test1(int argc, char *argv[]) {
  std::thread t1(fun1);
  for (int i = 0; i < 20; i++) printf("test1: %d\n", i + 1);
  t1.join();
}

int main(int argc, char *argv[]) {
  // test1();
  // test2(argc, argv);
  // test3(argc, argv);

  // if (argc > 1) csv_reader_test(argc, argv);
  csv_writer_test();



  // mt_test1(argc, argv);

  return 0;
}
