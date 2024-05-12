#include <QApplication>
#include "main_window.hpp"

int main(int argc, char * argv[])
{
  // Initialize client library
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Generate node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  QApplication app(argc, argv);
  auto main_window = std::make_shared<MainWindow>(options);
  main_window->show();

  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    app.processEvents();
    rclcpp::spin_some(main_window);
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}