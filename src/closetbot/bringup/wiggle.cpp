#include <chrono>
#include <fp/all.hpp>
#include <closetbot/command.hpp>
#include <closetbot/closetbot.hpp>
#include <thread>

int main() {
  // auto serial_port = closetbot::make_serial_connection_to_robot();

  // if (!serial_port) {
  //   fmt::print("{}\n", serial_port.error());
  //   return -1;
  // }

  // auto closetbot = closetbot::Closetbot(std::move(serial_port.value()));

  // // power on
  // {
  //   auto const result = closetbot.send(closetbot::power_on());
  //   if (!result) {
  //     fmt::print("{}\n", result.error());
  //   }
  //   std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // }

  // // read joint angles
  // {
  //   auto const joint_angles = closetbot.get_radians();
  //   fmt::print("get_radians: {}\n", joint_angles);
  // }

  // // move to origin
  // {
  //   auto const result = closetbot.send_radians({0, 0, 0, 0, 0, 0}, 50);
  //   if (!result) {
  //     fmt::print("{}\n", result.error());
  //   }
  // }
  // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // // read joint angles
  // {
  //   auto const joint_angles = closetbot.get_radians();
  //   fmt::print("get_radians: {}\n", joint_angles);
  // }

  // // move a bit
  // {
  //   auto const result =
  //       closetbot.send_radians({0.2, -0.2, 0.2, 0.2, -0.2, -0.2}, 50);
  //   if (!result) {
  //     fmt::print("{}\n", result.error());
  //   }
  // }
  // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // // release all servos
  // {
  //   auto const result = closetbot.send(closetbot::release_all_servos());
  //   if (!result) {
  //     fmt::print("{}\n", result.error());
  //   }
  // }

  // // power off
  // {
  //   auto const result = closetbot.send(closetbot::power_off());
  //   if (!result) {
  //     fmt::print("{}\n", result.error());
  //   }
  // }

  return 0;
}
