#include <cmath>
#include <thread>
#include "dxl_test/xl330_driver.hpp"

using namespace std::chrono_literals;

// nonfunctional example
int main()
{
  const int dxl_id_1 = 10;
  const int dxl_id_2 = 20;

  auto driver = XL330Driver("/dev/ttyACM0");
  {
    const auto init_return = driver.init();
    if (init_return != 0)
    {
      return init_return;
    }
  }

  std::this_thread::sleep_for(1s);

  // test velocity control
  driver.activateWithVelocityMode(dxl_id_1);
  driver.activateWithVelocityMode(dxl_id_2);

  driver.setTargetVelocityRadianPerSec(dxl_id_1, M_PI);
  driver.setTargetVelocityRadianPerSec(dxl_id_2, -M_PI);

  std::this_thread::sleep_for(5s);

  std::cout << "Velocity 1: " << driver.getVelocityRadianPerSec(dxl_id_1) << std::endl;
  std::cout << "Velocity 2: " << driver.getVelocityRadianPerSec(dxl_id_2) << std::endl;

  driver.deactivate(dxl_id_1);
  driver.deactivate(dxl_id_2);

  // test position control
  driver.activateWithPositionMode(dxl_id_1);
  driver.activateWithPositionMode(dxl_id_2);

  driver.setTargetPositionRadian(dxl_id_1, M_PI_2);
  driver.setTargetPositionRadian(dxl_id_2, -M_PI_2);

  std::this_thread::sleep_for(5s);

  std::cout << "Position 1: " << driver.getPositionRadian(dxl_id_1) << std::endl;
  std::cout << "Position 2: " << driver.getPositionRadian(dxl_id_2) << std::endl;

  driver.deactivate(dxl_id_1);
  driver.deactivate(dxl_id_2);

  return 0;
}