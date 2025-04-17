/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

int main(int argc, char **argv)
{
  unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
  unitree::robot::go2::ObstaclesAvoidClient sc;

  sc.SetTimeout(5.0f);
  sc.Init();
  sc.SwitchSet(false);//Turn on obstacle avoidance
  usleep(1000000);
  sc.UseRemoteCommandFromApi(false);//Seize the speed control of the remote control
  // sc.MoveToIncrementPosition(0.0,0.0,0.0);

  while (1)
  {
    sleep(10);
  }
  return 0;
}
