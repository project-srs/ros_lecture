/*
 *  Created on: Apr 16, 2018
 *      Author: Poom Pianpak
 */

#include "rosbridge_ws_client.hpp"
// #include <ncurses.h>

int main(int argc, char** argv)
{
  // initscr();

  // mvprintw(0,0,"hello");
  // while(true){
  //   int ch = getch();
  //   if(ch =='q')break;
  // }

  // endwin();

  // RosbridgeWsClient rbc("localhost:9090");

  // // sunscribe
  // rbc.addClient("topic_subscriber");
  // rbc.subscribe("topic_subscriber", "/ztopic",
  //               [](std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message) {
  //                 std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl;
  //               });

  // // publish
  // rbc.addClient("topic_advertiser");
  // rbc.advertise("topic_advertiser", "/ztopic", "std_msgs/String");

  // rapidjson::Document d;
  // d.SetObject();
  // d.AddMember("data", "Test message from /ztopic", d.GetAllocator());

  // for (int i = 0; i < 5; i++)
  // {
  //   rbc.publish("/ztopic", d);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // }

  // // close
  // rbc.removeClient("topic_advertiser");
  // rbc.removeClient("topic_subscriber");

  std::cout << "Program terminated" << std::endl;
}
