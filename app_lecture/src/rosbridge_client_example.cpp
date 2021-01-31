#include <stdio.h>
#include <client_ws.hpp>
#include <memory>
#include <thread>

#include <ros/static_assert.h>
#include <json_transport/json_transport.hpp>

using WsClient = SimpleWeb::SocketClient<SimpleWeb::WS>;

class RosBrigeClientExample
{
public:
  RosBrigeClientExample(std::string target) : client_(target)
  {
    using namespace std::placeholders;
    client_.on_open = std::bind(&RosBrigeClientExample::onOpen, this, _1);
    client_.on_message = std::bind(&RosBrigeClientExample::onMessage, this, _1, _2);
    client_.on_close = std::bind(&RosBrigeClientExample::onClose, this, _1, _2, _3);
    client_.on_error = std::bind(&RosBrigeClientExample::onError, this, _1, _2);
    send_thread_ = std::thread(std::bind(&RosBrigeClientExample::sendThread, this));
    ws_thread_ = std::thread([&] { client_.start(); });
  }

private:
  void onOpen(std::shared_ptr<WsClient::Connection> connection)
  {
    std::cout << "Client: Opened connection" << std::endl;
    connection_ = connection;

    // advertise
    json_transport::json_t command;
    command["op"] = "advertise";
    command["topic"] = "/hello_from_client";
    command["type"] = "std_msgs/String";
    connection_->send(command.dump());

    // subscribe
    command.clear();
    command["op"] = "subscribe";
    command["topic"] = "/hello_from_ros";
    command["type"] = "std_msgs/String";
    connection_->send(command.dump());
  };

  void onMessage(std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message)
  {
    json_transport::json_t recv = json_transport::json_t::parse(in_message->string());
    if (recv["topic"].is_string() && recv["topic"] == "/hello_from_ros")
    {
      std::cout << "recv: " << recv["msg"]["data"] << std::endl;
    }
  };

  void onClose(std::shared_ptr<WsClient::Connection> connection, int status, const std::string& reason)
  {
    std::cout << "Client: Closed connection with status code " << status << std::endl;
  };

  void onError(std::shared_ptr<WsClient::Connection> connection, const SimpleWeb::error_code& ec)
  {
    std::cout << "Client: Error: " << ec << ", error message: " << ec.message() << std::endl;
  };

  void sendThread(void)
  {
    while (true)
    {
      if (connection_)
      {
        json_transport::json_t command, msg;
        command["op"] = "publish";
        command["topic"] = "/hello_from_client";
        msg["data"] = "hello client->ros";
        command["msg"] = msg;
        const std::string output = command.dump();
        connection_->send(output);
      }
      sleep(2);
    }
  }

  WsClient client_;
  std::shared_ptr<WsClient::Connection> connection_;
  std::thread send_thread_;
  std::thread ws_thread_;
};

int main(int argc, char** argv)
{
  printf("start\n");
  RosBrigeClientExample rosbridge_client_example("localhost:9090");
  while (true)
  {
    sleep(1);
  }
  return 0;
}