#include <stdio.h>
#include <client_ws.hpp>
#include <memory>

using WsClient = SimpleWeb::SocketClient<SimpleWeb::WS>;

int main(int argc, char** argv)
{
  WsClient client("localhost:8080/echo");

  client.on_message = [](std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message) {
    std::cout << "Client: Message received: \"" << in_message->string() << "\"" << std::endl;
  };

  client.on_open = [](std::shared_ptr<WsClient::Connection> connection) {
    std::cout << "Client: Opened connection" << std::endl;

    const std::string send_msg = "Hello server!";
    connection->send(send_msg);
  };

  client.on_close = [](std::shared_ptr<WsClient::Connection> /*connection*/, int status,
                       const std::string& /*reason*/) {
    std::cout << "Client: Closed connection with status code " << status << std::endl;
  };

  client.on_error = [](std::shared_ptr<WsClient::Connection> /*connection*/, const SimpleWeb::error_code& ec) {
    std::cout << "Client: Error: " << ec << ", error message: " << ec.message() << std::endl;
  };

  client.start();
  return 0;
}