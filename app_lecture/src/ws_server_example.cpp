#include <stdio.h>
#include <server_ws.hpp>
#include <memory>


using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;

int main(int argc, char** argv)
{

  WsServer server;
  server.config.port = 8080;

  auto &echo = server.endpoint["^/echo/?$"];
  echo.on_message = [](std::shared_ptr<WsServer::Connection> connection, std::shared_ptr<WsServer::InMessage> in_message) {
    auto message_str = in_message->string();
    std::cout << "Server: Message received: \"" << message_str << "\" from " << connection.get() << std::endl;

    std::string out_message = "return hello!";
    connection->send(out_message);
  };

  echo.on_open = [](std::shared_ptr<WsServer::Connection> connection) {
    std::cout << "Server: Opened connection " << connection.get() << std::endl;
  };

  echo.on_close = [](std::shared_ptr<WsServer::Connection> connection, int status, const std::string & /*reason*/) {
    std::cout << "Server: Closed connection " << connection.get() << " with status code " << status << std::endl;
  };

  echo.on_error = [](std::shared_ptr<WsServer::Connection> connection, const SimpleWeb::error_code &ec) {
    std::cout << "Server: Error in connection " << connection.get() << ". "
         << "Error: " << ec << ", error message: " << ec.message() << std::endl;
  };

  server.start();
  return 0;
}