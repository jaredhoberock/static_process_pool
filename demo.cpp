#include "process.hpp"
#include "socket.hpp"
#include "file_descriptor_stream.hpp"
#include "active_message.hpp"
#include <system_error>

// XXX next step: make a single server static_process_pool

void server_function()
{
  int port = 71342;

  read_socket socket(port);
  file_descriptor_istream is(socket.release());
  
  int i = 0;
  while(!is.eof())
  {
    active_message message;

    {
      input_archive ar(is);

      ar(message);
    }

    message.activate();
  }
}

void hello()
{
  std::cout << "hello world from process " << this_process::get_id() << std::endl;
}

void goodbye()
{
  std::cout << "goodbye world from process " << this_process::get_id() << std::endl;
}

int main()
{
  int port = 71342;

  // start a server process
  process server(server_function);

  write_socket writer(server.get_hostname().c_str(), port);
  file_descriptor_ostream os(writer.get());

  os << to_string(active_message(hello));
  os << to_string(active_message(goodbye));
}

