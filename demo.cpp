#include "static_process_pool.hpp"

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

  // create a process pool
  static_process_pool pool(1, port);

  pool.executor().execute(hello);
  pool.executor().execute(goodbye);
}

