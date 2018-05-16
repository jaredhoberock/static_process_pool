#include "static_process_pool.hpp"

void hello()
{
  std::cout << "hello world from process " << this_process::get_id() << std::endl;
}

int main()
{
  // create a pool of ten processes
  static_process_pool pool(10);

  pool.executor().execute(hello);
  pool.executor().execute(hello);

  // wait for all processes in the pool to complete
  pool.wait();

  std::cout << "main() exiting" << std::endl;
}

