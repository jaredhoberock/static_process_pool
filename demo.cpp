#include "static_process_pool.hpp"

void hello()
{
  std::cout << "hello world from process " << this_process::get_id() << std::endl;
}

int make_int()
{
  std::cout << "making int on process " << this_process::get_id() << std::endl;
  return 13;
}

int main()
{
  // create a pool of ten processes
  static_process_pool pool(10);

  pool.executor().execute(hello);
  pool.executor().execute(hello);

  auto fut = pool.executor().twoway_execute(make_int);

  int result = fut.get();
  std::cout << "main(): got int " << result << std::endl;
  assert(result == 13);

  // wait for all processes in the pool to complete
  pool.wait();

  std::cout << "main() exiting" << std::endl;
}

