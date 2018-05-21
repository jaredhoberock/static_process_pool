# static_process_pool
A `std::static_thread_pool`-alike for processes.

# Demo

To execute work on remote processes, create a `static_process_pool` and use it like you would a thread pool:


```
#include "static_process_pool.hpp"
#include <iostream>

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

  // execute two hello world tasks
  pool.executor().execute(hello);
  pool.executor().execute(hello);

  // execute a task with a result
  auto fut = pool.executor().twoway_execute(make_int);

  int result = fut.get();
  std::cout << "main(): got int " << result << std::endl;
  assert(result == 13);

  // wait for all processes in the pool to complete
  pool.wait();

  std::cout << "main() exiting" << std::endl;
}
```

