#include "static_process_pool.hpp"
#include <cstdlib>
#include <algorithm>

interprocess_future<void*> async_malloc(const static_process_pool::executor_type& ex, size_t num_bytes)
{
  auto f = ex.query(binder)(std::malloc, num_bytes);

  return ex.twoway_execute(f);
}

// free doesn't return a future because it's not clear we care when the free actually happens
void free(const static_process_pool::executor_type& ex, void* ptr, size_t num_bytes)
{
  auto f = ex.query(binder)(std::free, ptr);

  ex.execute(f);
}

// XXX this should really return interprocess_future<void>, but we don't currently have a conversion from interprocess_future<T> to interprocess_future<void>
interprocess_future<float*> async_fill(const static_process_pool::executor_type& ex, float* ptr, size_t n, float value)
{
  auto f = ex.query(binder)(std::fill_n<float*, size_t, float>, ptr, n, value);

  return ex.twoway_execute(f);
}

// XXX return int because we don't currently handle T = void in interprocess_future
int saxpy(float a, const float* x, const float* y, float* z, size_t n)
{
  for(size_t i = 0; i < n; ++i)
  {
    z[i] = a * x[i] + y[i];
  }

  return 0;
}

// XXX return interprocess_future<int> because we don't currently handle T = void in interprocess_future
interprocess_future<int> async_saxpy(const static_process_pool::executor_type& ex, float a, const float* x, const float* y, float* z, size_t n)
{
  auto f = ex.query(binder)(saxpy, a, x, y, z, n);

  return ex.twoway_execute(f);
}

interprocess_future<std::iterator_traits<const float*>::difference_type> async_count(const static_process_pool::executor_type& ex, const float* first, size_t n, float value)
{
  auto f = ex.query(binder)(std::count<const float*,float>, first, first + n, value);

  return ex.twoway_execute(f);
}

int main()
{
  size_t num_processes = 2;

  // start some processes
  static_process_pool pool(num_processes);

  // evenly tile the problem size
  size_t n = 1 << 20;
  size_t tile_size = n / num_processes;

  // allocate tiled vectors x, y, & z on each process
  std::vector<float*> x, y, z;
  for(size_t i = 0; i < num_processes; ++i)
  {
    auto future_ptr = async_malloc(pool.executor(i), sizeof(float) * tile_size);
    x.push_back(reinterpret_cast<float*>(future_ptr.get()));

    future_ptr = async_malloc(pool.executor(i), sizeof(float) * tile_size);
    y.push_back(reinterpret_cast<float*>(future_ptr.get()));

    future_ptr = async_malloc(pool.executor(i), sizeof(float) * tile_size);
    z.push_back(reinterpret_cast<float*>(future_ptr.get()));
  }

  // intialize x & y
  for(size_t i = 0; i < num_processes; ++i)
  {
    async_fill(pool.executor(i), x[i], tile_size, 1.f).wait();
    async_fill(pool.executor(i), y[i], tile_size, 2.f).wait();
  }

  // execute SAXPY
  float a = 1.f;
  for(size_t i = 0; i < num_processes; ++i)
  {
    async_saxpy(pool.executor(i), a, x[i], y[i], z[i], tile_size).wait();
  }

  // check results
  for(size_t i = 0; i < num_processes; ++i)
  {
    size_t num_threes = async_count(pool.executor(i), z[i], tile_size, 3.0f).get();
    assert(tile_size == num_threes);
  }

  // deallocate each tile
  for(size_t i = 0; i < num_processes; ++i)
  {
    free(pool.executor(i), x[i], tile_size);
    free(pool.executor(i), y[i], tile_size);
    free(pool.executor(i), z[i], tile_size);
  }

  // wait for everything to complete
  pool.wait();

  std::cout << "OK" << std::endl;
}

