#include "process.hpp"
#include "socket.hpp"
#include "file_descriptor_stream.hpp"

#include <vector>

class static_process_pool
{
  public:
    // XXX consider an overload receiving an executor to actually create the processes
    inline explicit static_process_pool(std::size_t num_processes, int base_port = 71342)
      : next_worker_(0)
    {
      for(std::size_t i = 0; i < num_processes; ++i, ++base_port)
      {
        // create a new process listening on base_port
        processes_.emplace_back(serve, base_port);

        // create a new socket for writing to base_port
        sockets_.emplace_back(processes_.back().get_hostname().c_str(), base_port);
      }
    }

    inline static_process_pool(static_process_pool&& other)
      : static_process_pool(0, 0)
    {
      swap(other);
    }

    static_process_pool(const static_process_pool&) = delete;
    static_process_pool& operator=(const static_process_pool&) = delete;

    // stop accepting work and wait for work to drain
    inline ~static_process_pool()
    {
      stop();
      wait();
    }

    void swap(static_process_pool& other)
    {
      std::swap(processes_, other.processes_);
      std::swap(sockets_, other.sockets_);
      std::swap(next_worker_, other.next_worker_);
    }

    // signal all work to complete
    inline void stop()
    {
      // destroy each write socket
      sockets_.clear();
    }

    // wait for all processes in the process pool to complete
    inline void wait()
    {
      stop();
      processes_.clear();
    }

    class executor_type
    {
      public:
        template<class Function>
        void execute(Function&& f) const noexcept
        {
          pool_.execute(std::forward<Function>(f));
        }

      private:
        friend class static_process_pool;

        executor_type(static_process_pool& pool)
          : pool_(pool)
        {}

        static_process_pool& pool_;
    };

    executor_type executor() noexcept
    {
      return executor_type{*this};
    }

  private:
    static void serve(int port)
    {
      // XXX consider introducing socket_istream
      read_socket socket(port);
      file_descriptor_istream is(socket.get());

      active_message message;

      // read and activate messages until we run out
      while(is >> message)
      {
        message.activate();
      }
    }

    template<class Function>
    void execute(Function&& f)
    {
      // XXX consider introducing socket_ostream
      // create an ostream to communicate with the next worker 
      file_descriptor_ostream os(sockets_[next_worker_].get());

      // turn f into an active_message and write it to the ostream
      os << active_message(std::forward<Function>(f));

      // round robin through workers
      ++next_worker_;     
      next_worker_ %= processes_.size();
    };

    std::vector<process> processes_;
    std::vector<write_socket> sockets_;
    std::size_t next_worker_;
};

