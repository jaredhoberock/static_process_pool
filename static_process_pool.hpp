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

    static_process_pool(const static_process_pool&) = delete;
    static_process_pool& operator=(const static_process_pool&) = delete;

    // stop accepting work and wait for work to drain
    inline ~static_process_pool()
    {
      stop();
      wait();
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
      for(auto& process : processes_)
      {
        if(process.joinable())
        {
          process.join();
        }
      }
    }

    class executor_type
    {
      public:
        template<class Function>
        void execute(Function&& f) const noexcept
        {
          return pool_.execute(std::forward<Function>(f));
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
      read_socket socket(port);
      file_descriptor_istream is(socket.release());

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

    template<class Function>
    void execute(Function&& f)
    {
      // create an ostream to communicate with the next worker 
      file_descriptor_ostream os(sockets_[next_worker_].get());

      // turn f into an active_message and write it to the ostream
      os << to_string(active_message(std::forward<Function>(f)));

      // round robin through workers
      ++next_worker_;     
      next_worker_ %= processes_.size();
    };

    std::vector<process> processes_;
    std::vector<write_socket> sockets_;
    std::size_t next_worker_;
};

