// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "process.hpp"
#include "socket.hpp"
#include "file_descriptor_stream.hpp"

#include <vector>
#include <memory>

class static_process_pool
{
  public:
    // XXX consider an overload receiving an executor to actually create the processes
    inline explicit static_process_pool(std::size_t num_processes)
      : next_worker_(0)
    {
      for(std::size_t i = 0; i < num_processes; ++i)
      {
        // create a socket to listen for the remote port number for each process
        listening_socket listener(0);

        // create a new process
        processes_.emplace_back(serve, basic_active_message<std::ostream*>(make_ostream_to_host, this_process::hostname(), listener.port()));

        // create a socket to receive a message from the process
        read_socket reader(std::move(listener));
        file_descriptor_istream is(reader.get());

        // receive a message from the process, which, when activated
        // creates a new ostream connected to the process
        basic_active_message<std::ostream*> make_ostream;
        is >> make_ostream;

        // make a new ostream to use to communicate with the process
        ostream_ptrs_.emplace_back(make_ostream.activate());
      }
    }

    inline static_process_pool(static_process_pool&& other)
      : static_process_pool(0)
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
      std::swap(ostream_ptrs_, other.ostream_ptrs_);
      std::swap(next_worker_, other.next_worker_);
    }

    // signal all work to complete
    inline void stop()
    {
      // destroy each output stream
      ostream_ptrs_.clear();
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
    static std::ostream* make_ostream_to_host(const std::string& hostname, int port)
    {
      // create a new socket for writing to the host's port
      write_socket ws(hostname.c_str(), port);

      // turn that socket into a newly-created ostream
      return new owning_file_descriptor_ostream(ws.release());
    }

    static void serve(basic_active_message<std::ostream*> make_ostream_to_client)
    {
      // create a socket to listen for messages from the client
      listening_socket listener(0);
      
      // send the client an active message which, when activated on the client,
      // establishes an ostream connected to this server
      {
        // create an ostream to the client
        std::unique_ptr<std::ostream> os_ptr(make_ostream_to_client.activate());

        active_message message(make_ostream_to_host, this_process::hostname(), listener.port());
        *os_ptr << message;
      }

      // turn the listener into a reader
      read_socket reader(std::move(listener));
      file_descriptor_istream is(reader.get());

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
      // turn f into an active_message and write it to the next worker's ostream
      *ostream_ptrs_[next_worker_] << active_message(std::forward<Function>(f));

      // round robin through workers
      ++next_worker_;     
      next_worker_ %= processes_.size();
    };

    std::vector<process> processes_;
    std::vector<std::unique_ptr<std::ostream>> ostream_ptrs_;
    std::size_t next_worker_;
};

