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
#include "post_office.hpp"
#include "interprocess_future.hpp"
#include "interprocess_promise.hpp"

#include <vector>
#include <memory>

class static_process_pool
{
  using twoway_connection = std::pair<std::istream*, std::ostream*>;

  public:
    // XXX consider an overload receiving an executor to actually create the processes
    inline explicit static_process_pool(std::size_t num_processes)
      : next_worker_(0)
    {
      for(std::size_t i = 0; i < num_processes; ++i)
      {
        // create a socket to listen for reply from each process
        listening_socket listener(0);

        // create a new process
        processes_.emplace_back(serve, basic_active_message<twoway_connection>(make_twoway_connection_on_server, this_process::hostname(), listener.port()));

        // create a socket to receive a message from the process to establish our own twoway connection
        read_socket reader(std::move(listener));

        // turn that socket into an istream
        istream_ptrs_.emplace_back(new file_descriptor_istream(reader.release()));

        // create a post office for receiving on that istream
        post_offices_.emplace_back(*istream_ptrs_.back());

        // receive a message from the process which, when activated,
        // creates a new ostream connected to the process
        basic_active_message<std::ostream*> make_ostream;
        *istream_ptrs_.back() >> make_ostream;

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
      // this signals the processes to stop
      ostream_ptrs_.clear();

      // destroy each input stream for consistency
      istream_ptrs_.clear();
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
          pool_.execute(which_process_, std::forward<Function>(f));
        }

        template<class Function>
        auto twoway_execute(Function&& f) const noexcept
        {
          return pool_.twoway_execute(which_process_, std::forward<Function>(f));
        }

      private:
        friend class static_process_pool;

        executor_type(static_process_pool& pool, size_t which_process)
          : pool_(pool), which_process_(which_process)
        {}

        static_process_pool& pool_;
        size_t which_process_;
    };

    executor_type executor(size_t which_process)
    {
      return executor_type{*this, which_process};
    }

    executor_type executor() noexcept
    {
      // select a process on which to execute
      std::size_t selected_process = next_worker_;

      // round robin through workers
      ++next_worker_;     
      next_worker_ %= processes_.size();

      return executor(selected_process);
    }

  private:
    static std::ostream* make_ostream_to_host(const std::string& hostname, int port)
    {
      // create a new socket for writing to the host's port
      write_socket ws(hostname.c_str(), port);

      // turn that socket into a newly-created ostream
      return new owning_file_descriptor_ostream(ws.release());
    }

    static twoway_connection make_twoway_connection_on_server(const std::string& client_hostname, int client_port)
    {
      // create a socket to listen for messages from the client
      listening_socket listener(0);
      
      // create an ostream to the client
      std::unique_ptr<std::ostream> os_ptr(make_ostream_to_host(client_hostname, client_port));

      // send the client an active message which, when activated on the client,
      // establishes an ostream connected from the client to this server
      *os_ptr << make_active_message(make_ostream_to_host, this_process::hostname(), listener.port());

      // turn the listener into a reader
      read_socket reader(std::move(listener));

      // create an istream for incoming requests from the client
      std::unique_ptr<std::istream> is_ptr(new owning_file_descriptor_istream(reader.release()));

      return std::make_pair(is_ptr.release(), os_ptr.release());
    }

    // this is the function run by each server
    static void serve(basic_active_message<twoway_connection> make_twoway_connection_to_client)
    {
      // establish a two-way connection to the client
      auto stream_ptrs = make_twoway_connection_to_client.activate();

      // unpack the streams, being careful not to cross them
      std::unique_ptr<std::istream> is_ptr(stream_ptrs.first);
      std::unique_ptr<std::ostream> os_ptr(stream_ptrs.second);

      std::istream& is = *is_ptr;
      std::ostream& os = *os_ptr;

      basic_active_message<void,std::ostream&> message;

      // read and activate messages until we run out
      while(is >> message)
      {
        message.activate(os);
      }
    }

    inline void execute_active_message_on_process(std::size_t which_process, basic_active_message<void,std::ostream&> message)
    {
      // write the message to the process's ostream
      *ostream_ptrs_[which_process] << message;
    }

    template<class Function>
    static void ignore_ostream_and_invoke(Function f, std::ostream& os)
    {
      // any result from f is discarded
      f();
    }

    template<class Function>
    void execute(size_t which_process, Function&& f)
    {
      // turn f into an active_message
      // give f an ostream& parameter which will be ignored when invoked on the server
      basic_active_message<void,std::ostream&> message(ignore_ostream_and_invoke<typename std::decay<Function>::type>, std::forward<Function>(f));

      // execute the message on the selected process
      execute_active_message_on_process(which_process, message);
    };

    template<class Result, class Function>
    static void invoke_function_and_fulfill_promise_connected_to_ostream(Function f, post_office::address_type receiver_identity, std::ostream& os)
    {
      interprocess_promise<Result> promise(os, receiver_identity);

      try
      {
        auto value = f();

        promise.set_value(value);
      }
      catch(...)
      {
        promise.set_exception(interprocess_exception("exception"));
      }
    }

    template<class Function>
    interprocess_future<
      invoke_result_t<typename std::decay<Function>::type>
    >
      twoway_execute(size_t which_process, Function&& f)
    {
      using result_type = invoke_result_t<typename std::decay<Function>::type>;

      // create a future to receive our result
      interprocess_future<result_type> future_result(post_offices_[which_process]);

      // create an active message which, when executed on the remote process,
      // invokes f and fulfills a promise connected to this future
      auto execute_me_remotely = invoke_function_and_fulfill_promise_connected_to_ostream<result_type, typename std::decay<Function>::type>;
      basic_active_message<void,std::ostream&> message(execute_me_remotely, std::forward<Function>(f), future_result.identity());
      execute_active_message_on_process(which_process, std::move(message));

      // return the future
      return future_result;
    }

    std::vector<process> processes_;
    std::vector<std::unique_ptr<std::ostream>> ostream_ptrs_;

    // XXX keeping both a list of istreams and a list of post_offices is a little redundant
    //     if each post_office owned its istream, we could avoid having to manage both
    std::vector<std::unique_ptr<std::istream>> istream_ptrs_;
    std::vector<post_office> post_offices_;

    std::size_t next_worker_;
};

