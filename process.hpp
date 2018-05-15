// Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#pragma once

#include "new_posix_process_executor.hpp"
#include <sys/wait.h>
#include <utility>
#include <tuple>


class process
{
  public:
    using id = pid_t;

    // consider adding an overload which takes an executor which creates new processes
    template<class Function, class... Args,
             __REQUIRES(can_serialize_all<Function,Args...>::value),
             __REQUIRES(can_deserialize_all<Function,Args...>::value),
             __REQUIRES(is_invocable<Function,Args...>::value)
            >
    process(Function&& f, Args&&... args)
    {
      std::tie(id_, hostname_) = create_process_with_executor(std::forward<Function>(f), std::forward<Args>(args)...);
    }

    process(const process&) = delete;
    process& operator=(const process&) = delete;

    process(process&&) = default;

    inline ~process() noexcept
    {
      if(joinable())
      {
        join();
      }
    }

    inline id get_id() const noexcept
    {
      return id_;
    }

    inline const std::string& get_hostname() const noexcept
    {
      return hostname_;
    }

    inline bool joinable() const noexcept
    {
      return get_id() >= 0;
    }

    inline void join() noexcept
    {
      waitpid(get_id(), nullptr, 0);
      detach();
    }

    inline void detach() noexcept
    {
      id_ = -1;
    }

    inline void swap(process& other) noexcept
    {
      std::swap(id_, other.id_);
    }

  private:
    template<class Function, class... Args>
    static std::pair<id, std::string> create_process_with_executor(Function&& f, Args&&... args)
    {
      new_posix_process_executor ex;

      // bind together the function and its arguments into a function with no arguments
      auto g = ex.query(binder)(std::forward<Function>(f), std::forward<Args>(args)...);

      // execute the function in a new process
      ex.execute(std::move(g));

      // return the new process's id and name of its host
      return std::make_pair(ex.query(last_created_process_id), ex.query(last_created_process_hostname));
    }

    id id_;
    std::string hostname_;
};

