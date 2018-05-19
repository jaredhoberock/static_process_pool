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

#include <iostream>
#include <utility>
#include <future>

#include "optional.hpp"
#include "variant.hpp"
#include "interprocess_exception.hpp"


template<class T>
class interprocess_future
{
  public:
    interprocess_future(post_office& po)
      : mailbox_(po)
    {}

    interprocess_future(interprocess_future&& other) = default;

    interprocess_future(const interprocess_future&) = delete;

    T get()
    {
      // wait for the result to become ready
      wait();

      // after waiting, the result should be ready
      // otherwise, the result has already been retrieved
      if(!is_ready())
      {
        throw std::future_error(std::future_errc::future_already_retrieved);
      }

      // if the result holds an exception, throw it
      if(holds_alternative<interprocess_exception>(*result_or_exception_))
      {
        throw ::get<interprocess_exception>(*result_or_exception_);
      }

      // move the result into a variable
      T result = std::move(::get<T>(*result_or_exception_));

      // reset the result's container
      result_or_exception_.reset();

      return result;
    }

    bool is_ready() const noexcept
    {
      return result_or_exception_.has_value();
    }

    void wait()
    {
      if(!valid())
      {
        throw std::future_error(std::future_errc::no_state);
      }

      if(!is_ready())
      {
        result_or_exception_ = mailbox_.blocking_receive();
      }
    }

    bool valid() const noexcept
    {
      return mailbox_ or result_or_exception_;
    }

    post_office::address_type identity() const noexcept
    {
      return mailbox_.address();
    }

  private:
    using state_type = variant<T,interprocess_exception>;

    post_office::mailbox<state_type> mailbox_;
    optional<state_type> result_or_exception_;
};

