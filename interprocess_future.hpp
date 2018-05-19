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
    interprocess_future(post_office& po, post_office::address_type address)
      : post_office_(po), address_(address)
    {}

    interprocess_future(interprocess_future&& other) noexcept
      : post_office_(other.post_office_), address_(nullptr)
    {
      std::swap(address_, other.address_);
    }

    interprocess_future(const interprocess_future&) = delete;

    ~interprocess_future()
    {
      if(post_office_.valid(address_))
      {
        // XXX we should introduce a mailbox type for RAII
        post_office_.delete_address(address_);
      }
    }

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

    bool is_ready() const
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
        using state_type = variant<T,interprocess_exception>;
        result_or_exception_ = post_office_.blocking_receive<state_type>(address_);

        address_ = nullptr;
      }
    }

    bool valid() const
    {
      return post_office_.valid(address_) or result_or_exception_.has_value();
    }

  private:
    post_office& post_office_;
    post_office::address_type address_;
    optional<variant<T,interprocess_exception>> result_or_exception_;
};

