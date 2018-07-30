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

#include "variant.hpp"
#include "interprocess_exception.hpp"
#include "post_office.hpp"


// XXX this needs to be specialized for void T because variant can't hold void
template<class T>
class interprocess_promise
{
  public:
    interprocess_promise(std::ostream& os, post_office::address_type receiver_address)
      : os_(os), receiver_address_(receiver_address)
    {}

    interprocess_promise(interprocess_promise&&) = default;

    interprocess_promise(const interprocess_promise&) = delete;

    void set_value(const T& value)
    {
      output_archive ar(os_);

      // wrap the value in a variant before transmitting
      variant<T,interprocess_exception> value_or_exception = value;

      post_office::post(os_, receiver_address_, std::move(value_or_exception));
    }

    void set_value(T&& value)
    {
      output_archive ar(os_);

      // wrap the value in a variant before transmitting
      variant<T,interprocess_exception> value_or_exception = std::move(value);

      post_office::post(os_, receiver_address_, std::move(value_or_exception));
    }

    void set_exception(const interprocess_exception& exception)
    {
      output_archive ar(os_);

      // wrap the exception in a variant before transmitting
      variant<T,interprocess_exception> value_or_exception = exception;

      post_office::post(os_, receiver_address_, std::move(value_or_exception));
    }

  private:
    std::ostream& os_;
    post_office::address_type receiver_address_;
};

