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
#include <string>
#include <stdexcept>
#include "serialization.hpp"


// XXX next step: figure out how to make mailboxes work
// XXX we probably need two types: one for sending and one for receiving
class post_office
{
  public:
    using address_type = std::string*;

    post_office(std::istream& is)
      : is_(is)
    {}

    address_type make_new_address()
    {
      return new std::string();
    }

    bool available(address_type address) const
    {
      return !address->empty();
    }

    template<class T>
    T blocking_receive(address_type address)
    {
      if(address->empty() and !block_and_sort_available_messages_until(address))
      {
        throw std::runtime_error("mailbox empty");
      }

      std::string result = std::move(*address);

      delete address;

      return from_string<T>(result);
    }

    template<class T>
    static void send(std::ostream& os, address_type address, const T& value)
    {
      output_archive ar(os);
      ar(address, to_string(value));
    }

  private:
    bool block_and_sort_available_messages_until(address_type address)
    {
      address_type current_address = nullptr;
      std::string message;

      while(is_ and current_address != address)
      {
        // deserialize the next piece of mail
        input_archive ar(is_);
        ar(current_address, message);

        // sort the message into the right mailbox
        *current_address = std::move(message);
      }

      return address == current_address;
    }

    std::istream& is_;
};

