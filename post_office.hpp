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


class post_office
{
  public:
    using address_type = std::string*;

    // constructs a new post_office to receive incoming posts on is
    post_office(std::istream& is)
      : is_(is)
    {}

    // requires that os is connected to the istream which was used as a constructor argument
    // for the post_office which owns mailbox_address
    template<class T>
    static void post(std::ostream& os, address_type mailbox_address, const T& value)
    {
      output_archive ar(os);
      ar(mailbox_address, to_string(value));
    }

    template<class T>
    class mailbox
    {
      public:
        mailbox(post_office& po)
          : post_office_(po), address_(post_office_.new_address())
        {}

        mailbox(mailbox&& other)
          : post_office_(other.post_office_),
            address_(nullptr)
        {
          std::swap(address_, other.address_);
        }

        ~mailbox()
        {
          if(valid())
          {
            post_office_.delete_address(address());
          }
        }

        address_type address() const noexcept
        {
          return address_;
        }

        bool valid() const noexcept
        {
          return address_ != nullptr;
        }

        operator bool () const noexcept
        {
          return valid();
        }

        T blocking_receive()
        {
          // get the result
          T result = post_office_.blocking_receive<T>(address());

          // invalidate
          address_ = nullptr;

          return result;
        }

      private:
        post_office& post_office_;
        address_type address_;
    };

  private:
    address_type new_address()
    {
      return new std::string();
    }

    void delete_address(address_type address)
    {
      delete address;
    }

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

    template<class T>
    T blocking_receive(address_type address)
    {
      if(address->empty() and !block_and_sort_available_messages_until(address))
      {
        throw std::runtime_error("mailbox empty");
      }

      std::string result = std::move(*address);

      delete_address(address);

      return from_string<T>(result);
    }

    std::istream& is_;
};

