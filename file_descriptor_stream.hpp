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

// file_descriptor_ostream & file_descriptor_istream are based on
// http://www.josuttis.com/cppcode/fdstream.hpp.html
//
// (C) Copyright Nicolai M. Josuttis 2001.
//  Permission to copy, use, modify, sell and distribute this software
//  is granted provided this copyright notice appears in all copies.
//  This software is provided "as is" without express or implied
//  warranty, and with no claim as to its suitability for any purpose.

#pragma once

#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <utility>

class file_descriptor_ostream : public std::ostream
{
  public:
    inline file_descriptor_ostream(int fd)
      : std::ostream(nullptr), buffer_(fd)
    {
      rdbuf(&buffer_);
    }

    inline int file_descriptor() const noexcept
    {
      return buffer_.file_descriptor();
    }

  private:
    class file_descriptor_buffer : public std::streambuf
    {
      public:
        using traits_type = std::streambuf::traits_type;

        inline file_descriptor_buffer(int fd)
          : fd_(fd)
        {}

        inline virtual int_type overflow(int_type c)
        {
          if(c != traits_type::eof())
          {
            char z = c;
            if(::write(fd_, &z, 1) != 1)
            {
              return traits_type::eof();
            }
          }

          return c;
        }

        inline virtual std::streamsize xsputn(const char* s, std::streamsize num)
        {
          return ::write(fd_, s, num);
        }

        inline int file_descriptor() const noexcept
        {
          return fd_;
        }

      private:
        int fd_;
    };

    file_descriptor_buffer buffer_;
};


class owning_file_descriptor_ostream : public file_descriptor_ostream
{
  public:
    using file_descriptor_ostream::file_descriptor_ostream;

    virtual ~owning_file_descriptor_ostream()
    {
      if(file_descriptor() != -1)
      {
        if(close(file_descriptor()) == -1)
        {
          std::cerr << std::system_error(errno, std::system_category(), "owning_file_descriptor_ostream dtor: Error after close()").what() << std::endl;
          std::terminate();
        }
      }
    }
};


class file_descriptor_istream : public std::istream
{
  public:
    inline file_descriptor_istream(int fd)
      : std::istream(nullptr), buffer_(fd)
    {
      rdbuf(&buffer_);
    }

    inline file_descriptor_istream(file_descriptor_istream&& other)
      : std::istream(nullptr), buffer_(std::move(other.buffer_))
    {
      // null the other stream's buffer
      other.rdbuf(nullptr);

      // set our buffer
      rdbuf(&buffer_);
    }
    
    inline int file_descriptor() const
    {
      return buffer_.file_descriptor();
    }

  private:
    class file_descriptor_buffer : public std::streambuf
    {
      public:
        using traits_type = std::streambuf::traits_type;

        inline file_descriptor_buffer(int fd)
          : fd_(fd)
        {
          setg(buffer_.data() + putback_size_,  // beginning of putback area
               buffer_.data() + putback_size_,  // read position
               buffer_.data() + putback_size_); // end position
        }

        file_descriptor_buffer(file_descriptor_buffer&& other)
          : buffer_(std::move(other.buffer_)), fd_(-1)
        {
          std::swap(fd_, other.fd_);

          int eback_idx = other.eback() - other.buffer_.data();
          int gptr_idx  = other.gptr() - other.buffer_.data();
          int egptr_idx = other.egptr() - other.buffer_.data();

          setg(buffer_.data() + eback_idx,
               buffer_.data() + gptr_idx,
               buffer_.data() + egptr_idx);
        }
        
        int file_descriptor() const
        {
          return fd_;
        }

      protected:
        constexpr const static int putback_size_ = 4;
        constexpr const static int buffer_size_ = 1024;

        std::array<char,putback_size_ + buffer_size_> buffer_;
        int fd_;

        inline virtual int_type underflow()
        {
          // is read position before end of buffer?
          if(gptr() < egptr())
          {
            return traits_type::to_int_type(*gptr());
          }

          // process size of putback area
          // use number of characters read
          // but at most size of putback area
          int num_putback = gptr() - eback();
          num_putback = std::min(num_putback, putback_size_);

          // copy up to putback_size_ characters previously read
          // into the putback area
          std::memmove(buffer_.data() + (putback_size_ - num_putback), gptr() - num_putback, num_putback);

          // read at most buffer_size_ new characters
          int num = ::read(fd_, buffer_.data() + putback_size_, buffer_size_);
          if(num <= 0)
          {
            return traits_type::eof();
          }

          // reset buffer pointers
          setg(buffer_.data() + (putback_size_ - num_putback), // beginning of putback area
               buffer_.data() + putback_size_,                 // read position
               buffer_.data() + putback_size_ + num);          // end of buffer

          // return next character
          return traits_type::to_int_type(*gptr());
        }
    };

    file_descriptor_buffer buffer_;
};

// XXX for some reason we have to include the definition of putback_size_ here
//     but not so for buffer_size_
const int file_descriptor_istream::file_descriptor_buffer::putback_size_;


class owning_file_descriptor_istream : public file_descriptor_istream
{
  public:
    using file_descriptor_istream::file_descriptor_istream;

    virtual ~owning_file_descriptor_istream()
    {
      if(file_descriptor() != -1)
      {
        if(close(file_descriptor()) == -1)
        {
          std::cerr << std::system_error(errno, std::system_category(), "owning_file_descriptor_istream dtor: Error after close()").what() << std::endl;
          std::terminate();
        }
      }
    }
};

