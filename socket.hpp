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

#pragma once

#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>

#include <system_error>
#include <iostream>
#include <exception>

class listening_socket
{
  public:
    listening_socket(int port)
      : file_descriptor_(socket(AF_INET, SOCK_STREAM, 0))
    {
      if(file_descriptor_ == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after socket()");
      }

      // this ensures that closing the socket allows the port to be reused immediately
      // if other processes were using the port previously, the bind() call below may fail without this option set
      int yes = 1;
      if(setsockopt(file_descriptor_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after setsockopt()");
      }

      sockaddr_in server_address{};

      server_address.sin_family = AF_INET;
      server_address.sin_addr.s_addr = INADDR_ANY;
      server_address.sin_port = port;

      // bind the socket to our selected port
      if(bind(file_descriptor_, reinterpret_cast<const sockaddr*>(&server_address), sizeof(server_address)) == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after bind()");
      }

      // make this socket a listening socket, listen for a single connection
      if(listen(file_descriptor_, 1) == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after listen()");
      }
    }

    listening_socket(listening_socket&& other)
      : file_descriptor_(-1)
    {
      std::swap(file_descriptor_, other.file_descriptor_);
    }

    ~listening_socket()
    {
      if(file_descriptor_ != -1)
      {
        if(close(file_descriptor_) == -1)
        {
          std::cerr << "listening_socket dtor: Error after close()" << std::endl;
          std::terminate();
        }
      }
    }

    int get()
    {
      return file_descriptor_;
    }

  private:
    int file_descriptor_;
};

class read_socket
{
  public:
    read_socket(listening_socket listener)
      : file_descriptor_(accept(listener.get(), nullptr, nullptr))
    {
      if(file_descriptor_ == -1)
      {
        throw std::system_error(errno, std::system_category(), "make_read_socket(): Error after listen()");
      }
    }

    read_socket(read_socket&& other)
      : file_descriptor_(-1)
    {
      std::swap(file_descriptor_, other.file_descriptor_);
    }

    ~read_socket()
    {
      if(file_descriptor_ != -1)
      {
        if(close(file_descriptor_) == -1)
        {
          std::cerr << "read_socket dtor: Error after close()" << std::endl;
          std::terminate();
        }
      }
    }

    read_socket(int port)
      : read_socket(listening_socket(port))
    {}

    int get() const
    {
      return file_descriptor_;
    }

    int release()
    {
      int result = -1;
      std::swap(file_descriptor_, result);
      return result;
    }

  private:
    int file_descriptor_;
};


class write_socket
{
  public:
    write_socket(const char* hostname, int port)
      : file_descriptor_(socket(AF_INET, SOCK_STREAM, 0))
    {
      if(file_descriptor_ == -1)
      {
        throw std::system_error(errno, std::system_category(), "write_socket ctor: Error after socket()");
      }

      // get the address of the server
      struct hostent* server = gethostbyname(hostname);
      if(server == nullptr)
      {
        throw std::system_error(errno, std::system_category(), "write_socket ctor: Error after gethostbyname()");
      }

      sockaddr_in server_address{};

      server_address.sin_family = AF_INET;
      std::memcpy(&server_address.sin_addr.s_addr, server->h_addr, server->h_length);
      server_address.sin_port = port;

      // keep attempting a connection while the server refuses
      int attempt = 0;
      int connect_result = 0;
      while((connect_result = connect(file_descriptor_, reinterpret_cast<sockaddr*>(&server_address), sizeof(server_address))) == -1 && attempt < 1000)
      {
        if(errno != ECONNREFUSED)
        {
          throw std::system_error(errno, std::system_category(), "write_socket ctor: Error after connect()");
        }

        ++attempt;
      }

      if(connect_result == -1)
      {
        throw std::system_error(errno, std::system_category(), "write_socket ctor: Error after connect()");
      }
    }

    ~write_socket()
    {
      close(file_descriptor_);
    }

    int get() const
    {
      return file_descriptor_;
    }

  private:
    int file_descriptor_;
};

