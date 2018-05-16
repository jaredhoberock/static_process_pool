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

class basic_socket
{
  public:
    basic_socket(const basic_socket&) = delete;
    basic_socket& operator=(const basic_socket&) = delete;

    basic_socket(basic_socket&& other)
      : file_descriptor_(-1)
    {
      swap(other);
    }
    
    ~basic_socket()
    {
      if(file_descriptor_ != -1)
      {
        if(close(file_descriptor_) == -1)
        {
          std::cerr << std::system_error(errno, std::system_category(), "basic_socket dtor: Error after close()").what() << std::endl;
          std::terminate();
        }
      }
    }

    void swap(basic_socket& other) noexcept
    {
      std::swap(file_descriptor_, other.file_descriptor_);
    }

    int get() const noexcept
    {
      return file_descriptor_;
    }

    int release() noexcept
    {
      int result = -1;
      std::swap(file_descriptor_, result);
      return result;
    }

  protected:
    basic_socket(int file_descriptor)
      : file_descriptor_(file_descriptor)
    {}

  private:
    int file_descriptor_;
};


class listening_socket : public basic_socket
{
  public:
    listening_socket(int desired_port)
      : basic_socket(socket(AF_INET, SOCK_STREAM, 0)),
        port_(-1)
    {
      if(get() == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after socket()");
      }

      // this ensures that closing the socket allows the port to be reused immediately
      // if other processes were using the port previously, the bind() call below may fail without this option set
      int yes = 1;
      if(setsockopt(get(), SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after setsockopt()");
      }

      sockaddr_in server_address{};

      server_address.sin_family = AF_INET;
      server_address.sin_addr.s_addr = INADDR_ANY;
      server_address.sin_port = htons(desired_port);

      // bind the socket to our desired port
      if(bind(get(), reinterpret_cast<const sockaddr*>(&server_address), sizeof(server_address)) == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after bind()");
      }

      // make this socket a listening socket, listen for a single connection
      if(listen(get(), 1) == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after listen()");
      }

      // figure out which port we were assigned
      socklen_t len = sizeof(server_address);
      if(getsockname(get(), reinterpret_cast<sockaddr*>(&server_address), &len) == -1)
      {
        throw std::system_error(errno, std::system_category(), "listening_socket ctor: Error after getsockname()");
      }

      // note the port
      port_ = ntohs(server_address.sin_port);
    }

    inline int port() const noexcept
    {
      return port_;
    }

  private:
    int port_;
};


class read_socket : public basic_socket
{
  public:
    read_socket(listening_socket listener)
      : basic_socket(accept(listener.get(), nullptr, nullptr)),
        port_(listener.port())
    {
      if(get() == -1)
      {
        throw std::system_error(errno, std::system_category(), "read_socket ctor: Error after accept()");
      }
    }

    read_socket(int desired_port)
      : read_socket(listening_socket(desired_port))
    {}

    inline int port() const noexcept
    {
      return port_;
    }

  private:
    int port_;
};


class write_socket : public basic_socket
{
  public:
    write_socket(const char* hostname, int port, std::size_t max_num_connection_attempts = 1000)
      : basic_socket(socket(AF_INET, SOCK_STREAM, 0))
    {
      if(get() == -1)
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
      server_address.sin_port = htons(port);

      // keep attempting a connection while the server refuses
      int num_connection_attempts = 0;
      int connect_result = 0;
      while((connect_result = connect(get(), reinterpret_cast<sockaddr*>(&server_address), sizeof(server_address))) == -1 && num_connection_attempts < max_num_connection_attempts)
      {
        if(errno != ECONNREFUSED)
        {
          throw std::system_error(errno, std::system_category(), "write_socket ctor: Error after connect()");
        }

        ++num_connection_attempts;
      }

      if(connect_result == -1)
      {
        throw std::system_error(errno, std::system_category(), "write_socket ctor: Error after connect()");
      }
    }
};

