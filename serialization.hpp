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

#include <iostream>
#include <tuple>
#include <utility>
#include <string>
#include <typeinfo>
#include <sstream>
#include <cstring>
#include <array>
#include "string_view_stream.hpp"
#include "tuple.hpp"
#include "variant.hpp"


#define __REQUIRES(...) typename std::enable_if<(__VA_ARGS__)>::type* = nullptr


// this serialization scheme is based on Cereal
// see http://uscilab.github.io/cereal

template<class OutputArchive, class T>
void serialize(OutputArchive& ar, const T& value)
{
  // by default, use formatted output, and follow with whitespace
  ar.stream() << value << ",";
}

template<class OutputArchive, class Result, class... Args>
void serialize(OutputArchive& ar, Result (*const &fun_ptr)(Args...))
{
  void* void_ptr = reinterpret_cast<void*>(fun_ptr);

  serialize(ar, void_ptr);
}

template<class OutputArchive>
void serialize(OutputArchive& ar, const std::string& s)
{
  // output the length
  serialize(ar, s.size());

  // output the bytes
  ar.stream().write(s.data(), s.size());
}


template<class OutputArchive, class T, size_t N>
void serialize(OutputArchive& ar, const std::array<T,N>& a)
{
  // serialize each element
  for(const auto& e : a)
  {
    serialize(ar, e);
  }
}


template<class InputArchive, class T>
void deserialize(InputArchive& ar, T& value)
{
  // by default, use formatted input, and consume trailing whitespace
  char delimiter = 0;
  ar.stream() >> value >> delimiter;

  assert(delimiter == ',' or ar.stream().eof());
}

template<class InputArchive, class T,
         __REQUIRES(!std::is_void<T>::value)>
void deserialize(InputArchive& ar, T*& ptr)
{
  void* void_ptr = nullptr;
  deserialize(ar, void_ptr);

  ptr = reinterpret_cast<T*>(void_ptr);
}

template<class InputArchive, class Result, class... Args>
void deserialize(InputArchive& ar, Result (*&fun_ptr)(Args...))
{
  void* void_ptr = nullptr;
  deserialize(ar, void_ptr);

  using function_ptr_type = Result (*)(Args...);
  fun_ptr = reinterpret_cast<function_ptr_type>(void_ptr);
}


template<class InputArchive>
void deserialize(InputArchive& ar, std::string& s)
{
  // read the length and resize the string
  std::size_t length = 0;
  deserialize(ar, length);
  s.resize(length);

  // read characters from the stream
  ar.stream().read(&s.front(), length);
}


template<class InputArchive, class T, size_t N>
void deserialize(InputArchive& ar, std::array<T,N>& a)
{
  // deserialize each element
  for(auto& e : a)
  {
    deserialize(ar, e);
  }
}


template<class OutputArchive, class T1, class T2>
void serialize(OutputArchive& ar, const std::pair<T1,T2>& p)
{
  ar(p.first, p.second);
}

template<class InputArchive, class T1, class T2>
void deserialize(InputArchive& ar, std::pair<T1,T2>& p)
{
  ar(p.first, p.second);
}
         


template<size_t Index, class OutputArchive, class... Ts, __REQUIRES(Index == sizeof...(Ts))>
void serialize_tuple_impl(OutputArchive& ar, const std::tuple<Ts...>& tuple)
{
}

template<size_t Index, class OutputArchive, class... Ts, __REQUIRES(Index < sizeof...(Ts))>
void serialize_tuple_impl(OutputArchive& ar, const std::tuple<Ts...>& tuple)
{
  serialize(ar, std::get<Index>(tuple));
  serialize_tuple_impl<Index+1>(ar, tuple);
}

template<class OutputArchive, class... Ts>
void serialize(OutputArchive& ar, const std::tuple<Ts...>& tuple)
{
  serialize_tuple_impl<0>(ar, tuple);
}


template<size_t Index, class InputArchive, class... Ts, __REQUIRES(Index == sizeof...(Ts))>
void deserialize_tuple_impl(InputArchive& ar, std::tuple<Ts...>& tuple)
{
}

template<size_t Index, class InputArchive, class... Ts, __REQUIRES(Index < sizeof...(Ts))>
void deserialize_tuple_impl(InputArchive& ar, std::tuple<Ts...>& tuple)
{
  deserialize(ar, std::get<Index>(tuple));
  deserialize_tuple_impl<Index+1>(ar, tuple);
}

template<class InputArchive, class... Ts>
void deserialize(InputArchive& ar, std::tuple<Ts...>& tuple)
{
  deserialize_tuple_impl<0>(ar, tuple);
}


template<class OutputArchive>
struct serialize_visitor
{
  OutputArchive& ar;

  template<class T>
  void operator()(const T& value) const
  {
    ar(value);
  }
};


template<class OutputArchive, class... Types>
void serialize(OutputArchive& ar, const variant<Types...>& v)
{
  // serialize the index
  ar(v.index());

  // serialize the value
  visit(serialize_visitor<OutputArchive>{ar}, v);
}


template<size_t i, class InputArchive, class... Ts, __REQUIRES(i == sizeof...(Ts))>
void deserialize_variant_impl(InputArchive& ar, size_t index, variant<Ts...>& v)
{
  throw std::runtime_error("deserialize(InputArchive,variant): invalid index.");
}

template<size_t i, class InputArchive, class... Ts, __REQUIRES(i < sizeof...(Ts))>
void deserialize_variant_impl(InputArchive& ar, size_t index, variant<Ts...>& v)
{
  if(index == i)
  {
    variant_alternative_t<i, variant<Ts...>> value;
    ar(value);

    v = std::move(value);
  }
  else
  {
    deserialize_variant_impl<i+1>(ar, index, v);
  }
}

template<class InputArchive, class... Types>
void deserialize(InputArchive& ar, variant<Types...>& v)
{
  // deserialize the index
  size_t index;
  ar(index);

  deserialize_variant_impl<0>(ar, index, v);
}


class output_archive
{
  private:
    // this is the terminal case of operator() above
    // it never needs to be called by a client
    inline void operator()() {}

    std::ostream& stream_;

  public:
    inline output_archive(std::ostream& os)
      : stream_(os)
    {}

    inline ~output_archive()
    {
      stream_.flush();
    }

    template<class Arg, class... Args>
    void operator()(const Arg& arg, const Args&... args)
    {
      serialize(*this, arg);

      (*this)(args...);
    }

    inline std::ostream& stream()
    {
      return stream_;
    }
};

class input_archive
{
  public:
    inline input_archive(std::istream& is)
      : stream_(is)
    {}

    template<class Arg, class... Args>
    void operator()(Arg& arg, Args&... args)
    {
      deserialize(*this, arg);

      (*this)(args...);
    }

    inline std::istream& stream()
    {
      return stream_;
    }

  private:
    // this is the terminal case of operator() above
    // it never needs to be called by a client
    inline void operator()() {}

    std::istream& stream_;
};


// put any in a namespace because the identifier collides
// with a function CUDA puts in the global namespace
namespace detail
{


class any;

template<class ValueType>
ValueType any_cast(const any& self);


class any
{
  public:
    any() = default;

    template<class T>
    any(T&& value)
    {
      std::stringstream os;

      {
        output_archive archive(os);

        archive(value);
      }

      representation_ = os.str();
    }

    template<class ValueType>
    friend ValueType any_cast(const any& self)
    {
      ValueType result;

      std::stringstream is(self.representation_);
      input_archive archive(is);

      archive(result);

      return result;
    }

  private:
    std::string representation_;
};

template<>
any any_cast(const any& self)
{
  return self;
}


template<class OutputArchive>
void serialize(OutputArchive& ar, const any& a)
{
  ar.stream() << a;
}

template<class InputArchive>
void deserialize(InputArchive& ar, any& a)
{
  ar.stream() >> a;
}


} // end detail


template<class... Conditions>
struct conjunction;

template<>
struct conjunction<> : std::true_type {};

template<class Condition, class... Conditions>
struct conjunction<Condition, Conditions...>
  : std::integral_constant<
      bool,
      Condition::value && conjunction<Conditions...>::value
    >
{};


template<class T>
std::string to_string(const T& value)
{
  std::stringstream os;
  output_archive ar(os);
  ar(value);
  return os.str();
}


template<class T>
T from_string(const char* string, std::size_t size)
{
  T result;

  string_view_stream is(string, size);
  input_archive ar(is);
  ar(result);

  return result;
}


template<class T>
T from_string(const char* string)
{
  return from_string<T>(string, std::strlen(string));
}

template<class T>
T from_string(const std::string& string)
{
  return from_string<T>(string.c_str());
}

