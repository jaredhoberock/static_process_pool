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

#include "serialization.hpp"
#include <utility>
#include <type_traits>
#include <tuple>
#include <string>


template<class T>
struct can_serialize_impl
{
  template<class U,
           class Result = decltype(serialize(std::declval<output_archive&>(), std::declval<U>()))
          >
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<T>(0));
};

template<class T>
using can_serialize = typename can_serialize_impl<T>::type;

template<class... Ts>
using can_serialize_all = conjunction<can_serialize<Ts>...>;


template<class T>
struct can_deserialize_impl
{
  template<class U,
           class Result = decltype(deserialize(std::declval<input_archive&>(), std::declval<U&>()))
          >
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<T>(0));
};

template<class T>
using can_deserialize = typename can_deserialize_impl<T>::type;


template<class... Ts>
using can_deserialize_all = conjunction<can_deserialize<Ts>...>;



template<class Result, class... UnboundArgs>
class basic_serializable_function
{
  private:
    static_assert(std::is_void<Result>::value or can_deserialize<Result>::value, "Result must be void or deserializable.");

  public:
    basic_serializable_function() = default;

    template<class Function, class... BoundArgs,
             // must be able to serialize and deserialize all these constructor arguments
             __REQUIRES(can_serialize_all<Function,BoundArgs...>::value),
             __REQUIRES(can_deserialize_all<Function,BoundArgs...>::value),

             // the function needs to be invocable with the given args and return the expected result type
             __REQUIRES(
               is_invocable_r<Result,Function,BoundArgs...,UnboundArgs...>::value or
               is_invocable_r<void,Function,BoundArgs...,UnboundArgs...>::value and std::is_same<Result,any>::value
             )
            >
    explicit basic_serializable_function(Function func, BoundArgs... args)
      : serialized_(serialize_function_and_arguments(&deserialize_and_invoke<Function,BoundArgs...>, func, args...))
    {}

    explicit operator bool() const noexcept
    {
      return !serialized_.empty();
    }

    Result operator()(UnboundArgs... args) const
    {
      std::stringstream is(serialized_);
      input_archive archive(is);

      // extract a function_ptr_type from the beginning of the buffer
      using function_ptr_type = Result (*)(input_archive&, UnboundArgs...);
      function_ptr_type invoke_me = nullptr;

      archive(invoke_me);

      // invoke the function pointer on the remaining data
      return invoke_me(archive, args...);
    }

    template<class OutputArchive>
    friend void serialize(OutputArchive& ar, const basic_serializable_function& sf)
    {
      ar(sf.serialized_);
    }

    template<class InputArchive>
    friend void deserialize(InputArchive& ar, basic_serializable_function& sf)
    {
      ar(sf.serialized_);
    }

    friend std::istream& operator>>(std::istream& is, basic_serializable_function& sf)
    {
      return is >> sf.serialized_;
    }

    friend std::ostream& operator<<(std::ostream& os, const basic_serializable_function& sf)
    {
      return os << sf.serialized_;
    }

  private:
    template<class Function, class Tuple,
             __REQUIRES(
               std::is_convertible<
                 apply_result_t<Function, Tuple>, Result
               >::value
             )>
    static Result apply_and_return_result(Function&& f, Tuple&& t)
    {
      return apply(std::forward<Function>(f), std::forward<Tuple>(t));
    }

    template<class Function, class Tuple,
             __REQUIRES(
               std::is_void<apply_result_t<Function, Tuple>>::value and
               std::is_same<Result, any>::value
             )>
    static any apply_and_return_result(Function&& f, Tuple&& t)
    {
      apply(std::forward<Function>(f), std::forward<Tuple>(t));

      // an empty any object stands in for a void result
      return any{};
    }

    template<class Function, class... BoundArgs>
    static Result deserialize_and_invoke(input_archive& archive, UnboundArgs... unbound_args)
    {
      // deserialize function and its arguments which were bound at construction time
      std::tuple<Function,BoundArgs...> function_and_bound_args;
      archive(function_and_bound_args);

      // split tuple into a function and already bound arguments
      Function f = std::get<0>(function_and_bound_args);
      std::tuple<BoundArgs...> bound_arguments = tail(function_and_bound_args);

      // concatenate the bound and unbounded arguments into a single list of arguments
      std::tuple<BoundArgs...,UnboundArgs...> arguments = std::tuple_cat(bound_arguments, std::tuple<UnboundArgs...>(unbound_args...));

      return apply_and_return_result(f, arguments);
    }

    template<class Function, class... Args>
    static std::string serialize_function_and_arguments(const Function& f, const Args&... args)
    {
      std::stringstream os;

      {
        output_archive archive(os);

        // serialize the function and its arguments into the archive
        archive(f, args...);
      }

      return os.str();
    }

    std::string serialized_;
};

using serializable_function = basic_serializable_function<any>;

