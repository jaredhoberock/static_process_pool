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

#include <utility>
#include <type_traits>

template<class Function, class... Args>
struct invoke_result
{
  using type = decltype(std::declval<Function>()(std::declval<Args>()...));
};

template<class Function, class... Args>
using invoke_result_t = typename invoke_result<Function,Args...>::type;


template<class Result, class Function, class... Args>
struct is_invocable_r_impl
{
  template<class F,
           class R = decltype(std::declval<F>()(std::declval<Args>()...)),
           class = typename std::enable_if<
             std::is_convertible<R,Result>::value
           >::type
          >
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<Function>(0));
};

template<class Result, class Function, class... Args>
using is_invocable_r = typename is_invocable_r_impl<Result, Function, Args...>::type;

template<class Function, class... Args>
struct is_invocable_impl
{
  template<class F,
           class Result = decltype(std::declval<F>()(std::declval<Args>()...))
          >
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<Function>(0));
};

template<class Function, class... Args>
using is_invocable = typename is_invocable_impl<Function, Args...>::type;

