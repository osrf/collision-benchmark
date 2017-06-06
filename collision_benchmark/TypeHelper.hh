/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Author: Jennifer Buehler
 * Date: December 2016
 */

#ifndef COLLISION_BENCHMARK_TYPEHELPER_H
#define COLLISION_BENCHMARK_TYPEHELPER_H

#include <string>
#include <typeinfo>

namespace collision_benchmark
{
std::string Demangle(const char* name);

template <typename T>
std::string GetTypeName(const T &t)
{
    return Demangle(typeid(t).name());
}

template <typename T>
std::string GetTypeName()
{
    return Demangle(typeid(T).name());
}

// private use for InstantiationOf
template <typename T,
          template <typename...> class Templated>
struct InstantiationOfImpl : std::false_type {};

// private use for InstantiationOf
template <template <typename...> class T,
          typename... Ts>
struct InstantiationOfImpl<T<Ts...>, T> : std::true_type {};

/**
 * Can be used to find out whether a type is of a template type.
 * Usage: If you want to see if a class is an instance of a templated
 * type, e.g. of std::shared_ptr, call as follows:
 *  InstantiationOf<Your-Type, std::shared_ptr>::value.
 *
 * Or Use static_assert to generate errors at compile time:
 * static_assert
 *    (collision_benchmark::InstantiationOf<IntSet, std::shared_ptr>::value,
 *     "Not an instantiation of required template type");
 *
 */
template <typename T,
          template <typename...> class Templated>
using InstantiationOf =
      InstantiationOfImpl<typename std::remove_reference<T>::type, Templated>;

/**
 * Shortcut for InstantiationOf<T, Templated>::value;
 */
template <typename T,
          template <typename...> class Templated>
inline bool is_InstantiationOf()
{
  return InstantiationOf<T, Templated>::value;
}

// private use by IsBaseOfTemplate
template <template <typename...> class C, typename...Ts>
std::true_type IsBaseOfTemplateImpl(const C<Ts...>*);

// private use by IsBaseOfTemplate
template <template <typename...> class C>
std::false_type IsBaseOfTemplateImpl(...);

/**
 * \brief Implementation of std::is_base_of for template classes
 * with some limitations.
 * For example, if we have a template<typename X> class A{} and a class B
 * that derives from it (complete type known), we can do something like:
 * if (IsBaseOfTemplate<A, B<int>>()) { ... B is derived from A }.
 * Limitation: Won't work for multiple inheritance (if B derives from
 * another class X, then IsBaseOfTemplate<X, B<int>> won't work), and will
 * only work for public inheritance (will generate compiler error).
 */
template <template <typename...> class C, typename T>
using IsBaseOfTemplate = decltype(IsBaseOfTemplateImpl<C>(std::declval<T*>()));

}  // namespace

#endif  // COLLISION_BENCHMARK_TYPEHELPER_H
