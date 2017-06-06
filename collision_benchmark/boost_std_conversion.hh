/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef COLLISION_BENCHMARK_BOOST_STD_CONVERSION_H
#define COLLISION_BENCHMARK_BOOST_STD_CONVERSION_H

#include <algorithm>
#include <memory>
#include <boost/shared_ptr.hpp>

namespace collision_benchmark
{
namespace
{
template<class SharedPointer> struct Holder
{
  SharedPointer p;

  explicit Holder(const SharedPointer &p)
    : p(p) {}
  Holder(const Holder &other)
    : p(other.p) {}
  Holder(Holder &&other)
    : p(std::move(other.p)) {}

  void operator()(...) { p.reset(); }
};
}

template<class T> std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> &p)
{
  typedef Holder<std::shared_ptr<T>> H;

  if (H *h = boost::get_deleter<H, T>(p))
  {
    return h->p;
  }
  else
  {
    return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T> >(p));
  }
}

template<class T> boost::shared_ptr<T> to_boost_ptr(const std::shared_ptr<T> &p)
{
  typedef Holder<boost::shared_ptr<T>> H;

  if (H *h = std::get_deleter<H, T>(p))
  {
    return h->p;
  }
  else
  {
    return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T> >(p));
  }
}

}  // namespace

#endif  // COLLISION_BENCHMARK_BOOST_STD_CONVERSION_H
