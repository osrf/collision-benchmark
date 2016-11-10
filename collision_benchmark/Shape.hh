#ifndef COLLISION_BENCHMARK_SHAPE
#define COLLISION_BENCHMARK_SHAPE

#include <boost/shared_ptr.hpp>

namespace collision_benchmark
{

/**
 * \brief Basic interface for a shape
 * \author Jennifer Buehler
 * \date October 2016
 */
class Shape
{

  public: Shape(){}
  private: Shape(const Shape& c){}
  public: virtual ~Shape(){}

};

typedef boost::shared_ptr<Shape> ShapePtr;

}  // namespace

#endif  // COLLISION_BENCHMARK_SHAPE
