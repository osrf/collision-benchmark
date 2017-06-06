#ifndef COLLISION_BENCHMARK_EXCEPTION
#define COLLISION_BENCHMARK_EXCEPTION

#include <stdexcept>
#include <string>
#include <sstream>

#define THROW_EXCEPTION(stream) \
{ \
    std::stringstream str; \
    str << stream<< ": " << __FILE__ << ", " << __LINE__; \
    throw collision_benchmark::Exception(str.str()); \
}

namespace collision_benchmark
{

class Exception : public std::runtime_error
{
  public:
    // Constructor.
    Exception(const std::string &description):
      std::runtime_error(description) {}
};

}

#endif   // COLLISION_BENCHMARK_EXCEPTION
