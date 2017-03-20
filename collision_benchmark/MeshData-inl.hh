#include <collision_benchmark/MeshData.hh>

#include <random>

template<typename VP, int FS>
void collision_benchmark::MeshData<VP, FS>::Perturb(const double min,
                                                    const double max,
                                                    const Vertex& center)
{
  static std::random_device r;
  // Choose a random mean between 1 and 6
  static std::default_random_engine randEngine(r());
  std::uniform_real_distribution<double> dist(min, max);

  for (typename std::vector<Vertex>::iterator it = verts.begin();
       it != verts.end(); ++it)
  {
    Vertex& v = *it;
    Vertex diff = v - center;
    diff.Normalize();
    double randDisplace = dist(randEngine);
    v += diff * randDisplace;
  }
}
