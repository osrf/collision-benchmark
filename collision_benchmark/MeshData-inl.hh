#include <collision_benchmark/MeshData.hh>

#include <random>

template<typename VP, int FS>
void collision_benchmark::MeshData<VP, FS>::Perturb(const double min,
                                                    const double max,
                                                    const Vertex& center)
{
  static std::random_device r;
  static std::default_random_engine randEngine(r());

  // to generate random doubles between min and max with a uniform distribution
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

template<typename VP, int FS>
void collision_benchmark::MeshData<VP, FS>::Perturb(const double min,
                                                    const double max,
                                                    const Vertex& center,
                                                    const Vertex& dir)
{
  assert(dir.Length() > 1e-04);

  static std::random_device r;
  static std::default_random_engine randEngine(r());

  // normalized direction (just to be sure it is normal)
  Vertex dirNorm = dir;
  dirNorm.Normalize();

  // to generate random doubles between min and max with a uniform distribution
  std::uniform_real_distribution<double> dist(min, max);

  for (typename std::vector<Vertex>::iterator it = verts.begin();
       it != verts.end(); ++it)
  {
    Vertex& v = *it;

    // determine move direction which is orthogonal to the given line
    Vertex fromCenter = v - center;
    Vertex proj = dirNorm * (dirNorm.Dot(fromCenter));
    Vertex moveDir = fromCenter - proj;
    if (moveDir.Length() < 1e-02) continue;
    moveDir.Normalize();

    // if the move direction is not orthogonal to dir then it must
    // be on the line itself, in which case we won't perturb it.
    if (fabs(moveDir.Dot(dirNorm)) > 1e-04)
    {
      continue;
    }

    double randDisplace = dist(randEngine);
    v += moveDir * randDisplace;
  }
}
