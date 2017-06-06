#ifndef COLLISION_BENCHMARK_MESHSHAPEGENERATORVTK_INL_H
#define COLLISION_BENCHMARK_MESHSHAPEGENERATORVTK_INL_H

#include <collision_benchmark/MeshShapeGenerationVtk.hh>

namespace collision_benchmark
{


template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
GetTriMeshFromVtkPoly(const vtkSmartPointer<vtkPolyData>& poly)
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshData TriMeshData;
  typedef typename TriMeshData::Ptr TriMeshDataPtr;
  typedef std::vector<typename TriMeshData::Vertex> TriMeshVerts_V;
  typedef std::vector<typename TriMeshData::Face> TriMeshFaces_V;

  std::vector<collision_benchmark::vPoint> vtkPoints;
  std::vector<collision_benchmark::vTriIdx> vtkFaces;
  collision_benchmark::getTriangleSoup(poly, vtkPoints, vtkFaces);

  TriMeshDataPtr ret(new TriMeshData());
  TriMeshVerts_V &verts = ret->GetVertices();
  for (std::vector<collision_benchmark::vPoint>::const_iterator
       it = vtkPoints.begin(); it != vtkPoints.end(); ++it)
  {
    const collision_benchmark::vPoint &p = *it;
    verts.push_back(typename TriMeshData::Vertex(p.x, p.y, p.z));
  }

  TriMeshFaces_V &faces = ret->GetFaces();
  for (std::vector<collision_benchmark::vTriIdx>::const_iterator
       it = vtkFaces.begin(); it != vtkFaces.end(); ++it)
  {
    const collision_benchmark::vTriIdx &f = *it;
    faces.push_back(typename TriMeshData::Face(f.v1, f.v2, f.v3));
  }
  return ret;
}


template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeSphere(const double radius,
                                      const unsigned int theta,
                                      const unsigned int phi,
                                      const bool latLongTessel) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> sphere =
    collision_benchmark::makeSphereVtk(radius, theta, phi, latLongTessel);
  return GetTriMeshFromVtkPoly<VP>(sphere);
}

template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeCylinder(const double radius,
                                        const double height,
                                        const unsigned int resolution,
                                        const bool capping) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> cylinder =
    collision_benchmark::makeCylinderVtk(radius, height, resolution, capping);
  return GetTriMeshFromVtkPoly<VP>(cylinder);
}

template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeBox(const double x,
                                   const double y,
                                   const double z) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> box =
    collision_benchmark::makeBoxVtk(x, y, z);
  return GetTriMeshFromVtkPoly<VP>(box);
}

template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeBox(const double xMin, const double xMax,
                                   const double yMin, const double yMax,
                                   const double zMin, const double zMax) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> box =
    collision_benchmark::makeBoxVtk(xMin, xMax, yMin, yMax, zMin, zMax);
  return GetTriMeshFromVtkPoly<VP>(box);
}

template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeCone(const double radius,
                                    const double height,
                                    const unsigned int resolution,
                                    const double angle_deg,
                                    const bool capping,
                                    const double dir_x,
                                    const double dir_y,
                                    const double dir_z) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> cone =
    collision_benchmark::makeConeVtk(radius, height, resolution,
                                       angle_deg, capping, dir_x, dir_y, dir_z);
  return GetTriMeshFromVtkPoly<VP>(cone);
}

template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeDisk(const double innerRadius,
                                    const double outerRadius,
                                    const unsigned int radialRes,
                                    const unsigned int circumRes) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> disk =
    collision_benchmark::makeDiskVtk(innerRadius, outerRadius,
                                     radialRes, circumRes);
  return GetTriMeshFromVtkPoly<VP>(disk);
}

template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeEllipsoid(const double xRad,
                                         const double yRad,
                                         const double zRad,
                                         const unsigned int uRes,
                                         const unsigned int vRes) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> ellipsoid =
    collision_benchmark::makeEllipsoidVtk(xRad, yRad, zRad, uRes, vRes);
  return GetTriMeshFromVtkPoly<VP>(ellipsoid);
}

template<typename VP>
typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr
MeshShapeGeneratorVtk<VP>::MakeTorus(const double ringRadius,
                                     const double crossRadius,
                                     const unsigned int uRes,
                                     const unsigned int vRes) const
{
  typedef typename MeshShapeGeneratorVtk<VP>::TriMeshDataPtr TriMeshDataPtr;
  vtkSmartPointer<vtkPolyData> torus =
    collision_benchmark::makeTorusVtk(ringRadius, crossRadius, uRes, vRes);
  return GetTriMeshFromVtkPoly<VP>(torus);
}

}  // namespace
#endif  // COLLISION_BENCHMARK_MESHSHAPEGENERATORVTK_INL_H
