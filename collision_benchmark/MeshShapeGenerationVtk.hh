#ifndef COLLISION_BENCHMARK_MESHSHAPEGENERATIONVTK_H
#define COLLISION_BENCHMARK_MESHSHAPEGENERATIONVTK_H

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vector>

namespace collision_benchmark
{

/**
 * \param[in] radius radius of sphere
 * \param[in] theta number of points in the longitude direction
 * \param[in] phi number of points in the latitude direction
 * \param[in] latLongTessel Cause the sphere to be tessellated with edges along
 *    the latitude and longitude lines. If off, triangles are generated at
 *    non-polar regions, which results in edges that are not parallel to
 *    latitude and longitude lines. This can be useful for generating a
 *    wireframe sphere with natural latitude and longitude lines.
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeSphereVtk(const double radius,
                                           const unsigned int theta,
                                           const unsigned int phi,
                                           const bool latLongTessel);
/**
 * \param[in] radius radius of cylinder
 * \param[in] height height of cylinder
 * \param[in] resolution number of facets used to define cylinder.
 * \param[in] Turn on/off whether to cap cylinder with polygons.
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeCylinderVtk(const double radius,
                                             const double height,
                                             const unsigned int resolution,
                                             const bool capping);
/**
 * \param[in] x x dimension
 * \param[in] y y dimension
 * \param[in] z z dimension
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeBoxVtk(const double x,
                                        const double y,
                                        const double z);
/**
 * \brief Create a box using AABB cornder coordinates
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeBoxVtk(const double xMin, const double xMax,
                                        const double yMin, const double yMax,
                                        const double zMin, const double zMax);
/**
 * \brief Creates a cone
 * \param[in] radius base radius of the cone.
 * \param[in] height height of cone in its specified direction.
 * \param[in] resolution number of facets used to represent the cone.
 * \param[in] dir_x along with \e dir_y and \e dir_z: the orientation vector
 *    of the cone. The vector does not have to be normalized.
 *    The direction goes from the center of the base toward the apex.
 * \param[in] angle_deg angle of the cone. This is the angle between the axis of
 *    the cone and a generatrix. Warning: this is not the aperture!
 *    The aperture is twice this angle.
 *    As a side effect, the angle plus height sets the base radius of the cone.
 *    Angle is expressed in degrees.
 * \param[in] capping whether to cap the base of the cone with a polygon.
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeConeVtk(const double radius,
                                         const double height,
                                         const unsigned int resolution,
                                         const double angle_deg,
                                         const bool capping,
                                         const double dir_x=0,
                                         const double dir_y=0,
                                         const double dir_z=1);
/**
 * \param[in] innerRadius the inner radius
 * \param[in] outerRadius the outer radius
 * \param[in] radialResolution number of points in radius direction.
 * \param[in] circumResolution number of points in circumferential direction.
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeDiskVtk(const double innerRadius,
                                         const double outerRadius,
                                         const unsigned int radialResolution,
                                         const unsigned int circumResolution);
/**
 * \param[in] xRad radius in x-direction
 * \param[in] yRad radius in y-direction
 * \param[in] zRad radius in z-direction
 * \param[in] uRes resolution in u-direction
 * \param[in] vRes resolution in v-direction
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeEllipsoidVtk(const double xRad,
                                              const double yRad,
                                              const double zRad,
                                              const unsigned int uRes,
                                              const unsigned int vRes);
/**
 * \param[in] ringRadius radius from the center to the middle of the
 *    ring of the torus.
 * \param[in] crossRadius radius of the cross section of ring of the torus.
 * \param[in] uRes resolution in u-direction. Will create uRes-1 "circle
 *    sections", e.g. with uRes=4 it will be a triangle-shaped torus.
 * \param[in] vRes resolution in v-direction
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeTorusVtk(const double ringRadius,
                                          const double crossRadius,
                                          const unsigned int uRes,
                                          const unsigned int vRes);
// Simple helper for a point
struct vPoint
{
  double x,y,z;
};

// Simple helper for a triangle index set
struct vTriIdx
{
  unsigned int v1, v2, v3;
};

/**
 * \brief Gets the triangle soup out of the polygon data.
 * Will triangulate \e polydata first.
 * \param[in] polydata the polygon data
 * \param[out] vertices the vertices
 * \param[out] triangles the triangles
 */
void getTriangleSoup(const vtkSmartPointer<vtkPolyData>& polydata,
                     std::vector<vPoint>& vertices,
                     std::vector<vTriIdx>& triangles);


/**
 * Triangulates the data and returns the triangulated data
 */
vtkSmartPointer<vtkPolyData>
  triangulate(const vtkSmartPointer<vtkPolyData>& polydata);

// test method
// void testMeshShapeGenerationVtk();

}  // namespace
#endif  // COLLISION_BENCHMARK_MESHSHAPEGENERATIONVTK_H
