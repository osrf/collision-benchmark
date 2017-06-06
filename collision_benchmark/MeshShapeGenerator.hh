#ifndef COLLISION_BENCHMARK_MESHSHAPEGENERATOR_H
#define COLLISION_BENCHMARK_MESHSHAPEGENERATOR_H

#include <collision_benchmark/MeshData.hh>

#include <memory>

namespace collision_benchmark
{

/**
 * \brief Interface to generate triangle mesh representations out of shapes.
 *
 * All shapes are always at origin position (0,0,0) and at default orientation
 * (identity quaternion).
 *
 * \author Jennifer Buehler
 * \date March 2017
 */
template<typename VertexPrecision_ = float>
class MeshShapeGenerator
{
  public: typedef VertexPrecision_ VertexPrecision;
  private: typedef MeshShapeGenerator<VertexPrecision> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: typedef MeshData<VertexPrecision, 3> TriMeshData;
  public: typedef typename TriMeshData::Ptr TriMeshDataPtr;

  /**
   * \brief Makes a sphere.
   * \param[in] radius radius of sphere
   * \param[in] theta number of points in the longitude direction
   * \param[in] phi number of points in the latitude direction
   * \param[in] latLongTessel Cause the sphere to be tessellated with edges
   *    along the latitude and longitude lines. If off, triangles are generated
   *    at non-polar regions, which results in edges that are not parallel to
   *    latitude and longitude lines. This can be useful for generating a
   *    wireframe sphere with natural latitude and longitude lines.
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeSphere(const double radius,
                                    const unsigned int theta,
                                    const unsigned int phi,
                                    const bool latLongTessel = true) const = 0;
  /**
   * \brief Makes a cylinder along the y axis.
   * \param[in] radius radius of cylinder
   * \param[in] height height of cylinder
   * \param[in] resolution number of facets used to define cylinder.
   * \param[in] Turn on/off whether to cap cylinder with polygons.
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeCylinder(const double radius,
                                              const double height,
                                              const unsigned int resolution,
                                              const bool capping) const = 0;
  /**
   * \brief Makes a box.
   * \param[in] x x dimension
   * \param[in] y y dimension
   * \param[in] z z dimension
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeBox(const double x,
                                         const double y,
                                         const double z) const = 0;
  /**
   * \brief Makes a box.
   * \brief Create a box using AABB cornder coordinates
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeBox(const double xMin, const double xMax,
                                         const double yMin, const double yMax,
                                         const double zMin,
                                         const double zMax) const = 0;
  /**
   * \brief Creates a cone with tip at the origin, extending along the x axis.
   * \param[in] radius base radius of the cone.
   * \param[in] height height of cone in its specified direction.
   * \param[in] resolution number of facets used to represent the cone.
   * \param[in] dir_x along with \e dir_y and \e dir_z: the orientation vector
   *    of the cone. The vector does not have to be normalized.
   *    The direction goes from the center of the base toward the apex.
   * \param[in] angle_deg angle of the cone. This is the angle between the axis
   *    of the cone and a generatrix. Warning: this is not the aperture!
   *    The aperture is twice this angle.
   *    As a side effect, the angle plus height sets the base radius of the
   *    cone. Angle is expressed in degrees.
   * \param[in] capping whether to cap the base of the cone with a polygon.
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeCone(const double radius,
                                           const double height,
                                           const unsigned int resolution,
                                           const double angle_deg,
                                           const bool capping,
                                           const double dir_x = 0,
                                           const double dir_y = 0,
                                           const double dir_z = 1) const = 0;
  /**
   * \brief Makes a disk.
   * \param[in] innerRadius the inner radius
   * \param[in] outerRadius the outer radius
   * \param[in] radialRes number of points in radius direction.
   * \param[in] circumRes number of points in circumferential direction.
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeDisk(const double innerRadius,
                                       const double outerRadius,
                                       const unsigned int radialRes,
                                       const unsigned int circumRes) const = 0;
  /**
   * \brief Makes an ellipsoid.
   * \param[in] xRad radius in x-direction
   * \param[in] yRad radius in y-direction
   * \param[in] zRad radius in z-direction
   * \param[in] uRes resolution in u-direction
   * \param[in] vRes resolution in v-direction
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeEllipsoid(const double xRad,
                                            const double yRad,
                                            const double zRad,
                                            const unsigned int uRes,
                                            const unsigned int vRes) const = 0;
  /**
   * \brief Makes a torus.
   * \param[in] ringRadius radius from the center to the middle of the
   *    ring of the torus.
   * \param[in] crossRadius radius of the cross section of ring of the torus.
   * \param[in] uRes resolution in u-direction. Will create uRes-1 "circle
   *    sections", e.g. with uRes = 4 it will be a triangle-shaped torus.
   * \param[in] vRes resolution in v-direction
   * \return the triangle mesh data
   */
  public: virtual TriMeshDataPtr MakeTorus(const double ringRadius,
                                           const double crossRadius,
                                           const unsigned int uRes,
                                           const unsigned int vRes) const = 0;
};  // class

}  // namespace
#endif  // COLLISION_BENCHMARK_MESHSHAPEGENERATOR_H
