

#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkDiskSource.h>

#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkParametricTorus.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkTriangleFilter.h>
#include <vtkAlgorithmOutput.h>

// for visualization
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>


void test(vtkPolyData* polydata)
{
  std::cout << "There are " << polydata->GetNumberOfPoints()
            << " points." << std::endl;
  std::cout << "There are " << polydata->GetNumberOfPolys()
            << " triangles." << std::endl;

  for(vtkIdType i = 0; i < polydata->GetNumberOfPoints(); i++)
  {
    // This is identical to:
    // polydata->GetPoints()->GetPoint(i,p);
    double p[3];
    polydata->GetPoint(i,p);
    std::cout << "Point " << i << " : (" << p[0] << " " << p[1] << " " << p[2] << ")" << std::endl;
  }

  // Write all of the coordinates of the points in the vtkPolyData to the console.
  polydata->GetPolys()->InitTraversal();
  vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
  while(polydata->GetPolys()->GetNextCell(idList))
  // for(vtkIdType i = 0; i < polydata->GetNumberOfPolys(); i++)
  {
    // std::cout << "Size: "<< polydata->GetPolys()->GetSize() << std::endl;
    std::cout << "Poly has " << idList->GetNumberOfIds() << " points." << std::endl;

    for(vtkIdType pointId = 0; pointId < idList->GetNumberOfIds(); pointId++)
    {
      std::cout << idList->GetId(pointId) << " ";
    }
    std::cout << std::endl;
  }
}

vtkSmartPointer<vtkPolyData> triangulate(vtkAlgorithmOutput* data)
{
  // cylinders are no triangles. Apply the filter.
  vtkSmartPointer<vtkTriangleFilter> triangleFilter =
        vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputConnection(data);
  triangleFilter->Update();
  return triangleFilter->GetOutput();
}


/**
 * \param[in] theta number of points in the longitude direction
 * \param[in] phi number of points in the latitude direction
 * \param[in] latLongTessel Cause the sphere to be tessellated with edges along
 *    the latitude and longitude lines. If off, triangles are generated at
 *    non-polar regions, which results in edges that are not parallel to
 *    latitude and longitude lines. This can be useful for generating a
 *    wireframe sphere with natural latitude and longitude lines.
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeSphere(const unsigned int theta,
                                        const unsigned int phi,
                                        const bool latLongTessel)
{
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();

  sphereSource->SetThetaResolution(theta);
  sphereSource->SetPhiResolution(phi);
  sphereSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);
  sphereSource->SetLatLongTessellation(latLongTessel);

  // does the generation after re-setting the parameters
  sphereSource->Update();

  vtkSmartPointer<vtkPolyData> polySphere =
    triangulate(sphereSource->GetOutputPort());
  return polySphere;
}


/**
 * \param[in] radius radius of cylinder
 * \param[in] height height of cylinder
 * \param[in] resolution number of facets used to define cylinder.
 * \param[in] Turn on/off whether to cap cylinder with polygons.
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeCylinder(const double radius,
                                          const double height,
                                          const unsigned int resolution,
                                          const bool capping)
{

  vtkSmartPointer<vtkCylinderSource> cylinderSource =
      vtkSmartPointer<vtkCylinderSource>::New();
  cylinderSource->SetCenter(0.0, 0.0, 0.0);
  cylinderSource->SetRadius(radius);
  cylinderSource->SetHeight(height);
  cylinderSource->SetResolution(resolution);
  cylinderSource->SetCapping(capping);
  cylinderSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);

  // does the generation after re-setting the parameters
  cylinderSource->Update();

  vtkSmartPointer<vtkPolyData> polyCylinder =
    triangulate(cylinderSource->GetOutputPort());
  return polyCylinder;
}

/**
 * \param[in] x x dimension
 * \param[in] y y dimension
 * \param[in] z z dimension
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeBox(const double x,
                                     const double y,
                                     const double z)
{
  vtkSmartPointer<vtkCubeSource> boxSource =
      vtkSmartPointer<vtkCubeSource>::New();
  boxSource->SetCenter(0.0, 0.0, 0.0);
  boxSource->SetXLength(x);
  boxSource->SetYLength(y);
  boxSource->SetZLength(z);
  // boxSource->SetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
  boxSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);

  // does the generation after re-setting the parameters
  boxSource->Update();

  vtkSmartPointer<vtkPolyData> polyBox =
    triangulate(boxSource->GetOutputPort());
  return polyBox;
}

/**
 * \brief Create a box using AABB cornder coordinates
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeBox(const double xMin, const double xMax,
                                     const double yMin, const double yMax,
                                     const double zMin, const double zMax)
{
  vtkSmartPointer<vtkCubeSource> boxSource =
      vtkSmartPointer<vtkCubeSource>::New();
  boxSource->SetCenter(0.0, 0.0, 0.0);
  boxSource->SetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
  boxSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);

  // does the generation after re-setting the parameters
  boxSource->Update();

  vtkSmartPointer<vtkPolyData> polyBox =
    triangulate(boxSource->GetOutputPort());
  return polyBox;
}


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
vtkSmartPointer<vtkPolyData> makeCone(const double radius,
                                      const double height,
                                      const unsigned int resolution,
                                      const double angle_deg,
                                      const bool capping,
                                      const double dir_x=0,
                                      const double dir_y=0,
                                      const double dir_z=1)
{
  vtkSmartPointer<vtkConeSource> coneSource =
      vtkSmartPointer<vtkConeSource>::New();
  coneSource->SetCenter(0.0, 0.0, 0.0);
  coneSource->SetRadius(radius);
  coneSource->SetHeight(height);
  coneSource->SetResolution(resolution);
  coneSource->SetDirection(dir_x, dir_y, dir_z);
  coneSource->SetAngle(angle_deg);
  coneSource->SetCapping(capping);
  coneSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);

  // does the generation after re-setting the parameters
  coneSource->Update();

  vtkSmartPointer<vtkPolyData> polyCone =
    triangulate(coneSource->GetOutputPort());
  return polyCone;
}



/**
 * \param[in] innerRadius the inner radius
 * \param[in] outerRadius the outer radius
 * \param[in] radialResolution number of points in radius direction.
 * \param[in] circumResolution number of points in circumferential direction.
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeDisk(const double innerRadius,
                                      const double outerRadius,
                                      const unsigned int radialResolution,
                                      const unsigned int circumResolution)
{
  vtkSmartPointer<vtkDiskSource> diskSource =
      vtkSmartPointer<vtkDiskSource>::New();
  diskSource->SetInnerRadius(innerRadius);
  diskSource->SetOuterRadius(outerRadius);
  diskSource->SetRadialResolution(radialResolution);
  diskSource->SetCircumferentialResolution(circumResolution);

  diskSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);

  // does the generation after re-setting the parameters
  diskSource->Update();

  vtkSmartPointer<vtkPolyData> polyDisk =
    triangulate(diskSource->GetOutputPort());
  return polyDisk;
}



/**
 * \param[in] xRad radius in x-direction
 * \param[in] yRad radius in y-direction
 * \param[in] zRad radius in z-direction
 * \param[in] uRes resolution in u-direction
 * \param[in] vRes resolution in v-direction
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeEllipsoid(const double xRad,
                                           const double yRad,
                                           const double zRad,
                                           const unsigned int uRes,
                                           const unsigned int vRes)
{
  vtkSmartPointer<vtkParametricEllipsoid> ellipsoid =
      vtkSmartPointer<vtkParametricEllipsoid>::New();
  ellipsoid->SetXRadius(xRad);
  ellipsoid->SetYRadius(yRad);
  ellipsoid->SetZRadius(zRad);

  vtkSmartPointer<vtkParametricFunctionSource> ellipsoidSource =
      vtkSmartPointer<vtkParametricFunctionSource>::New();
  ellipsoidSource->SetParametricFunction(ellipsoid);
  ellipsoidSource->SetUResolution(uRes);
  ellipsoidSource->SetVResolution(vRes);
  ellipsoidSource->SetGenerateNormals(0);
  ellipsoidSource->SetGenerateTextureCoordinates(0);

  ellipsoidSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);

  // does the generation after re-setting the parameters
  ellipsoidSource->Update();

  vtkSmartPointer<vtkPolyData> polyEllipsoid =
    triangulate(ellipsoidSource->GetOutputPort());
  return polyEllipsoid;
}


/**
 * \param[in] ringRadius radius from the center to the middle of the
 *    ring of the torus.
 * \param[in] crossRadius radius of the cross section of ring of the torus.
 * \param[in] uRes resolution in u-direction. Will create uRes-1 "circle
 *    sections", e.g. with uRes=4 it will be a triangle-shaped torus.
 * \param[in] vRes resolution in v-direction
 * \return the polygon data
 */
vtkSmartPointer<vtkPolyData> makeTorus(const double ringRadius,
                                       const double crossRadius,
                                       const unsigned int uRes,
                                       const unsigned int vRes)
{
  vtkSmartPointer<vtkParametricTorus> torus =
      vtkSmartPointer<vtkParametricTorus>::New();
  torus->SetRingRadius(ringRadius);
  torus->SetCrossSectionRadius(crossRadius);

  vtkSmartPointer<vtkParametricFunctionSource> torusSource =
      vtkSmartPointer<vtkParametricFunctionSource>::New();
  torusSource->SetParametricFunction(torus);
  torusSource->SetUResolution(uRes);
  torusSource->SetVResolution(vRes);
  torusSource->SetGenerateNormals(0);
  torusSource->SetGenerateTextureCoordinates(0);

  torusSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);

  // does the generation after re-setting the parameters
  torusSource->Update();

  vtkSmartPointer<vtkPolyData> polyTorus =
    triangulate(torusSource->GetOutputPort());
  return polyTorus;
}

/**
 * \param[in]
 * \return the polygon data
 */
// vtkSmartPointer<vtkPolyData> makeXXX(const ...)


void visualize(const vtkSmartPointer<vtkPolyData>& polyData)
{
  // The mapper is responsible for pushing the geometry into the graphics library.
  // It may also do color mapping, if scalars or other attributes are defined.
  vtkSmartPointer<vtkPolyDataMapper> polyMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  polyMapper->SetInputData(polyData);

  // The actor is a grouping mechanism: besides the geometry (mapper), it
  // also has a property, transformation matrix, and/or texture map.
  vtkSmartPointer<vtkActor> polyActor =
    vtkSmartPointer<vtkActor>::New();
  polyActor->SetMapper(polyMapper);
  polyActor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
  polyActor->RotateX(30.0);
  polyActor->RotateY(-45.0);
  polyActor->GetProperty()->SetRepresentationToWireframe();

  // The renderer generates the image
  // which is then displayed on the render window.
  // It can be thought of as a scene to which the actor is added
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(polyActor);
  renderer->SetBackground(0.1, 0.2, 0.4);
  // Zoom in a little by accessing the camera and invoking its "Zoom" method.
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Zoom(1.5);

  // The render window is the actual GUI window
  // that appears on the computer screen
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(200, 200);
  renderWindow->AddRenderer(renderer);

  // The render window interactor captures mouse events
  // and will perform appropriate camera or actor manipulation
  // depending on the nature of the events.
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // This starts the event loop and as a side effect causes an initial render.
  renderWindowInteractor->Start();
}

int main(int, char*[])
{
  ////////////////// Primitives         /////////////////////////////////
  ///////////////////////////////////////////////////////////////////////
  // http://www.vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/GeometricObjectsDemo

  std::cout << " +++++++++++++ SPHERE +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polySphere = makeSphere(4,4,true);
  test(polySphere);

  std::cout << " +++++++++++++ CYLINDER+++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyCylinder = makeCylinder(1.0, 2.0, 5, true);
  test(polyCylinder);

  std::cout << " +++++++++++++ BOX +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyBox = makeBox(1.0,2.0,3.0);
  test(polyBox);

  std::cout << " +++++++++++++ CONE +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyCone = makeCone(2.0, 3.0, 5, 45, true);
  test(polyCone);

  std::cout << " +++++++++++++ DISK +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyDisk = makeDisk(1.0, 5.0, 1, 5);
  test(polyDisk);


  ////////////////// Parametric objects /////////////////////////////////
  ///////////////////////////////////////////////////////////////////////
  // see also http://www.vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/ParametricObjects
  ///////////////////////////////////////////////////////////////////////

  std::cout << " +++++++++++++ ELLIPSOID +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyEllipsoid = makeEllipsoid(1.0, 2.0, 3.0,
                                                             10, 10);
  test(polyEllipsoid);

  std::cout << " +++++++++++++ TORUS +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyTorus = makeTorus(1, 0.1, 10, 10);
  test(polyTorus);


  ////////////////// Test visualization /////////////////////////////////
  // visualize(polySphere);
  // visualize(polyCylinder);
  // visualize(polyBox);
  // visualize(polyCone);
  // visualize(polyDisk);
  visualize(polyEllipsoid);
  // visualize(polyTorus);
  return EXIT_SUCCESS;
}

