
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
/*
 * Author: Jennifer Buehler
 * Date: March 2017
 */

#include <collision_benchmark/MeshShapeGenerationVtk.hh>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkDiskSource.h>

#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkParametricTorus.h>

#include <vtkCleanPolyData.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkTriangleFilter.h>
#include <vtkAlgorithmOutput.h>

#include <iostream>

vtkSmartPointer<vtkPolyData>
collision_benchmark::triangulate(const vtkSmartPointer<vtkPolyData>& polydata)
{
  vtkSmartPointer<vtkTriangleFilter> triangleFilter =
        vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputData(polydata);
  triangleFilter->Update();
  return triangleFilter->GetOutput();
}

void collision_benchmark::getTriangleSoup
    (const vtkSmartPointer<vtkPolyData>& polydata,
     std::vector<vPoint>& points,
     std::vector<vTriIdx>& faces)
{
  points.clear();
  faces.clear();

  vtkSmartPointer<vtkPolyData> tridataRaw = triangulate(polydata);

/*  std::cout << "There are " << tridata->GetNumberOfPoints()
            << " points." << std::endl;
  std::cout << "There are " << tridata->GetNumberOfPolys()
            << " triangles." << std::endl;*/

  // also remove duplicate vertices
  vtkSmartPointer<vtkCleanPolyData> tridataClean =
           vtkSmartPointer<vtkCleanPolyData>::New();
  tridataClean->SetInputData(tridataRaw);
  tridataClean->Update();
  vtkSmartPointer<vtkPolyData> tridata = tridataClean->GetOutput();

  for (vtkIdType i = 0; i < tridata->GetNumberOfPoints(); i++)
  {
    // This is identical to:
    // tridata->GetPoints()->GetPoint(i, p);
    double p[3];
    tridata->GetPoint(i, p);
    vPoint point;
    point.x = p[0];
    point.y = p[1];
    point.z = p[2];
    points.push_back(point);
  }

  // Write all of the coordinates of the points in the
  // vtkPolyData to the console.
  tridata->GetPolys()->InitTraversal();
  vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
  while (tridata->GetPolys()->GetNextCell(idList))
  {
    if (idList->GetNumberOfIds() != 3)
    {
      std::cerr << "INCONSISTENCY: Triangle should have 3 vertex indices! "
                << std::endl;
      continue;
    }
    vTriIdx face;
    face.v1 = idList->GetId(0);
    face.v2 = idList->GetId(1);
    face.v3 = idList->GetId(2);
    faces.push_back(face);
  }
}

void printTriangleSoup
    (const std::vector<collision_benchmark::vPoint>& points,
     const std::vector<collision_benchmark::vTriIdx>& faces)
{
  for (std::vector<collision_benchmark::vPoint>::const_iterator
       it = points.begin(); it != points.end(); ++it)
  {
    const collision_benchmark::vPoint &p = *it;
    std::cout << "Point " << p.x << ", " << p.y << ", " << p.z << std::endl;
  }

  for (std::vector<collision_benchmark::vTriIdx>::const_iterator
       it = faces.begin(); it != faces.end(); ++it)
  {
    const collision_benchmark::vTriIdx &f = *it;
    std::cout << "Face " << f.v1 << ", " << f.v2 << ", " << f.v3 << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeSphereVtk(const double radius,
                                   const unsigned int theta,
                                   const unsigned int phi,
                                   const bool latLongTessel)
{
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(radius);
  sphereSource->SetThetaResolution(theta);
  sphereSource->SetPhiResolution(phi);
  sphereSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);
  sphereSource->SetLatLongTessellation(latLongTessel);

  // does the generation after re-setting the parameters
  sphereSource->Update();

  return sphereSource->GetOutput();
}


///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeCylinderVtk(const double radius, const double height,
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

  return cylinderSource->GetOutput();
}

///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeBoxVtk(const double x, const double y, const double z)
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

  return boxSource->GetOutput();
}

///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeBoxVtk(const double xMin, const double xMax,
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

  return boxSource->GetOutput();
}


///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeConeVtk(const double radius, const double height,
            const unsigned int resolution, const double angle_deg,
            const bool capping, const double dir_x,
            const double dir_y, const double dir_z)
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

  return coneSource->GetOutput();
}

///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeDiskVtk(const double innerRadius,
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

  return diskSource->GetOutput();
}


///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeEllipsoidVtk(const double xRad,
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

  return ellipsoidSource->GetOutput();
}

///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkPolyData>
collision_benchmark::makeTorusVtk(const double ringRadius,
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

  return torusSource->GetOutput();
}


/*
void collision_benchmark::testMeshShapeGenerationVtk()
{
  std::vector<collision_benchmark::vPoint> points;
  std::vector<collision_benchmark::vTriIdx> faces;

  ////////////////// Primitives         /////////////////////////////////
  ///////////////////////////////////////////////////////////////////////
  // http://www.vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/GeometricObjectsDemo

  std::cout << " +++++++++++++ SPHERE +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polySphere = collision_benchmark::makeSphereVtk(4,4,true);
  collision_benchmark::getTriangleSoup(polySphere, points, faces);
  printTriangleSoup(points, faces);

  std::cout << " +++++++++++++ CYLINDER+++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyCylinder = collision_benchmark::makeCylinderVtk(1.0, 2.0, 5, true);
  collision_benchmark::getTriangleSoup(polyCylinder, points, faces);
  printTriangleSoup(points, faces);

  std::cout << " +++++++++++++ BOX +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyBox = collision_benchmark::makeBoxVtk(1.0,2.0,3.0);
  collision_benchmark::getTriangleSoup(polyBox, points, faces);
  printTriangleSoup(points, faces);

  std::cout << " +++++++++++++ CONE +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyCone = collision_benchmark::makeConeVtk(2.0, 3.0, 5, 45, true);
  collision_benchmark::getTriangleSoup(polyCone, points, faces);
  printTriangleSoup(points, faces);

  std::cout << " +++++++++++++ DISK +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyDisk = collision_benchmark::makeDiskVtk(1.0, 5.0, 1, 5);
  collision_benchmark::getTriangleSoup(polyDisk, points, faces);
  printTriangleSoup(points, faces);

  ////////////////// Parametric objects /////////////////////////////////
  ///////////////////////////////////////////////////////////////////////
  // see also http://www.vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/ParametricObjects
  ///////////////////////////////////////////////////////////////////////

  std::cout << " +++++++++++++ ELLIPSOID +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyEllipsoid = collision_benchmark::makeEllipsoidVtk(1.0, 2.0, 3.0,
                                                             10, 10);
  collision_benchmark::getTriangleSoup(polyEllipsoid, points, faces);
  printTriangleSoup(points, faces);

  std::cout << " +++++++++++++ TORUS +++++++++++++++ " << std::endl;
  vtkSmartPointer<vtkPolyData> polyTorus = collision_benchmark::makeTorusVtk(1, 0.1, 10, 10);
  collision_benchmark::getTriangleSoup(polyTorus, points, faces);
  printTriangleSoup(points, faces);
}
*/
