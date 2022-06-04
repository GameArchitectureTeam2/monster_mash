// Copyright 2020-2021 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "def3dsdl.h"
#include <glm/ext/matrix_transform.hpp>
using namespace Eigen;

void drawControlPoint(const Def3D::CP &cp, MyPainter &painter, int size,
                      const Eigen::Matrix4d &M) {
  const Vector3d &p = (M * cp.pos.homogeneous()).hnormalized();
  painter.setColor(0, 0, 0, 255);
  painter.filledEllipse(p(0), p(1), size, size);
  painter.setColor(255, 0, 0, 255);
  painter.filledEllipse(p(0), p(1), size - 2, size - 2);
}

void drawControlPoint(const Eigen::VectorXd &q, MyPainter &painter, int size,
                      const Eigen::Matrix4d &M, int thickness,
                      const Cu &colorBg, const Cu &colorFg) {
  const Vector3d &p = (M * q.homogeneous()).hnormalized();
  painter.setColor(colorBg);
  painter.filledEllipse(p(0), p(1), size, size);
  painter.setColor(colorFg);
  painter.filledEllipse(p(0), p(1), size - thickness, size - thickness);
}

void drawControlPoints(const Def3D &def, MyPainter &painter, int size,
                       const Eigen::Matrix4d &M, int thickness,
                       const Cu &colorBg, const Cu &colorFg) {
  auto &cps = def.getCPs();
  Eigen::Vector3d tmp[50];
  Eigen::Vector3d tmpa[50];
  Eigen::Vector3d tmpRotation[50];
  int i = 0;
  for (const auto &it : cps) {
    drawControlPoint(it.second->pos, painter, size, M, thickness, colorBg,
                     colorFg);
    tmp[i] = it.second->pos;
    tmpa[i] = (M * (it.second->pos).homogeneous()).hnormalized();
    tmpRotation[i] = it.second->Rotation;
    i++;
  }
  i--;
  for(;i>=0;){

      glm::mat4 m(1.0f);
      m = glm::rotate(m,float(tmpRotation[i][2]/180 *M_PI),glm::vec3(0,0,1)); 
      m = glm::rotate(m,float(tmpRotation[i][1]/180 *M_PI),glm::vec3(0,1,0));
      m = glm::rotate(m,float(tmpRotation[i][0]/180 *M_PI),glm::vec3(1,0,0));
      Eigen::Matrix4d tmpM;
      tmpM <<
      m[0][0],m[0][1],m[0][2],m[0][3],
      m[1][0],m[1][1],m[1][2],m[1][3],
      m[2][0],m[2][1],m[2][2],m[2][3],
      m[3][0],m[3][1],m[3][2],m[3][3];
      Eigen::Vector3d a(50,0,0);
      Eigen::Vector3d b(0,50,0);
      Eigen::Vector3d c(0,0,50);
      
      Eigen::Vector3d pp = (M * ((Eigen::Vector3d(0,0,0))).homogeneous()).hnormalized();
      Eigen::Vector3d ppp = (M * ((Eigen::Vector3d(0,0,50))).homogeneous()).hnormalized();
      Eigen::Vector3d pppx = (M * ((Eigen::Vector3d(50,0,0))).homogeneous()).hnormalized();
      Eigen::Vector3d pppy = (M * ((Eigen::Vector3d(0,50,0))).homogeneous()).hnormalized();
      Eigen::Vector3d px = (M * (tmp[i]+(tmpM * (a).homogeneous()).hnormalized()).homogeneous()).hnormalized();
      Eigen::Vector3d py = (M * (tmp[i]+(tmpM * (b).homogeneous()).hnormalized()).homogeneous()).hnormalized();
      Eigen::Vector3d pz = (M * (tmp[i]+(tmpM * (c).homogeneous()).hnormalized()).homogeneous()).hnormalized();
      //draw Arraw
      painter.setColor(255,0,0,255);
      painter.drawLine(tmpa[i][0],tmpa[i][1],px[0],px[1],3);
      painter.drawLine(pp[0],pp[1],pppx[0],pppx[1],3);
      painter.setColor(0,255,0,255);
      painter.drawLine(tmpa[i][0],tmpa[i][1],py[0],py[1],3);
      painter.drawLine(pp[0],pp[1],pppy[0],pppy[1],3);
      painter.setColor(0,0,255,255);
      painter.drawLine(tmpa[i][0],tmpa[i][1],pz[0],pz[1],3);
      painter.drawLine(pp[0],pp[1],ppp[0],ppp[1],3);
      painter.setColor(0,0,0,50);
      
      i--;
  }

}
void drawCustomLine(const Def3D &Def, MyPainter &painter,const Eigen::Matrix4d &M){

  auto &cps = Def.getCPs();
  Eigen::VectorXd q;
  Eigen::VectorXd qParent;

  Eigen::Vector3d p;
  Eigen::Vector3d ppParent;

  Eigen::Vector3d tmp[10];
  Eigen::Vector3d tmpParent[10];
  int i = 0;
  for(const auto &it : cps){
    q = it.second->pos;
    p = (M * q.homogeneous()).hnormalized();
    if(it.second->pParent != nullptr){
        qParent = it.second->pParent->pos;
        ppParent = (M * qParent.homogeneous()).hnormalized();
        tmp[i] = p;
        tmpParent[i] = ppParent;
    }
    i++;
  }

  for(;i>0;){
    i--;
    painter.drawSkeletonLine(tmp[i][0],tmp[i][1],tmpParent[i][0],tmpParent[i][1]);

  }
}
