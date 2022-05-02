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
  Eigen::VectorXd tmp[10];
  Eigen::VectorXd tmpa[10];

  int i = 0;
  for (const auto &it : cps) {
    drawControlPoint(it.second->pos, painter, size, M, thickness, colorBg,
                     colorFg);
    tmp[i] = it.second->pos;
    tmpa[i] = (M * (it.second->pos).homogeneous()).hnormalized();
    i++;
  }
  i--;
  for(;i>=0;){
      Eigen::Vector3d a(50,0,0);
      Eigen::Vector3d b(0,50,0);
      Eigen::Vector3d c(0,0,50);

      Eigen::Vector3d px = (M * (tmp[i]+a).homogeneous()).hnormalized();
      Eigen::Vector3d py = (M * (tmp[i]+b).homogeneous()).hnormalized();
      Eigen::Vector3d pz = (M * (tmp[i]+c).homogeneous()).hnormalized();
      //draw Arraw
      painter.setColor(255,0,0,255);
      painter.drawLine(tmpa[i][0],tmpa[i][1],px[0],px[1],7);
      painter.setColor(0,255,0,255);
      painter.drawLine(tmpa[i][0],tmpa[i][1],py[0],py[1],7);
      painter.setColor(0,0,255,255);
      painter.drawLine(tmpa[i][0],tmpa[i][1],pz[0],pz[1],7);
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
