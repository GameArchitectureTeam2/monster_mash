// Copyright (c) 2018 Marek Dvoroznak
// Licensed under the MIT License.

#include "def3d.h"
#include "mesh3d.h"
#include <unordered_set>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;


void Def3D::CP::setControlPointPosition(Eigen::Vector3d t){
  //forwardKinematics(t);
  jacobianInverse(t);
  
  int i = 0;
      std::cout << i << std::endl;
    std::cout << " x:" << this->pos[0] << " y:" << this->pos[1] << " z:" << this->pos[2] << std::endl;
    std::cout << " lx:" << this->localPos[0] << " ly:" << this->localPos[1] << " lz:" << this->localPos[2] << std::endl;
    std::cout << " rx:" << this->Rotation[0] << " ry:" << this->Rotation[1] << " rz:" << this->Rotation[2] << std::endl;
    std::cout << this->length << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    i++;
  
  std::shared_ptr<Def3D::CP> tmp = this->pParent;
  
  while(tmp != nullptr){
    std::cout << i << std::endl;
    std::cout << " x:" << tmp->pos[0] << " y:" << tmp->pos[1] << " z:" << tmp->pos[2] << std::endl;
    std::cout << " lx:" << tmp->localPos[0] << " ly:" << tmp->localPos[1] << " lz:" << tmp->localPos[2] << std::endl;
    std::cout << " rx:" << tmp->Rotation[0] << " ry:" << tmp->Rotation[1] << " rz:" << tmp->Rotation[2] << std::endl;
    std::cout << tmp->length << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    i++;
    tmp = tmp->pParent;
  }
  
}

void Def3D::CP::jacobianInverse(Eigen::Vector3d delta){
  //set target point
  Eigen::Vector3d targetPosition = this->pos + delta;
  if(pParent != nullptr && pParent->pParent != nullptr){
    Eigen::Vector3d rootPosition = pParent->pParent->pos;
    Eigen::Vector3d rootChildNodePosition = pParent->pos;
    Eigen::Vector3d lastNodePosition = this->pos;
    float EPS = 0.000001f;
    float steps = 1;
    while(std::sqrt(pow(targetPosition[0] - this->pos[0],2)
    + pow(targetPosition[1] - this->pos[1],2)
    + pow(targetPosition[2] - this->pos[2],2)) >= EPS){

      Eigen::MatrixXd dTheta = GetDeltaOrigentation(targetPosition);
      //std::cout << dTheta.row(0)<< std::endl;
      forwardKinematics(dTheta.row(0));
      this->pParent->forwardKinematics(dTheta.row(1));
    }
  }
}
Eigen::Vector3d  Def3D::CP::GetDeltaOrigentation(Eigen::Vector3d target){
  Eigen::MatrixXd Jt = GetJacobianTranspose(target);
  Eigen::Vector3d V = target - this->pos;
  //std::cout << Jt;
  Eigen::MatrixXd dTheta = Jt * V;
  return dTheta;
}
Eigen::MatrixXd Def3D::CP::GetJacobianTranspose(Eigen::Vector3d target){
  MatrixXd J(3,6);
  //std::cout << this->pos - pParent->pos;
  Eigen::Vector3d v1 = (Eigen::Vector3d(1,0,0).cross(this->pos - pParent->pos));
  Eigen::Vector3d v2 = (Eigen::Vector3d(0,1,0).cross(this->pos - pParent->pos));
  Eigen::Vector3d v3 = (Eigen::Vector3d(0,0,1).cross(this->pos - pParent->pos));
  Eigen::Vector3d v4 = (Eigen::Vector3d(1,0,0).cross(this->pos - pParent->pParent->pos));
  Eigen::Vector3d v5 = (Eigen::Vector3d(0,1,0).cross(this->pos - pParent->pParent->pos));
  Eigen::Vector3d v6 = (Eigen::Vector3d(0,0,1).cross(this->pos - pParent->pParent->pos));
  //std::cout << v1 ;
  cout.precision(10);
  //std::cout << Eigen::Vector3d(pParent->Rotation.normalized()[0],0,0).cross(this->pos - pParent->pos) << std::endl;
  //std::cout << Eigen::Vector3d(pParent->Rotation.normalized()[0],0,0) << std::endl;
  //std::cout << (this->pos - pParent->pos) << std::endl;
  J << v1[0] ,v2[0] ,v3[0] ,v4[0] ,v5[0] ,v6[0] 
  ,v1[1] ,v2[1] ,v3[1],v4[1] ,v5[1] ,v6[1] 
  ,v1[2] ,v2[2] ,v3[2],v4[2] ,v5[2] ,v6[2] ;
  //std::cout << v1[0];
  return J.transpose();
}

void Def3D::CP::forwardKinematics(Eigen::Vector3d deltaRotation){

  
  if(length != -1){
    double theta = atan2(deltaRotation[1],deltaRotation[0]);
    double anglePi = acos(deltaRotation[2]/this->length);
    Eigen::Vector3d tmp = this->localPos;
    this->localPos[0] = length * cos(theta) * sin(anglePi);
    this->localPos[1] = length * sin(theta) * sin(anglePi);
    this->localPos[2] = length * cos(anglePi);
    Eigen::Vector3d delta = this->localPos-tmp;
    this->pos[0] = this->pParent->pos[0] + this->localPos[0];
    this->pos[1] = this->pParent->pos[1] + this->localPos[1];
    this->pos[2] = this->pParent->pos[2] + this->localPos[2];
  //this->pos = this->pos * M;
    pParent->Rotation += deltaRotation;
    double tmp0 = pParent->Rotation[0] - static_cast<int>(pParent->Rotation[0]);
    pParent->Rotation[0] = static_cast<int>(pParent->Rotation[0]) % 360;
    pParent->Rotation[0] += tmp0;

    double tmp1 = pParent->Rotation[1] - static_cast<int>(pParent->Rotation[01]);
    pParent->Rotation[1] = static_cast<int>(pParent->Rotation[1]) % 360;
    pParent->Rotation[1] += tmp1;
    double tmp2 = pParent->Rotation[2] - static_cast<int>(pParent->Rotation[2]);
    pParent->Rotation[2] = static_cast<int>(pParent->Rotation[2]) % 360;
    pParent->Rotation[2] += tmp2;
    //std::cout << pParent->Rotation[0];
    for(auto &it :pChild){
      if(it != nullptr){
        it->pos += delta;
      }
    }
  }
}

void Def3D::CP::setControlPointRotation(Eigen::Vector3d t){
    this->Rotation = this->Rotation + t;
    //set Child's Rotation
    for(int i=0; i<this->childNum; i++){
        if(pChild[i] != nullptr){
            pChild[i]->setControlPointRotation(t);
        }
    }
}

//GET ONE CP POSITION
Eigen::Vector3d Def3D::CP::getControlPointPosition(){
    return pos;
}


std::shared_ptr<Def3D::CP>* Def3D::CP::getChild(){
    return this->pChild;
};

Def3D::Def3D()
{

}



Def3D::~Def3D()
{
}

void Def3D::updatePtsAccordingToCPs(Mesh3D &inMesh)
{
  for(const auto &it : cps) {
    const CP &cp = *it.second;
    inMesh.VCurr.row(cp.ptId) = cp.pos;
//      p.weight = cp.weight;
//      p.fixed = cp.fixed;
  }
}



const Def3D::CP& Def3D::getCP(int index) const
{
  const auto &it = cps.find(index);
  if (it == cps.end()) {
    throw(out_of_range("Def3D::getCP: index (" + to_string(index) + ") not found"));
  }
  return *it->second;
}

Def3D::CP& Def3D::getCP(int index)
{
  return const_cast<CP&>(static_cast<const Def3D&>(*this).getCP(index));
}

const std::map<int,std::shared_ptr<Def3D::CP>>& Def3D::getCPs() const
{
  return cps;
}

int Def3D::addCP(CP &cp)
{
  //first cp?
  cps[nextId] = make_shared<CP>(cp);

  //find cps nearest control points
  double lengthm = 99999999;
  for(auto &it :cps){
      //out sqrt

        double distance =
                sqrt(pow(cp.pos[0]-it.second->pos[0],2) +
                pow(cp.pos[1]-it.second->pos[1],2) +
                pow(cp.pos[2]-it.second->pos[2],2));
        if((distance <= lengthm) && (distance != 0)){
            //apply this cp's parent
            cps[nextId]->pParent = it.second;
            //apply child and length(frame length)

            cps[nextId]->length = distance;
            //update lengthm
            lengthm = distance;
            cps[nextId]->localPos = cp.pos - it.second->pos;
        }

  }
  if(nextId > 0){
      cps[nextId]->pParent->pChild[cps[nextId]->pParent->childNum] = cps[nextId];
      cps[nextId]->pParent->childNum++;
  }
  std::cout 
  << cp.localPos[0] << " "
  << cp.localPos[1] << " "
  << cp.localPos[2] << " "
  << std::endl;
  nextId++;
  cpChangedNum++;
  return nextId-1;
}

std::vector<int> Def3D::getControlPointsInsideRect(double x1, double y1, double x2, double y2, const Eigen::Matrix4d &M)
{
  vector<int> ids;
  for(const auto &it : cps) {
    const CP &cp = *it.second;
    const int cpId = it.first;
    const Vector3d p = (M * cp.pos.homogeneous()).hnormalized();
    if (p(0) >= x1 && p(1) >= y1 && p(0) <= x2 && p(1) <= y2) ids.push_back(cpId);
  }
  return ids;
}

int Def3D::getControlPoint(double x, double y, double radius, double depth, bool considerDepth, bool highestDepth, const Eigen::Matrix4d &M)
{
  int ind = -1;
  double extDepth = highestDepth ? -numeric_limits<double>::infinity() : numeric_limits<double>::infinity();
  double minDist = numeric_limits<double>::infinity();
  Vector3d pos(x,y,depth);
  for(const auto &it : cps) {
    CP &cp = *it.second;
    const int cpId = it.first;
    const Vector3d p = (M * cp.pos.homogeneous()).hnormalized();
    double d;
    if (considerDepth) d = (pos-p).norm();
    else d = (pos.head(2)-p.head(2)).norm();
    double depth = p(2);
    if (d <= radius && d < minDist && (highestDepth ? depth>extDepth : depth<extDepth)) {
      minDist = d;
      extDepth = depth;
      ind = cpId;
    }
  }
  return ind;
}

bool Def3D::addControlPointOnFace(const MatrixXd &V, const MatrixXi &F, double x, double y, double planarRadius, bool highest, bool reverseDirection)
{
  int index;
  return addControlPointOnFace(V, F, x, y, planarRadius, highest, reverseDirection, index);
}

bool Def3D::addControlPointOnFace(const MatrixXd &V, const MatrixXi &F, double x, double y, double planarRadius, bool highest, bool reverseDirection, int &index, const Eigen::Matrix4d &M)
{
  int ind = getControlPoint(x, y, planarRadius, 0, false, highest, M);
  if (ind != -1) {
    index = ind;
    return false;
  }

  const MatrixXd VProj = (V.rowwise().homogeneous() * M.transpose()).rowwise().hnormalized();
  int faceId = Mesh3D::getMeshFace(VProj, F, x, y, highest, reverseDirection);
  if (faceId == -1) {
    index = -1;
    return false;
  }

  const VectorXi &facePts = F.row(faceId);
  // choose the nearest point of the face
  double min = numeric_limits<double>::infinity();
  Vector2d pos(x,y);
  int ptId = -1;
  fora(i, 0, facePts.size()) {
    const Vector3d &p = VProj.row(facePts(i));
    double d = (p.head(2)-pos).norm();
    if (d < min) {
      min = d;
      ptId = facePts[i];
    }
  }
  ind = ptId;

  // check if such control point is associated to a mesh point
  for(const auto &it : cps) {
    const int cpId = it.first;
    const CP &cp = *it.second;
    if (cp.ptId == ind) {
      index = cpId;
      return false;
    }
  }
  double z = VProj(ind,2);
  const Vector3d unproj = (M.inverse() * Vector3d(x,y,z).homogeneous()).hnormalized();
  CP cp(unproj, cpDefaultFixed, cpDefaultWeight);
  cp.prevPos = cp.pos;
  cp.ptId = ind;
  index = addCP(cp);
  return true;
}

bool Def3D::addControlPoint(const MatrixXd &V, double x, double y, double planarRadius, bool highest, int &index, const MatrixXd &M)
{
  int ind = getControlPoint(x, y, planarRadius, 0, false, highest);
  if (ind != -1) {
    index = ind;
    return false;
  }

  const MatrixXd VProj = (V.rowwise().homogeneous() * M.transpose()).rowwise().hnormalized();
  ind = Mesh3D::getMeshPoint(VProj, x, y, planarRadius, highest);
  if (ind == -1) {
    index = -1;
    return false;
  }

  double z = VProj(ind,2);
  const Vector3d unproj = (M.inverse() * Vector3d(x,y,z).homogeneous()).hnormalized();
  CP cp(unproj, cpDefaultFixed, cpDefaultWeight);
  cp.ptId = ind;
  index = addCP(cp);
  return true;
}

// Adds a control point to a mesh point that is the nearest to a point with [x,y,depth] coordinate
// within a radius only if there is not a control point associated to the mesh point already.
// Returns true if new control point was added and also it's index.
// Returns false if there is already a control point and also index of the control point.
// Returns false if there is not a mesh point within the radius and also index -1 in this case.
bool Def3D::addControlPoint(const MatrixXd &V, double x, double y, double radius, double depth, int &index, bool considerDepth)
{
  int ind = getControlPoint(x, y, radius, depth, considerDepth, false);
  if (ind != -1) {
    index = ind;
    return false;
  }
  ind = Mesh3D::getMeshPoint(V, x, y, radius, depth, considerDepth);
  if (ind == -1) {
    index = -1;
    return false;
  }
  CP cp(Vector3d(x,y,depth), cpDefaultFixed, cpDefaultWeight);
  cp.ptId = ind;
  index = addCP(cp);
  return true;
}

// Adds a control point to a mesh point that is the nearest to a point with [x,y,depth] coordinate
// within a radius and that is at the same time also not associated with some existing control point.
// It search the whole space around specified point and tries to find a "free" mesh point that is
// not associated with a control point.
// Returns true if new control point was added and also it's index.
// Returns false if there is already a control point and also index of the control point.
// Returns false if there is not a mesh point within the radius and also index -1 in this case.
bool Def3D::addControlPointExhaustive(const Eigen::MatrixXd &V, double x, double y, double radius, double depth, bool considerDepth, int &index)
{
  const int nPts = V.rows();
  const Vector3d pos(x,y,depth);

  // find the nearest mesh point that is not associated with a control point
  int bestInd = -1;
  int occupiedInd = -1;
  unordered_set<int> occupiedInds;
  for(const auto &it : cps) {
    int ptInd = it.second->ptId;
    occupiedInds.insert(ptInd);
  }
  double min = numeric_limits<double>::infinity();
  fora(ind, 0, nPts) {
    const Vector3d &p = V.row(ind);
    double d;
    const Vector3d dvec = p-pos;
    if (considerDepth) d = dvec.norm();
    else d = dvec.head(2).norm();
    if (d <= radius && d < min) {
      // test if the mesh point is associated with a control point
      const auto &it = occupiedInds.find(ind);
      if (it == occupiedInds.end()) {
        // this is the nearest point that is not associated
        bestInd = ind;
        min = d;
      } else {
        occupiedInd = *it;
      }
    }
  }

  if (bestInd != -1) {
    CP cp(Vector3d(x,y,depth), cpDefaultFixed, cpDefaultWeight);
    cp.ptId = bestInd;
    index = addCP(cp);
    return true;
  }

  index = occupiedInd;
  return false;
}

bool Def3D::removeControlPoint(int index)
{
  const auto &it = cps.find(index);
  if (it == cps.end()) return false;
  cps.erase(it);
  cpChangedNum++;
  return true;
}

bool Def3D::removeControlPoint(double x, double y, double radius, double depth, bool considerDepth)
{
  int ind = getControlPoint(x, y, radius, depth, considerDepth, false);
  return removeControlPoint(ind);
}

bool Def3D::removeLastControlPoint()
{
  return removeControlPoint(nextId-1);
}

void Def3D::removeControlPoints()
{
  cps.clear();
  cpChangedNum++;
}

long Def3D::getCp2ptChangedNum() const
{
  return cpChangedNum;
}
