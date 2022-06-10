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
#include <glm/ext/matrix_transform.hpp>

void Def3D::CP::FARBIK(Eigen::Vector3d delta){
 
  if(this->pParent != nullptr && this->pParent->pParent != nullptr){
     //save current end effector's current position
  Eigen::Vector3d tmpPosition = this->pos;
  //End effector to Target Position
  this->pos = this->pos + delta;
  //Load bone length;
  double distance = this->length;
  //make new joint using end effector and end effector's parent node(joint)
  Eigen::Vector3d tmpDistanceVector;
  tmpDistanceVector[0] =  this->pParent->pos[0] - this->pos[0];
  tmpDistanceVector[1] =  this->pParent->pos[1] - this->pos[1];
  tmpDistanceVector[2] =  this->pParent->pos[2] - this->pos[2];
  //O -------- X ----------- D
  double longDistance = std::sqrt(pow(tmpDistanceVector[0],2) + pow(tmpDistanceVector[1],2) + pow(tmpDistanceVector[2],2));
  tmpDistanceVector[0] = tmpDistanceVector[0] * (distance / longDistance);
  tmpDistanceVector[1] = tmpDistanceVector[1] * (distance / longDistance); 
  tmpDistanceVector[2] = tmpDistanceVector[2] * (distance / longDistance); 
  this->pParent->pos = this->pos + tmpDistanceVector;
  std::cout << distance << " " << longDistance<< std::endl;
  distance = this->pParent->length;
  tmpDistanceVector = this->pParent->pParent->pos    -   this->pParent->pos;
  //O -------- X ----------- D
  longDistance = std::sqrt(pow(tmpDistanceVector[0],2) + pow(tmpDistanceVector[1],2) + pow(tmpDistanceVector[2],2));
  tmpDistanceVector[0] = tmpDistanceVector[0] * (distance / longDistance);
  tmpDistanceVector[1] = tmpDistanceVector[1] * (distance / longDistance); 
  tmpDistanceVector[2] = tmpDistanceVector[2] * (distance / longDistance); 
  //Save root position for second 
  Eigen::Vector3d tmpRootPos = this->pParent->pParent->pos;

  this->pParent->pParent->pos = this->pParent->pos + tmpDistanceVector;


  this->pParent->pParent->pos = tmpRootPos;
  //Only first child;
  distance = this->pParent->length;
  tmpDistanceVector = this->pParent->pos - this->pParent->pParent->pos;
  longDistance = std::sqrt(pow(tmpDistanceVector[0],2) + pow(tmpDistanceVector[1],2) + pow(tmpDistanceVector[2],2));
  tmpDistanceVector[0] = tmpDistanceVector[0] * (distance / longDistance);
  tmpDistanceVector[1] = tmpDistanceVector[1] * (distance / longDistance); 
  tmpDistanceVector[2] = tmpDistanceVector[2] * (distance / longDistance); 

  this->pParent->pos = this->pParent->pParent->pos + tmpDistanceVector;

  distance = this->length;
  tmpDistanceVector = this->pos - this->pParent->pos;
  longDistance = std::sqrt(pow(tmpDistanceVector[0],2) + pow(tmpDistanceVector[1],2) + pow(tmpDistanceVector[2],2));
  tmpDistanceVector[0] = tmpDistanceVector[0] * (distance / longDistance);
  tmpDistanceVector[1] = tmpDistanceVector[1] * (distance / longDistance); 
  tmpDistanceVector[2] = tmpDistanceVector[2] * (distance / longDistance); 

  this->pos = this->pParent->pos + tmpDistanceVector;


  }


}

void Def3D::CP::setControlPointPosition(Eigen::Vector3d t){
  //Forward Kinematics by delta t vector or IK!
  //forwardKinematics(t);
  //jacobianInverse(t);
  FARBIK(t);
  
  
}

void Def3D::CP::jacobianInverse(Eigen::Vector3d delta){
  //set target point
  Eigen::Vector3d targetPosition = delta;

  //2 Bone IK...
  if(pParent != nullptr ){
    //Error squares
    float EPS = 0.1f;
    //Learning rate
    float steps = 0.00001;
    //Maximum iterate
    int iteridx = 0;

    //while  we not close to target to enough
    while(std::sqrt(pow(targetPosition[0] - this->pos[0],2)
    + pow(targetPosition[1] - this->pos[1],2)
    + pow(targetPosition[2] - this->pos[2],2)) >= EPS){
      
      //Take Delta
      // (root)------(child1)--------(child2)
      //We call child2 node and child1 node's FK function
      //by Jacobian * V(target - this.pos) Matrix

      Eigen::MatrixXd dTheta = GetDeltaOrigentation(targetPosition);
      //dTheta Matrix's row have Node's delta Rotation
      // (0.001, 0.1223, 0.3445)- for Child2 Node Rotation delta
      // (0.1122, 0.3145, 1.2314) - for Child1 Node Rotation delta
        //std::cout << "---------------" << std::endl;
        //std::cout << std::sqrt(pow(targetPosition[0] - this->pos[0],2)
    //+ pow(targetPosition[1] - this->pos[1],2)
    //+ pow(targetPosition[2] - this->pos[2],2))<< std::endl;
      //std::cout << "---------------" << std::endl;
      forwardKinematics(dTheta.row(0)*steps);
      //this->pParent->forwardKinematics(dTheta.row(1)*steps);
      //if iteridx greater than 1000, break loop;
      if(iteridx >= 1000){
        break;
      }
      iteridx++;


    }
  }
}
Eigen::Vector3d  Def3D::CP::GetDeltaOrigentation(Eigen::Vector3d target){
  std::cout << target <<std::endl;
  //Take Jacabian Transpose Matrix its maybe 6x3 Matrix
  Eigen::MatrixXd Jt = GetJacobianTranspose(target);
  // target and this->pos 's direction and scalar
  Eigen::Vector3d V = target - this->pos;
  // multiply them
  Eigen::MatrixXd dTheta = Jt * V;
  //std::cout << "---------------" << std::endl;
  //std::cout << dTheta << std::endl;
  //std::cout << "---------------" << std::endl;
  return dTheta;
}
Eigen::MatrixXd Def3D::CP::GetJacobianTranspose(Eigen::Vector3d target){
  //Make Empty matrix(3x6)
  MatrixXd J(3,3);
  //Make identity matrix
  glm::mat4 rot_m = glm::mat4(1.0f);
  //Make rotation Matrix by Axis
  //Yaw - Pitch - Roll by root Node's Rotation. 
  rot_m = glm::rotate(rot_m,float(pParent->Rotation[2]),glm::vec3(0,0,1));
  rot_m = glm::rotate(rot_m,float(pParent->Rotation[1]),glm::vec3(0,1,0));
  rot_m = glm::rotate(rot_m,float(pParent->Rotation[0]),glm::vec3(1,0,0));
  
  //We make child1's Axis!!!! orderly x , y , z
  glm::vec4 tmp =  rot_m * glm::vec4(1,0,0,0);
  glm::vec4 tmp1 =  rot_m * glm::vec4(0,1,0,0);
  glm::vec4 tmp2 =  rot_m * glm::vec4(0,0,1,0);
  //So we now calculate Jacobian by (Axis) Cross (Effector Position and Node's Position)
  //Eigen::Vector3d v1 = ((Eigen::Vector3d(tmp[0],tmp[1],tmp[2]).normalized()).cross(this->pos - pParent->pos));
  //Eigen::Vector3d v2 = ((Eigen::Vector3d(tmp1[0],tmp1[1],tmp1[2]).normalized()).cross(this->pos - pParent->pos));
  //Eigen::Vector3d v3 = ((Eigen::Vector3d(tmp2[0],tmp2[1],tmp2[2]).normalized()).cross(this->pos - pParent->pos));
  Eigen::Vector3d v4 = ((Eigen::Vector3d(1,0,0).normalized()).cross(this->pos - pParent->pos));
  Eigen::Vector3d v5 = ((Eigen::Vector3d(0,1,0).normalized()).cross(this->pos - pParent->pos));
  Eigen::Vector3d v6 = ((Eigen::Vector3d(0,0,1).normalized()).cross(this->pos - pParent->pos));
  //put on it
  //J << v1[0] ,v2[0] ,v3[0] ,v4[0] ,v5[0] ,v6[0] 
  //,v1[1] ,v2[1] ,v3[1],v4[1] ,v5[1] ,v6[1] 
  //,v1[2] ,v2[2] ,v3[2],v4[2] ,v5[2] ,v6[2] ;
    J << v4[0] ,v5[0] ,v6[0] 
  ,v4[1] ,v5[1] ,v6[1], 
  v4[2] ,v5[2] ,v6[2] ;
  return J.transpose();
}

void Def3D::CP::forwardKinematics(Eigen::Vector3d deltaRotation){
    

  // Angle to Cartesian
  if(this->length != -1){
    double theta = atan2(deltaRotation[1],deltaRotation[0]);
    double anglePi = acos(deltaRotation[2]/this->length);
    Eigen::Vector3d tmp = this->localPos;
    this->localPos[0] = this->length * cos(theta) * sin(anglePi);
    this->localPos[1] = this->length * sin(theta) * sin(anglePi);
    this->localPos[2] = this->length * cos(anglePi);
    Eigen::Vector3d delta = this->localPos-tmp;
    this->pos[0] = this->pParent->pos[0] + this->localPos[0];
    this->pos[1] = this->pParent->pos[1] + this->localPos[1];
    this->pos[2] = this->pParent->pos[2] + this->localPos[2];
  
    //apply rotation
    glm::vec3 directionVector(this->localPos[0], this->localPos[1],this->localPos[2]);
            
    //Child1 Node(Axis goes Ex,Ey,Ez)
    if(pParent->pParent == nullptr){
      
      //Y-axis
      //Y-axis's Rotation Can Caculate Inner Product between X axis and (x,0,z)
      this->pParent->Rotation[1] = 
      glm::acos(
        glm::dot(
          glm::vec3(directionVector[0],0,directionVector[2]),glm::vec3(0,0,1))
          /sqrt(pow(directionVector[0],2)+pow(directionVector[2],2))) / M_PI * 180;

      //Z-Axis
      //z-axis can get by Inner product between Y axis and Vector(x,y,0)

            this->pParent->Rotation[2] = 
            glm::acos(glm::dot(glm::vec3(directionVector[0],directionVector[1],0),glm::vec3(1,0,0))/
            sqrt(pow(directionVector[0],2)+pow(directionVector[1],2)))/ M_PI * 180;

             //z-Axis
            this->pParent->Rotation[0] = 
            glm::acos(glm::dot(glm::vec3(0,directionVector[1],directionVector[2]),glm::vec3(0,1,0))/
            sqrt(pow(directionVector[2],2)+pow(directionVector[1],2)))/ M_PI * 180;
            //std::cout << "---------------------------" << std::endl;
            //std::cout << "x: " << this->pParent->Rotation[0] << std::endl;
            //std::cout << "y: " << this->pParent->Rotation[1] << std::endl;
            //std::cout << "z: " << this->pParent->Rotation[2] << std::endl;
            //std::cout << "---------------------------" << std::endl;
    }else{
        glm::mat4 rot_m = glm::mat4(1.0f);
      //Make rotation Matrix by Axis
      //Rotate (Matrix, Angle , Axis)
      rot_m = glm::rotate(rot_m,float(pParent->pParent->Rotation[2]),glm::vec3(0,0,1));
      rot_m = glm::rotate(rot_m,float(pParent->pParent->Rotation[1]),glm::vec3(0,1,0));
      rot_m = glm::rotate(rot_m,float(pParent->pParent->Rotation[0]),glm::vec3(1,0,0));
  
  
      glm::vec4 tmp =  rot_m * glm::vec4(1,0,0,0);
      glm::vec4 tmp1 =  rot_m * glm::vec4(0,1,0,0);
      glm::vec4 tmp2 =  rot_m * glm::vec4(0,0,1,0);
      this->pParent->Rotation[1] = 
            glm::acos(glm::dot(glm::vec3(directionVector[0],0,directionVector[2]),glm::vec3(tmp2[0],tmp2[1],tmp2[2]))/
            sqrt(pow(directionVector[0],2)+pow(directionVector[2],2))) / M_PI * 180;
            //y-Axis
      this->pParent->Rotation[2] = 
            glm::acos(glm::dot(glm::vec3(directionVector[0],directionVector[1],0),glm::vec3(tmp1[0],tmp1[1],tmp1[2]))/
            sqrt(pow(directionVector[0],2)+pow(directionVector[1],2)))/ M_PI * 180;
             //z-Axis
      this->pParent->Rotation[0] = 
            glm::acos(glm::dot(glm::vec3(0,directionVector[1],directionVector[2]),glm::vec3(tmp[0],tmp[1],tmp[2]))/
            sqrt(pow(directionVector[2],2)+pow(directionVector[1],2)))/ M_PI * 180;
    }
    
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

Def3D::Def3D(){

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
 
            
            
        }
  
  }
  
  if(nextId > 0){
      cps[nextId]->pParent->pChild[cps[nextId]->pParent->childNum] = cps[nextId];
      cps[nextId]->pParent->childNum++;

           cps[nextId]->localPos = cp.pos - cps[nextId]->pParent->pos;

            //apply rotation
            glm::vec3 directionVector(cp.pos[0]- cps[nextId]->pParent->pos[0],cp.pos[1]-cps[nextId]->pParent->pos[1],cp.pos[0]-cps[nextId]->pParent->pos[2]);
                  cps[nextId]->pParent->Rotation[1] = 
      glm::acos(
        glm::dot(
          glm::vec3(directionVector[0],0,directionVector[2]),glm::vec3(0,0,1))
          /sqrt(pow(directionVector[0],2)+pow(directionVector[2],2))) / M_PI * 180;

      //Z-Axis
      //z-axis can get by Inner product between Y axis and Vector(x,y,0)
            cps[nextId]->pParent->Rotation[2] = 
            glm::acos(glm::dot(glm::vec3(directionVector[0],directionVector[1],0),glm::vec3(0,1,0))/
            sqrt(pow(directionVector[0],2)+pow(directionVector[1],2)))/ M_PI * 180;
             //z-Axis
            cps[nextId]->pParent->Rotation[0] = 
            glm::acos(glm::dot(glm::vec3(0,directionVector[1],directionVector[2]),glm::vec3(1,0,0))/
            sqrt(pow(directionVector[2],2)+pow(directionVector[1],2)))/ M_PI * 180;

  
  }

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
