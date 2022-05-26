// Copyright (c) 2018 Marek Dvoroznak
// Licensed under the MIT License.

#include "def3d.h"
#include "mesh3d.h"
#include <unordered_set>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

void Def3D::CP::setControlPointPosition(Eigen::Vector3d t){
  
  this->pos;
  /*
  //set Child's position
  for(int i=0; i<this->childNum; i++){
      if(pChild[i] != nullptr){
          pChild[i]->setControlPointPosition(t);
      }
  }
  */
  if (ik.init() != IK_OK)
    std::cout <<("Failed to initialize IK");
  struct ik_solver_t* solver =ik.solver.create(IK_TWO_BONE);

  solver->max_iterations = 20;
  solver->tolerance = 0.01;

  //Create 3-bone
  struct ik_node_t* root = solver->node->create(0);
  struct ik_node_t* child1 = solver->node->create_child(root,1);
  struct ik_node_t* child2 = solver->node->create_child(child1,2);
  //struct ik_node_t* child3 = solver->node->create_child(child2, 3); 
  std::cout << pParent->pParent->pos[0] << std::endl;
  //child2->position = ik.vec3.vec3(this->pos[0],this->pos[1],this->pos[2]);
  child2->position = ik.vec3.vec3(
  0,
  1,
  1);
  
  if(pParent != nullptr){
/*
    child1->position = ik.vec3.vec3(
      pParent->pos[0],
      pParent->pos[1],
      pParent->pos[2]); */
          child1->position = ik.vec3.vec3(
      0,
      1,
      -1);
      
      if(pParent->pParent != nullptr){
        /* root->position = ik.vec3.vec3(
      pParent->pParent->pos[0],
      pParent->pParent->pos[1],
      pParent->pParent->pos[2]
      );*/
      /*
      root->position = ik.vec3.vec3(
      0,
      0,
      0
      );
      */

      //make effecter at the end(child3)
      struct ik_effector_t* eff = solver->effector->create();
      solver->effector->attach(eff, child2);

      //set target position
      //eff->target_position =  ik.vec3.vec3(t[0]/100 + this->pos[0], t[1]/100+this->pos[1], t[2]/100+this->pos[2]);
      eff->target_position =  ik.vec3.vec3(t[0]/100 , t[1]/100, t[2]/100);
      
      solver->flags |= IK_ENABLE_TARGET_ROTATIONS;
      //solver->flags |= IK_ENABLE_JOINT_ROTATIONS;
      ik.solver.set_tree(solver,root);
      //ik.solver.rebuild_data(solver);
      ik.solver.rebuild(solver);
      if(ik.solver.solve(solver) == true){
        (float*)root->user_data;
        pParent->pParent->pos += Eigen::Vector3d(root->position.x,root->position.y,root->position.z);
        pParent->pos += Eigen::Vector3d(child1->position.x,child1->position.y,child1->position.z);
        this->pos += Eigen::Vector3d(child2->position.x,child2->position.y,child2->position.z);
      }
      }
      
    }

}

static void apply_nodes_to_scene(struct ik_node_t* ikNode){
  Def3D::CP* node = (Def3D::CP*)ikNode->user_data;
  //node->pos = ikNode->position;
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


int Def3D::CP::getLength(){
    return this->length;
};

std::shared_ptr<Def3D::CP> Def3D::CP::getParent(){
    return this->pParent;
};

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

            it.second->length = distance;
            //update lengthm
            lengthm = distance;
        }

  }
  if(nextId > 0){
      cps[nextId]->pParent->pChild[cps[nextId]->pParent->childNum] = cps[nextId];
      cps[nextId]->pParent->childNum++;
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
