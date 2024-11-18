//
// Created by Karthik Iyer on 04/11/24.
//

#include "RigidBody.h"

#include <iostream>
#include <fstream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "tiny_obj_loader.h"

#include <random>

#include "MatrixStack.h"
#include "RigidBody.h"
#include "GLSL.h"
#include "Program.h"
#include "Polygon.h"
#include "SimParams.h"
#include "Vertex.h"
#include "IForceField.h"
#include "Shape.h"

using namespace std;
using namespace glm;

#define kEpsilon 1e-5

RigidBody::RigidBody(double m, Eigen::Vector3d pos, Eigen::Vector3d v, Eigen::Vector3d angV) :
        mass(m),
        x0(pos),
        x(pos),
        v0(v),
        v(v),
        angV0(angV),
        angV(angV),
        prog(NULL),
        posBufID(0),
        norBufID(0),
        texBufID(0),
        isObstacle(true)
{
    R0 = Eigen::Matrix3d::Identity();
    R = R0;
    massInv = 1.0 / mass;
}

RigidBody::RigidBody(double m, Eigen::Vector3d pos, Eigen::Vector3d v, Eigen::Vector3d angV, Eigen::Matrix3d rotMat) :
        mass(m),
        x0(pos),
        x(pos),
        v0(v),
        v(v),
        angV0(angV),
        angV(angV),
        prog(NULL),
        posBufID(0),
        norBufID(0),
        texBufID(0),
        isObstacle(true) {
    R0 = Eigen::Matrix3d::Identity();
    R0 *= rotMat;
    R = R0;
    massInv = 1.0 / mass;
}

RigidBody::~RigidBody()
{
}

void RigidBody::loadObj(const string &filename, vector<float> &pos, vector<float> &nor, vector<float> &tex, bool loadNor, bool loadTex)
{

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str());
    if(!warn.empty()) {
        //std::cout << warn << std::endl;
    }
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }
    if(!ret) {
        return;
    }
    // Loop over vertices
    for (size_t s = 0; s < attrib.vertices.size()/3; s++) {
        float px = attrib.vertices[3*s + 0];
        float py = attrib.vertices[3*s + 1];
        float pz = attrib.vertices[3*s + 2];
        vertices.push_back(Eigen::Vector3d(px, py, pz));
        vertices0.push_back(Eigen::Vector3d(px, py, pz));
        verticesTemp.push_back(Eigen::Vector3d(px, py, pz));
    }
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];
            // Loop over vertices in the face.
            Polygon polygon;
            for(size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                pos.push_back(attrib.vertices[3*idx.vertex_index+0]);
                pos.push_back(attrib.vertices[3*idx.vertex_index+1]);
                pos.push_back(attrib.vertices[3*idx.vertex_index+2]);

                polygon.addPoint(idx.vertex_index);

                if(!attrib.normals.empty() && loadNor) {
                    nor.push_back(attrib.normals[3*idx.normal_index+0]);
                    nor.push_back(attrib.normals[3*idx.normal_index+1]);
                    nor.push_back(attrib.normals[3*idx.normal_index+2]);
                }
                if(!attrib.texcoords.empty() && loadTex) {
                    tex.push_back(attrib.texcoords[2*idx.texcoord_index+0]);
                    tex.push_back(attrib.texcoords[2*idx.texcoord_index+1]);
                }
            }
            polygons.push_back(polygon);
            index_offset += fv;
        }
    }

    for (Polygon p: polygons) {
        for (int i = 0; i < p.vertIndices.size() - 1; i++) {
            edges.insert(std::make_pair(p.vertIndices[i], p.vertIndices[i+1]));
        }
        edges.insert(std::make_pair(p.vertIndices[p.vertIndices.size()-1], p.vertIndices[0]));
    }

    computeMomentOfInertia();
}

void RigidBody::computeAABB() {
    xMin = DBL_MAX;
    yMin = DBL_MAX;
    zMin = DBL_MAX;
    xMax = DBL_MIN;
    yMax = DBL_MIN;
    zMax = DBL_MIN;
    for (int i = 0; i < vertices.size(); ++i) {
        Eigen::Vector3d p = vertices[i];
        xMin = std::min(xMin, p.x());
        yMin = std::min(yMin, p.y());
        zMin = std::min(zMin, p.z());
        xMax = std::max(xMax, p.x());
        yMax = std::max(yMax, p.y());
        zMax = std::max(zMax, p.z());
    }
}


// from https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection.html
bool RigidBody::intersectsTri(Polygon p, Eigen::Vector3d pt, Eigen::Vector3d ray) {
    Eigen::Vector3d p0 = vertices[p.vertIndices[0]];
    Eigen::Vector3d p1 = vertices[p.vertIndices[1]];
    Eigen::Vector3d p2 = vertices[p.vertIndices[2]];

    Eigen::Vector3d edge1 = p1 - p0;
    Eigen::Vector3d edge2 = p2 - p0;

    Eigen::Vector3d pVec = ray.cross(edge2);
    double det = edge1.dot(pVec);

    if (det < kEpsilon)
        return false;

    double invDet = 1.0 / det;

    Eigen::Vector3d tVec = pt - p0;

    double u = tVec.dot(pVec) * invDet;
    
    if (u < 0 || u > 1) {
        return false;
    }
    
    Eigen::Vector3d qVec = tVec.cross(edge1);
    double v = ray.dot(qVec) * invDet;
    
    if (v < 0 || u + v > 1) {
        return false;
    }
    
    return true;
}

void RigidBody::initObjWithRot() {
    for (int i = 0; i < vertices.size(); ++i) {
//        vertices0[i] = R * vertices0[i];
        vertices[i] = R * vertices0[i];
        verticesTemp[i] = vertices[i];
    }
}

void RigidBody::initObjWithLoc() {
    for (int i = 0; i < vertices.size(); ++i) {
        vertices[i] = vertices0[i] + x;
        verticesTemp[i] = vertices[i];
    }
}

void RigidBody::computeMomentOfInertia() {
    computeAABB();
    Eigen::Vector3d minPt(xMin, yMin, zMin);
    double spacing = 1e-2;
    int xPts = (int) std::ceil((xMax - xMin)/spacing);
    int yPts = (int) std::ceil((yMax - yMin)/spacing);
    int zPts = (int) std::ceil((zMax - zMin)/spacing);
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < xPts; ++i) {
        for (int j = 0; j < yPts; ++j) {
            for (int k = 0; k < zPts; ++k) {
                Eigen::Vector3d pt = minPt + Eigen::Vector3d(i * spacing, j * spacing, k * spacing);

                // consider point if it is inside mesh
                int intersectionCount = 0;
                for (auto polygon: polygons) {
                    if (intersectsTri(polygon, pt, Eigen::Vector3d(1, 0, 0)))
                        intersectionCount++;
                }
                if (intersectionCount != 0 && intersectionCount % 2 == 1) {
                    points.push_back(pt);
                }
            }
        }
    }

    double pointMass = (double) (mass / points.size());

    I0 << 0, 0, 0,
         0, 0, 0,
         0, 0, 0;

    for (auto pt: points) {
        double rix = pt.x();
        double riy = pt.y();
        double riz = pt.z();

        Eigen::Matrix3d Ii;

        Ii << (riy*riy + riz*riz), -rix * riy, -rix * riz,
              -riy * rix, (rix*rix + riz*riz), -riy * riz,
              -riz * rix, -riz * riy, (rix*rix + riy*riy);

        I0 += pointMass * Ii;
    }

    I0 = R * I0 * R.transpose();
    I = I0;
    I0inv = I0.inverse();
    Iinv = I0inv;

    std::cout<<"I: \n";
    std::cout<<I<<"\n";
    initObjWithRot();
    initObjWithLoc();
//    R = Eigen::Matrix3d::Identity();
//    R0 = Eigen::Matrix3d::Identity();

//    reset();
}

void RigidBody::reset() {
    x = x0;
    v = v0;
    angV = angV0;
    R = R0;
    I = I0;
    Iinv = I0inv;
    initObjWithRot();
    initObjWithLoc();
}

inline glm::mat4 RigidBody::convertToGLMMat(Eigen::Matrix3d rot) {
    glm::mat4 glmR = glm::mat4(1.0f);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glmR[i][j] = rot(i, j);
        }
    }
    return glmR;
}

double RigidBody::sgn(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

bool RigidBody::pointTriCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, std::vector<CollisionData>& collData) {
    collData.clear();
    for (int i = 0; i < verticesTemp.size(); ++i) {

        Eigen::Vector3d pt = verticesTemp[i];
        Eigen::Vector3d ptNew = vertices[i];

        for (auto shape: shapes) {
            if (!shape->getObstacle())
                continue;

            for (Polygon p: shape->getPolygons()) {
                Eigen::Vector3d P = p.points[0];
                Eigen::Vector3d Q = p.points[1];
                Eigen::Vector3d R = p.points[2];

                Eigen::Vector3d u = Q - P;
                Eigen::Vector3d v = R - P;
                Eigen::Vector3d n = u.cross(v);
                n.normalize();
                if (n.norm() == 0) continue;

                double pn = P.dot(n);
                double d0 = (n.dot(pt) - pn);
                double d1 = (n.dot(ptNew) - pn);

                if (sgn(d0) * sgn(d1) >= 0) continue;

                Eigen::Vector3d dir = ptNew - pt;
                dir.normalize();
                double t = (pn - n.dot(pt))/(n.dot(dir));
                Eigen::Vector3d xColl = pt + (t * dir);

                // check if point is inside polygon by projecting along axis with highest normal value
                Eigen::Vector3d Pproj, Qproj, Rproj, xCollProj;
                if (std::abs(n.x()) >= std::max(std::abs(n.y()), std::abs(n.z()))) {
                    Pproj = Eigen::Vector3d(P.y(), P.z(), 0.0);
                    Qproj = Eigen::Vector3d(Q.y(), Q.z(), 0.0);
                    Rproj = Eigen::Vector3d(R.y(), R.z(), 0.0);
                    xCollProj = Eigen::Vector3d(xColl.y(), xColl.z(), 0.0);
                } else if (std::abs(n.y()) >= std::max(std::abs(n.x()), std::abs(n.z()))) {
                    Pproj = Eigen::Vector3d(P.z(), P.x(), 0.0);
                    Qproj = Eigen::Vector3d(Q.z(), Q.x(), 0.0);
                    Rproj = Eigen::Vector3d(R.z(), R.x(), 0.0);
                    xCollProj = Eigen::Vector3d(xColl.z(), xColl.x(), 0.0);
                } else {
                    Pproj = Eigen::Vector3d(P.x(), P.y(), 0.0);
                    Qproj = Eigen::Vector3d(Q.x(), Q.y(), 0.0);
                    Rproj = Eigen::Vector3d(R.x(), R.y(), 0.0);
                    xCollProj = Eigen::Vector3d(xColl.x(), xColl.y(), 0.0);
                }

                Eigen::Vector3d edge1Proj = (Pproj - Qproj);
                Eigen::Vector3d edge2Proj = (Pproj - Rproj);
                Eigen::Vector3d areaProj = edge1Proj.cross(edge2Proj);
                double S = 0.5 * areaProj.norm() * (areaProj.z() / std::abs(areaProj.z()));
                Eigen::Vector3d AaVec = (Qproj - xCollProj).cross(Rproj - xCollProj);
                double Aa = 0.5 * AaVec.norm() * (AaVec.z()/std::abs(AaVec.z()));
                Eigen::Vector3d AbVec = (Rproj - xCollProj).cross(Pproj - xCollProj);
                double Ab = 0.5 * AbVec.norm() * (AbVec.z()/std::abs(AbVec.z()));
                Eigen::Vector3d AcVec = (Pproj - xCollProj).cross(Qproj - xCollProj);
                double Ac = 0.5 * AcVec.norm() * (AcVec.z()/std::abs(AcVec.z()));
                double a = Aa/S;
                double b = Ab/S;
                double c = Ac/S;

                if (a >= 0 && a <= 1.0 && b >= 0 && b <= 1 && c >= 0 && c <= 1) { // collision
                    CollisionData cData;
                    cData.xColl = xColl;
                    cData.nColl = n;
                    cData.corrVec = xColl - ptNew;
                    collData.push_back(cData);
                    std::cout<<"Collision!\n";
                    std::cout<<"xColl: "<<xColl.transpose()<<"\n";
                    std::cout<<"nColl: "<<n.transpose()<<"\n\n";
//                    return true;
                } else {
                    continue;
                }
            }
        }
    }
    return !collData.empty();
}

void RigidBody::detectCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, std::vector<Eigen::Vector3d> &c) {
//    std::cout << "Here!\n";
//    std::cout << "I:\n" << I << "\n";
//    std::cout << "Iinv:\n" << Iinv << "\n";

//    CollisionData collData;
    std::vector<CollisionData> collData;
    bool collided = pointTriCollision(h, shapes, simParams, collData);
    if (!collided)
        return;
//    return;
    // for testing
//    Eigen::Vector3d collPt(1, 1, -1);
//    Eigen::Vector3d nColl(0, 1, 0);
    double corrVecX = 0;
    double corrVecY = 0;
    double corrVecZ = 0;
    for (CollisionData& cData: collData) {
        Eigen::Vector3d collPt = cData.xColl;
        Eigen::Vector3d nColl = cData.nColl;

        // calc coll pos in c.o.m. frame
        Eigen::Vector3d rColl = collPt - x;

        Eigen::Vector3d vCollPt = v + angV.cross(rColl);
        double vCollPtNor = vCollPt.dot(nColl);

        Iinv = R * I0inv * R.transpose();

        double jDen = massInv + nColl.dot((Iinv * (rColl.cross(nColl)).cross(rColl)));
        double jn = (-(1.0 + simParams.restitutionCoefficient) * vCollPtNor) / jDen;

        // for testing
//        jn = 0.0;

        Eigen::Vector3d deltaAngV = jn * Iinv * (rColl.cross(nColl));
//        std::cout << "I: \n" << I << "\n";
//        std::cout << "Iinv: \n" << Iinv << "\n";
        std::cout << "deltaAngV: " << deltaAngV.transpose() << "\n";
//        std::cout << "vn bef: " << (v + angV.cross(rColl)).dot(nColl) << "\n";
//        angV += deltaAngV;
//        std::cout << "angV: " << angV.transpose() << "\n";
        Eigen::Vector3d deltaV = massInv * jn * nColl;
//        std::cout << "v bef: " << v.transpose() << "\n";
//        v += deltaV;
        std::cout << "deltaV: " << deltaV.transpose() << "\n";
//        std::cout << "v aft: " << v.transpose() << "\n";
//        std::cout << "vn aft: " << (v + angV.cross(rColl)).dot(nColl) << "\n";

        if (std::abs(cData.corrVec.x()) > std::abs(corrVecX))
            corrVecX = cData.corrVec.x();
        if (std::abs(cData.corrVec.y()) > std::abs(corrVecY))
            corrVecY = cData.corrVec.y();
        if (std::abs(cData.corrVec.z()) > std::abs(corrVecZ))
            corrVecZ = cData.corrVec.z();
        cData.deltaV = deltaV;
        cData.deltaAngV = deltaAngV;
//        corrVec += cData.corrVec;
        c.push_back(cData.xColl);
    }
    std::cout<<"collDataSize: "<<collData.size()<<"\n";

    std::cout<<"v bef: "<<v.transpose()<<"\n";
    std::cout<<"ang v bef: "<<angV.transpose()<<"\n";
    for (int i = 0; i < collData.size(); ++i) {
        v += collData[i].deltaV;
        angV += collData[i].deltaAngV;
//        x += ((1 + 2e-2) * collData[i].corrVec);
    }
    std::cout<<"v aft: "<<v.transpose()<<"\n";
    std::cout<<"ang v afte: "<<angV.transpose()<<"\n";

    Eigen::Vector3d corrVec(corrVecX, corrVecY, corrVecZ);
//    corrVec /= collData.size();
    x += ((1 + 1e-2) * corrVec);
    for (int i = 0; i < vertices.size(); ++i) {
        verticesTemp[i] = vertices[i];
        vertices[i] = (R * vertices0[i]) + x;
    }
}

bool RigidBody::edgeEdgeCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, std::vector<CollisionData>& collData) {
    for (std::pair<int, int> e: edges) {
        Eigen::Vector3d p0 = vertices[e.first];
        Eigen::Vector3d p1 = vertices[e.second];

        Eigen::Vector3d p0Temp = verticesTemp[e.first];
        Eigen::Vector3d p1Temp = verticesTemp[e.second];

        Eigen::Vector3d e1 = p1Temp - p0Temp;
        Eigen::Vector3d P1 = p0Temp;

        Eigen::Vector3d e1Plus = p1 - p0;
        Eigen::Vector3d P1Plus = p0;

        for (auto shape: shapes) {
            if (!shape->getObstacle()) {
                continue;
            }

            for (Polygon p: shape->getPolygons()) {
                Eigen::Vector3d P = p.points[0];
                Eigen::Vector3d Q = p.points[1];
                Eigen::Vector3d R = p.points[2];

                // PQ
                Eigen::Vector3d e2 = Q - P;
                Eigen::Vector3d n = e1.cross(e2);
                Eigen::Vector3d q = P - P1;
                n.normalize();
                double s = (q.dot(e2.normalized().cross(n))) / (e1.dot(e2.normalized().cross(n)));
                double t = (-q.dot(e1.normalized().cross(n))) / (e2.dot(e1.normalized().cross(n)));

                Eigen::Vector3d nPlus = e1Plus.cross(e2);
                nPlus.normalize();
                Eigen::Vector3d qPlus = P - P1Plus;
                double sPlus = (qPlus.dot(e2.normalized().cross(nPlus))) / (e1Plus.dot(e2.normalized().cross(nPlus)));
                double tPlus = (-qPlus.dot(e1Plus.normalized().cross(nPlus))) / (e2.dot(e1Plus.normalized().cross(nPlus)));

                if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
                    Eigen::Vector3d collPos = P1 + s * e1;
                    Eigen::Vector3d collPos2 = P + t * e2;
//                    std::cout<<"Before: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                    Eigen::Vector3d mMinus = collPos2 - collPos;
                    collPos = P1Plus + sPlus * e1Plus;
                    collPos2 = P + tPlus * e2;

//                    std::cout<<"After: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                    Eigen::Vector3d mPlus = collPos2 - collPos;
//                    return;
                    if (mMinus.dot(mPlus) < 0) {

                        CollisionData cData;
                        cData.xColl = collPos2;
                        cData.nColl = n;
                        cData.corrVec = mPlus - mMinus;
                        collData.push_back(cData);
                        std::cout<<"Edge Collision!\n";
                        std::cout<<"xColl: "<<collPos2.transpose()<<"\n";
                        std::cout<<"nColl: "<<n.transpose()<<"\n\n";

                        return !collData.empty();
                    }
                }

                // QR
                e2 = R - Q;
                n = e1.cross(e2);
                q = Q - P1;
                n.normalize();
                s = (q.dot(e2.normalized().cross(n))) / (e1.dot(e2.normalized().cross(n)));
                t = (-q.dot(e1.normalized().cross(n))) / (e2.dot(e1.normalized().cross(n)));

                nPlus = e1Plus.cross(e2);
                nPlus.normalize();
                qPlus = Q - P1Plus;
                sPlus = (qPlus.dot(e2.normalized().cross(nPlus))) / (e1Plus.dot(e2.normalized().cross(nPlus)));
                tPlus = (-qPlus.dot(e1Plus.normalized().cross(nPlus))) / (e2.dot(e1Plus.normalized().cross(nPlus)));

                if ((s >= 0 && s <= 1 && t >= 0 && t <= 1) || (sPlus >= 0 && sPlus <= 1 && tPlus >= 0 && tPlus <= 1)) {
                    Eigen::Vector3d collPos = P1 + s * e1;
                    Eigen::Vector3d collPos2 = Q + t * e2;
                    double dist = (collPos2 - collPos).norm();
//                    std::cout<<"Before: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                    Eigen::Vector3d mMinus = collPos2 - collPos;
                    collPos = P1Plus + sPlus * e1Plus;
                    collPos2 = Q + tPlus * e2;

//                    std::cout<<"After: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                    Eigen::Vector3d mPlus = collPos2 - collPos;
//                    return;
                    if (mMinus.dot(mPlus) < 0) {
                        CollisionData cData;
                        cData.xColl = collPos2;
                        cData.nColl = n;
                        cData.corrVec = mPlus - mMinus;
                        collData.push_back(cData);
                        std::cout<<"Edge Collision!\n";
                        std::cout<<"xColl: "<<collPos2.transpose()<<"\n";
                        std::cout<<"nColl: "<<n.transpose()<<"\n\n";
                        return !collData.empty();
                    }
                }

                // RP
                e2 = P - R;
                n = e1.cross(e2);
                q = R - P1;
                n.normalize();
                s = (q.dot(e2.normalized().cross(n))) / (e1.dot(e2.normalized().cross(n)));
                t = (-q.dot(e1.normalized().cross(n))) / (e2.dot(e1.normalized().cross(n)));

                nPlus = e1Plus.cross(e2);
                nPlus.normalize();
                qPlus = R - P1Plus;
                sPlus = (qPlus.dot(e2.normalized().cross(nPlus))) / (e1Plus.dot(e2.normalized().cross(nPlus)));
                tPlus = (-qPlus.dot(e1Plus.normalized().cross(nPlus))) / (e2.dot(e1Plus.normalized().cross(nPlus)));

                if ((s >= 0 && s <= 1 && t >= 0 && t <= 1) || (sPlus >= 0 && sPlus <= 1 && tPlus >= 0 && tPlus <= 1)) {
                    Eigen::Vector3d collPos = P1 + s * e1;
                    Eigen::Vector3d collPos2 = R + t * e2;
                    double dist = (collPos2 - collPos).norm();
//                    std::cout<<"Before: \n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                    Eigen::Vector3d mMinus = collPos2 - collPos;
                    collPos = P1Plus + sPlus * e1Plus;
                    collPos2 = R + tPlus * e2;

//                    std::cout<<"After: \n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                    Eigen::Vector3d mPlus = collPos2 - collPos;
//                    return;
                    if (mMinus.dot(mPlus) < 0) {
                        CollisionData cData;
                        cData.xColl = collPos2;
                        cData.nColl = n;
                        cData.corrVec = mPlus - mMinus;
                        collData.push_back(cData);
                        std::cout<<"Edge Collision!\n";
                        std::cout<<"xColl: "<<collPos2.transpose()<<"\n";
                        std::cout<<"nColl: "<<n.transpose()<<"\n\n";
                        return !collData.empty();
                    }
                }

            }
        }
    }
    return !collData.empty();
}

void RigidBody::detectEdgeCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams) {
    //    std::cout << "Here!\n";
//    std::cout << "I:\n" << I << "\n";
//    std::cout << "Iinv:\n" << Iinv << "\n";

//    CollisionData collData;
    std::vector<CollisionData> collData;
    bool collided = edgeEdgeCollision(h, shapes, simParams, collData);
    if (!collided)
        return;
//    return;
    // for testing
//    Eigen::Vector3d collPt(1, 1, -1);
//    Eigen::Vector3d nColl(0, 1, 0);
    double corrVecX = 0;
    double corrVecY = 0;
    double corrVecZ = 0;
    for (CollisionData& cData: collData) {
        Eigen::Vector3d collPt = cData.xColl;
        Eigen::Vector3d nColl = cData.nColl;

        // calc coll pos in c.o.m. frame
        Eigen::Vector3d rColl = collPt - x;

        Eigen::Vector3d vCollPt = v + angV.cross(rColl);
        double vCollPtNor = vCollPt.dot(nColl);

        Iinv = R * I0inv * R.transpose();

        double jDen = massInv + nColl.dot((Iinv * (rColl.cross(nColl)).cross(rColl)));
        double jn = (-(1.0 + simParams.restitutionCoefficient) * vCollPtNor) / jDen;

        // for testing
//        jn = 0.0;

        Eigen::Vector3d deltaAngV = jn * Iinv * (rColl.cross(nColl));
//        std::cout << "I: \n" << I << "\n";
//        std::cout << "Iinv: \n" << Iinv << "\n";
        std::cout << "deltaAngV: " << deltaAngV.transpose() << "\n";
//        std::cout << "vn bef: " << (v + angV.cross(rColl)).dot(nColl) << "\n";
//        angV += deltaAngV;
//        std::cout << "angV: " << angV.transpose() << "\n";
        Eigen::Vector3d deltaV = massInv * jn * nColl;
//        std::cout << "v bef: " << v.transpose() << "\n";
//        v += deltaV;
        std::cout << "deltaV: " << deltaV.transpose() << "\n";
//        std::cout << "v aft: " << v.transpose() << "\n";
//        std::cout << "vn aft: " << (v + angV.cross(rColl)).dot(nColl) << "\n";

        if (std::abs(cData.corrVec.x()) > std::abs(corrVecX))
            corrVecX = cData.corrVec.x();
        if (std::abs(cData.corrVec.y()) > std::abs(corrVecY))
            corrVecY = cData.corrVec.y();
        if (std::abs(cData.corrVec.z()) > std::abs(corrVecZ))
            corrVecZ = cData.corrVec.z();
        cData.deltaV = deltaV;
        cData.deltaAngV = deltaAngV;
//        corrVec += cData.corrVec;
    }
    std::cout<<"collDataSize: "<<collData.size()<<"\n";

    std::cout<<"v bef: "<<v.transpose()<<"\n";
    std::cout<<"ang v bef: "<<angV.transpose()<<"\n";
    for (int i = 0; i < collData.size(); ++i) {
        v += collData[i].deltaV;
        angV += collData[i].deltaAngV;
//        x += ((1 + 2e-2) * collData[i].corrVec);
    }
    std::cout<<"v aft: "<<v.transpose()<<"\n";
    std::cout<<"ang v afte: "<<angV.transpose()<<"\n";

    Eigen::Vector3d corrVec(corrVecX, corrVecY, corrVecZ);
//    corrVec /= collData.size();
    x += ((1 + 1e-2) * corrVec);
    for (int i = 0; i < vertices.size(); ++i) {
        verticesTemp[i] = vertices[i];
        vertices[i] = (R * vertices0[i]) + x;
    }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> RigidBody::getVelAcc(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, int integrationScheme) {
    Eigen::Vector3d fNet(0,0,0);
    for (const auto & forceField : forceFields) {
        fNet += forceField->getForce(x);
    }
    Eigen::Vector3d acc = fNet / mass;

    if (integrationScheme == 0) { // Euler
        Eigen::Vector3d vel = v;
        Eigen::Vector3d a = acc;
        return std::make_pair(vel, a);
    } else if (integrationScheme == 1) { // RK4
        // K1
        Eigen::Vector3d vk1 = v;
        Eigen::Vector3d ak1 = acc;

        // K2
        Eigen::Vector3d vk2 = v + ak1 * h * 0.5;
        Eigen::Vector3d ak2 = acc;

        // K3
        Eigen::Vector3d vk3 = v + ak2 * h * 0.5;
        Eigen::Vector3d ak3 = acc;

        // K4
        Eigen::Vector3d vk4 = v + ak3 * h;
        Eigen::Vector3d ak4 = acc;

        Eigen::Vector3d vel = (1.0 / 6.0) * (vk1 + 2 * vk2 + 2 * vk3 + vk4);
        Eigen::Vector3d a = (1.0 / 6.0) * (ak1 + 2 * ak2 + 2 * ak3 + ak4);
        return std::make_pair(vel, a);
    } else if (integrationScheme == 2) { // semi implicit euler
        Eigen::Vector3d vel = v;
        Eigen::Vector3d a = acc;
        vel += a * h;
        return std::make_pair(vel, a);
    }
    return std::make_pair(v, acc);
}

void RigidBody::step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams) {
    auto velAcc = getVelAcc(h, forceFields, simParams.integrationMethod);
    Eigen::Vector3d vel = velAcc.first;
    Eigen::Vector3d acc = velAcc.second;
//    std::cout<<"v: "<<v.transpose()<<"\n";
//    std::cout<<"fNet: "<<fNet.transpose()<<"\n";
//    std::cout<<"acc: "<<acc.transpose()<<"\n";
    x += h * vel;
    v += h * acc;
    Eigen::Matrix3d wstar;
    wstar << 0, -angV.z(), angV.y(),
            angV.z(), 0, -angV.x(),
            -angV.y(), angV.x(), 0;
    R += h * (wstar * R);
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2).normalize();

//    R.row(0).normalize();
//    R.row(1).normalize();
//    R.row(2).normalize();
//    std::cout<<"x: "<<x.transpose()<<"\n";

    for (int i = 0; i < vertices.size(); ++i) {
        verticesTemp[i] = vertices[i];
        vertices[i] = (R * vertices0[i]) + x;
    }

}

std::vector<Polygon>& RigidBody::getPolygons() {
    return polygons;
}

void RigidBody::loadMesh(const string &meshName)
{
    // Load geometry
    meshFilename = meshName;
    loadObj(meshFilename, posBuf, norBuf, texBuf);
    std::cout<<"Loaded obj\n";
}

void RigidBody::setObstacle(bool isObstacle) {
    this->isObstacle = isObstacle;
}

bool RigidBody::getObstacle() {
    return isObstacle;
}

void RigidBody::init()
{
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);


    glGenBuffers(1, &posBufID);
    glGenBuffers(1, &norBufID);
    glGenBuffers(1, &texBufID);

    // Send the position array to the GPU
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_STATIC_DRAW);

    // Send the normal array to the GPU
    glBindBuffer(GL_ARRAY_BUFFER, norBufID);
    glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_STATIC_DRAW);

    // Send the texcoord array to the GPU

    glBindBuffer(GL_ARRAY_BUFFER, texBufID);
    glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);

    // Unbind the arrays
//    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, Normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, TexCoords));
    glBindVertexArray(0);
    GLSL::checkError(GET_FILE_LINE);
}

void RigidBody::cleanupBuffers() {
    glDeleteBuffers(1, &posBufID);
    glDeleteBuffers(1, &norBufID);
    glDeleteBuffers(1, &texBufID);
    glDeleteVertexArrays(1, &VAO);
}

void RigidBody::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p)
{
    this->setProgram(p);
    glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
    glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
    glUniform1f(prog->getUniform("s"), 200.0f);
    MV->pushMatrix();
    MV->translate(x(0), x(1), x(2));
    MV->multMatrix(convertToGLMMat(R));
    MV->scale(1.0);

    glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));

    assert(prog);
    glBindVertexArray(VAO);
    int h_pos = prog->getAttribute("aPos");
    glEnableVertexAttribArray(h_pos);
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);

    int h_nor = prog->getAttribute("aNor");
    glEnableVertexAttribArray(h_nor);
    glBindBuffer(GL_ARRAY_BUFFER, norBufID);
    glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);

    int h_tex = prog->getAttribute("aTex");
    glEnableVertexAttribArray(h_tex);
    glBindBuffer(GL_ARRAY_BUFFER, texBufID);
    glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);

    // Draw
    int count = (int)posBuf.size()/3; // number of indices to be rendered
    glDrawArrays(GL_TRIANGLES, 0, count);

    glDisableVertexAttribArray(h_tex);
    glDisableVertexAttribArray(h_nor);
    glDisableVertexAttribArray(h_pos);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    GLSL::checkError(GET_FILE_LINE);

    MV->popMatrix();
}
