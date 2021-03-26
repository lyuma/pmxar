/*
pmxar (MMDGeometryLoader.mm)
Copyright (c) 2018 Lyuma

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#import "MMDGeometryLoader.h"
#include "MMDPmxParser.h"
#include "MMDPmdParser.h"
#include "PmdPmxConverter.h"
#include "TargetConditionals.h"
#include "btBulletDynamicsCommon.h"
#include <fstream>
#include "Node.h"
#include <map>
#include <vector>
#include <string>
#include <set>

#ifdef MACAPP
#define UIColor NSColor
#define UIImage NSImage
#endif

std::string convertSjisToUtf8(const std::string &in) {
    NSString *boneName = [NSString stringWithCString:in.c_str() encoding:NSShiftJISStringEncoding];
    char *outBuf = new char[in.size() * 4 + 10];
    [boneName getCString:outBuf maxLength:(in.size() * 4 + 10) encoding:NSUTF8StringEncoding];
    std::string ret (outBuf);
    delete []outBuf;
    return ret;
}

inline simd_float3 applyTransform(const simd_float4x4 &transform, simd_float3 vec) {
    simd_float4 vec4 = {vec.x, vec.y, vec.z, 1.0f};
    return simd_mul(transform, vec4).xyz;
}

inline simd_quatf eulerToQuat(float (&orient)[3]) {
    return simd_mul(simd_mul(
        simd_quaternion(orient[1], (simd_float3){0.f, 1.f, 0.f}),
        simd_quaternion(orient[0], (simd_float3){1.f, 0.f, 0.f})),
        simd_quaternion(orient[2], (simd_float3){0.f, 0.f, 1.f}));
        //float (&pos)[3], pos[0], pos[1], pos[2]);
}

inline btQuaternion eulerToBtQuat(float (&orient)[3]) {
    return btQuaternion(btVector3(0, 1, 0), orient[1]) *
        btQuaternion(btVector3(1, 0, 0), orient[0]) *
        btQuaternion(btVector3(0, 0, 1), orient[2]);
        //float (&pos)[3], pos[0], pos[1], pos[2]);
}

inline btVector3 arrToBtVec3(float (&pos)[3]) {
    return btVector3(pos[0], pos[1], pos[2]);
}

inline btTransform quatToBtTransform(simd_quatf quat, simd_float3 origin) {
    return btTransform(btQuaternion(quat.vector.x, quat.vector.y, quat.vector.z, quat.vector.w),
        btVector3(origin.x, origin.y, origin.z));
}

inline simd_float3 arrToSimdVec3(float (&pos)[3]) {
    return {pos[0], pos[1], pos[2]};
}

inline simd_quatf btTransformToSimdQuat(const btTransform&trans) {
    btQuaternion btq = trans.getRotation();
    return simd_quaternion(btq.getX(), btq.getY(), btq.getZ(), btq.getW());
}

inline simd_float3 btTransformToSimdPosition(const btTransform&trans) {
    return {trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()};
}


struct IKData {
    struct IKLink {
        int index;
        bool enabled;
        bool limitation; // minAngle and maxAngle not currently implemented.

        IKLink(int index, bool enabled, bool limitation)
            : index(index), enabled(enabled), limitation(limitation) {
        }
    };

    int effector;
    int target;
    int iterationCount;
    float loopAngleLimit;
    std::vector<IKLink> links;
    
    IKData(int effector, int target, int iterationCount, float loopAngleLimit)
        : effector(effector),
        target(target),
        iterationCount(iterationCount),
        loopAngleLimit(loopAngleLimit) {
    }
};

class MMDNode {
    bool hasDesiredOrientation;
    simd_quatf simdDesiredOrientation;
    bool hasDesiredPosition;
    simd_float3 simdDesiredPosition;
public:
    MMDNode *parentNode;
    simd_quatf simdPrevOrientation;
    simd_quatf simdOrientation;
    simd_float3 simdPosition;
    simd_quatf simdInitialOrientation;
    simd_float3 simdInitialPosition;
    simd_quatf simdWorldOrientation;
    simd_float3 simdWorldPosition;
    //float loopAngleLimit;
    //bool limitation;
    int boneIndex;
    pmx::PmxBone &bone;
    
    //MMDNode(const MMDNode &oth) = delete;
    //MMDNode &operator=(const MMDNode &oth) = delete;

    MMDNode(int boneIndex, pmx::PmxBone &bone, MMDNode *parent, simd_quatf o, simd_float3 p) : hasDesiredOrientation(false), hasDesiredPosition(false),
        parentNode(parent), simdOrientation(o), simdPosition(p), boneIndex(boneIndex), bone(bone) {
        simdPrevOrientation = simdOrientation;
        updateWorld();
        simdInitialOrientation = simdWorldOrientation;
        simdInitialPosition = simdWorldPosition;
    }

    void updateWorld() {
        if (parentNode) {
            simdWorldPosition = simd_act(parentNode->simdWorldOrientation, simdPosition) + parentNode->simdWorldPosition;
            simdWorldOrientation = simd_mul(parentNode->simdWorldOrientation, simdOrientation);
        } else {
            simdWorldPosition = simdPosition;
            simdWorldOrientation = simdOrientation;
        }
    }

    class btTransform btTransform() {
        simd_float3 &pos = hasDesiredPosition ? simdDesiredPosition : simdWorldPosition;
        simd_quatf &orient = hasDesiredOrientation ? simdDesiredOrientation : simdWorldOrientation;
        return quatToBtTransform(orient, pos);
    }

    class btTransform btInitialTransform() {
        return quatToBtTransform(simdInitialOrientation, simdInitialPosition);
    }
    
    void setDesiredOrientation(simd_quatf o) {
        hasDesiredOrientation = true;
        simdDesiredOrientation = o;
    }
    void setDesiredPosition(simd_float3 p) {
        hasDesiredPosition = true;
        simdDesiredPosition = p;
    }
    void setToDesiredTransform() {
        bool update = (hasDesiredOrientation || hasDesiredPosition);
        if (hasDesiredOrientation) {
            simdWorldOrientation = simdDesiredOrientation;
            if (parentNode == nullptr) {
                simdOrientation = simdWorldOrientation;
            } else {
                simdOrientation = simd_mul(simd_inverse(parentNode->simdWorldOrientation), simdWorldOrientation);
            }
            hasDesiredOrientation = false;
        }
        if (hasDesiredPosition) {
            simdWorldPosition = simdDesiredPosition;
            if (parentNode == nullptr) {
                simdPosition = simdWorldPosition;
            } else {
                simdPosition = simd_act(simd_inverse(parentNode->simdWorldOrientation), simdWorldPosition - parentNode->simdWorldPosition);
            }
            hasDesiredPosition = false;
        }
        if (update) {
            updateWorld();
        }
    }
    /*
    simd_quatf getDesiredOrientation() {
        return simdDesiredOrientation;
    }
    simd_float3 getDesiredPosition() {
        return simdDesiredPosition;
    }
    bool desiredOrientation() {
        return hasDesiredOrientation;
    }
    bool desiredPosition() {
        return hasDesiredPosition;
    }*/
};


// http://sio2interactive.forumotion.net/t599-enabling-bullet-debug-draw-code-included
class GLDebugDrawer : public btIDebugDraw
{
    int m_debugMode;
  public:
    std::vector<float> debugWire;
    std::vector<int> wireIndices;
    std::multimap<std::string, btVector3> text;
    btVector3 overrideColor;
    bool doOverrideColor;

    GLDebugDrawer() : m_debugMode(0), doOverrideColor(false) {}

    void clearNewFrame() {
        debugWire.clear();
        wireIndices.clear();
        text.clear();
    }

    virtual void drawLine(const btVector3& from,const btVector3& to,const btVector3& requestedColor) {
       //      if (m_debugMode > 0)
       {
          const btVector3 &color = doOverrideColor ? overrideColor : requestedColor;
          const int VERTEX_SZ = 14;
          float tmp[ VERTEX_SZ ] = { from.getX(), from.getY(), from.getZ(),
                        color.getX(), color.getY(), color.getZ(), 1.0f,
                        to.getX(), to.getY(), to.getZ(),
                        color.getX(), color.getY(), color.getZ(), 1.0f
          };
          debugWire.insert(debugWire.end(), tmp, tmp + VERTEX_SZ );
          // silly...
          wireIndices.push_back((int)wireIndices.size());
          wireIndices.push_back((int)wireIndices.size());
          /*
          glPushMatrix();
          {
             glColor4f(color.getX(), color.getY(), color.getZ(), 1.0f);
             glVertexPointer( 3,
                         GL_FLOAT,
                         0,
                         &tmp );
           
             glPointSize( 5.0f );
             glDrawArrays( GL_POINTS, 0, 2 );
             glDrawArrays( GL_LINES, 0, 2 );
          }
          glPopMatrix();
          */
       }
    }

    virtual void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) {
      //btVector3 to=pointOnB+normalOnB*distance;
      //const btVector3&from = pointOnB;
      //glColor4f(color.getX(), color.getY(), color.getZ(), 1.0f);
      
      //GLDebugDrawer::drawLine(from, to, color);
      
      //glRasterPos3f(from.x(),  from.y(),  from.z());
      //char buf[12];
      //sprintf(buf," %d",lifeTime);
      //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
   }
    virtual void reportErrorWarning(const char* warningString) {
        NSLog(@"%s", warningString);
    }
    virtual void draw3dText(const btVector3& location,const char* textString) {
       //glRasterPos3f(location.x(),  location.y(),  location.z());
       //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
       text.insert(std::make_pair(std::string(textString), location));
    }
    virtual void setDebugMode(int debugMode) { m_debugMode = debugMode; }
    virtual int getDebugMode() const { return m_debugMode;}
    void changeFlag(int flag, bool enable) {
        int mask = enable * flag;
        int invmask = ~((!enable) * flag);
        m_debugMode = (m_debugMode & invmask) | mask;
    }
    void setOverrideColor(const btVector3 & color) {
        overrideColor = color;
        doOverrideColor = true;
    }
    void clearOverrideColor() {
        doOverrideColor = false;
    }
};


class MMDGeometryState : public pmx::PmxModel {
public:
    pmx::PmxModel &model;
    std::vector<IKData> iks;
    std::vector<btTransform> rigidBodyBindTransforms;
    std::vector<btRigidBody*> rigidBodies;
    std::vector<btMotionState*> motionStates;
    double lastPhysicsTime; // should we use fixed point?
    std::vector<MMDNode> bones;

    // Physics state:
    btCollisionConfiguration *m_collisionConfiguration;
    btSequentialImpulseConstraintSolver* m_solver;
    btDbvtBroadphase *m_broadphase;
    btCollisionDispatcher *m_dispatcher;
    btDefaultCollisionConfiguration *btDefaultCollisionConfiguration;
    btDiscreteDynamicsWorld *m_dynamicsWorld;
    std::vector<btCollisionShape*> m_collisionShapes;
    std::vector<btRigidBody*> m_rigidBodies;
    GLDebugDrawer m_debugDrawer;

    void fixupModel() {
        bool foundSubete = false;
        for (int i = 0; i < model.bone_count; i++) {
            const pmx::PmxBone &bone = model.bones[i];
            if (bone.bone_name == "全ての親") {
                foundSubete = true;
            }
        }
        if (!foundSubete) {
            pmx::PmxBone *newBoneList = new pmx::PmxBone[(unsigned)model.bone_count + 1];
            for (int i = 0; i < model.bone_count; i++) {
                newBoneList[i + 1] = std::move(model.bones[i]);
            }
            newBoneList[0].bone_english_name = "root node";
            newBoneList[0].bone_name = "全ての親";
            newBoneList[0].bone_flag = 0x5;
            newBoneList[0].grant_parent_index = -1;
            newBoneList[0].parent_index = -1;
            newBoneList[0].target_index = -1;
            newBoneList[0].ik_target_bone_index = -1;
            model.bones.reset(newBoneList);
            model.bone_count++;
            for (int i = 0; i < model.vertex_count; i++) {
                switch (model.vertices[i].skinning_type) {
                case pmx::PmxVertexSkinningType::BDEF1:
                    ((pmx::PmxVertexSkinningBDEF1*)(model.vertices[i].skinning.get()))->bone_index++;
                    break;
                case pmx::PmxVertexSkinningType::BDEF2:
                {
                    auto b2 = ((pmx::PmxVertexSkinningBDEF2*)(model.vertices[i].skinning.get()));
                    if (b2->bone_index1 >= 0) { b2->bone_index1++; }
                    if (b2->bone_index2 >= 0) { b2->bone_index2++; }
                    break;
                }
                case pmx::PmxVertexSkinningType::BDEF4:
                {
                    auto b4 = ((pmx::PmxVertexSkinningBDEF4*)(model.vertices[i].skinning.get()));
                    if (b4->bone_index1 >= 0) { b4->bone_index1++; }
                    if (b4->bone_index2 >= 0) { b4->bone_index2++; }
                    if (b4->bone_index3 >= 0) { b4->bone_index3++; }
                    if (b4->bone_index4 >= 0) { b4->bone_index4++; }
                    break;
                }
                case pmx::PmxVertexSkinningType::QDEF:
                {
                    auto q4 = ((pmx::PmxVertexSkinningQDEF*)(model.vertices[i].skinning.get()));
                    if (q4->bone_index1 >= 0) { q4->bone_index1++; }
                    if (q4->bone_index2 >= 0) { q4->bone_index2++; }
                    if (q4->bone_index3 >= 0) { q4->bone_index3++; }
                    if (q4->bone_index4 >= 0) { q4->bone_index4++; }
                    break;
                }
                case pmx::PmxVertexSkinningType::SDEF:
                {
                    auto s2 = ((pmx::PmxVertexSkinningSDEF*)(model.vertices[i].skinning.get()));
                    if (s2->bone_index1 >= 0) { s2->bone_index1++; }
                    if (s2->bone_index2 >= 0) { s2->bone_index2++; }
                    break;
                }
                }
            }
            for (int i = 1; i < model.bone_count; i++) {
                if (model.bones[i].grant_parent_index >= 0) {
                    model.bones[i].grant_parent_index++;
                }
                if (model.bones[i].ik_target_bone_index >= 0) {
                    model.bones[i].ik_target_bone_index++;
                }
                if (model.bones[i].target_index >= 0) {
                    model.bones[i].target_index++;
                }
                model.bones[i].parent_index++;
                for (int j = 0; j < model.bones[i].ik_link_count; j++) {
                    model.bones[i].ik_links[j].link_target++;
                }
            }
            for (int i = 0; i < model.morph_count; i++) {
                auto &morph = model.morphs[i];
                if (morph.morph_type == pmx::MorphType::Bone) {
                    for (int j = 0; j < morph.offset_count; j++) {
                        morph.bone_offsets[j].bone_index++;
                    }
                }
            }
            for (int i = 0; i < model.rigid_body_count; i++) {
                if (model.rigid_bodies[i].target_bone >= 0) {
                    model.rigid_bodies[i].target_bone++;
                }
            }
        }
    }

    MMDNode *rootNode() {
        return &(bones[0]);
    }
    
    MMDNode *getBoneAtIndex(int idx) {
        if (idx < 0 || idx >= (int)bones.size()) {
            return nullptr;
        }
        return &(bones[idx]);
    }
    
    MMDGeometryState()
        : model(*this),
            lastPhysicsTime(0),
            m_collisionConfiguration(nullptr),
            m_solver(nullptr),
            m_broadphase(nullptr),
            m_dispatcher(nullptr),
            btDefaultCollisionConfiguration(nullptr),
            m_dynamicsWorld(nullptr) {
        model.Init();
    }
    
    void setupIK();
    void initGround();
    void setupPhysics();
    bool parse(std::ifstream *is);
    void tick(double time);
};

void MMDGeometryState::initGround() {
    btCollisionShape *groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
    m_collisionShapes.push_back(groundShape);
    btTransform groundTransform;
    groundTransform.setIdentity();
    //groundTransform.setOrigin(btVector3(0,-50,0));
    btScalar mass(0.);
    btVector3 localInertia(0, 0, 0);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, groundShape, localInertia);
    btRigidBody* body = new btRigidBody(cInfo);
    body->setUserIndex(-1);
    //body->setContactStiffnessAndDamping(300,10);
    body->setContactStiffnessAndDamping(0,0);
    // body->setFriction(.25);
    // kinematic object:
    body->setCollisionFlags(body-> getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    body->setActivationState(DISABLE_DEACTIVATION);
    m_dynamicsWorld->addRigidBody(body);
}

void MMDGeometryState::setupPhysics() {
    m_collisionConfiguration = new class btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_broadphase = new btDbvtBroadphase();
    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    m_solver = new btSequentialImpulseConstraintSolver;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
    /*
    m_dynamicsWorld->getSolverInfo().m_erp2 = 0.f;
    m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.f;
    m_dynamicsWorld->getSolverInfo().m_numIterations = 3;
    m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
    m_dynamicsWorld->getSolverInfo().m_splitImpulse = false;
    */

    m_dynamicsWorld->setGravity(btVector3(0, -9.8 * 10, 0));

//btIDebugDraw::DBG_DrawWireframe |
    m_debugDrawer.setDebugMode(
        btIDebugDraw::DBG_DrawConstraints |
        btIDebugDraw::DBG_DrawConstraintLimits |
        btIDebugDraw::DBG_DrawText |
        btIDebugDraw::DBG_DrawFeaturesText |
        btIDebugDraw::DBG_DrawFrames);



    m_dynamicsWorld->setDebugDrawer(&m_debugDrawer);
    //[self initGround];

    //rigidBodyDebugNodes = [[NSMutableArray alloc] initWithCapacity:model.rigid_body_count];
    
    //UIImage *debugImage = [UIImage imageNamed:@"floor.jpg"];


    for (int i = 0; i < model.rigid_body_count; i++) {
        pmx::PmxRigidBody &rb = model.rigid_bodies[i];
        btCollisionShape* childShape;
        //SCNGeometry *debugGeom;
        switch (rb.shape) {
        case 0: // sphere
        default:
            // Ignore size[1], size[2]
            childShape = new btSphereShape(btScalar(rb.size[0]));
            //debugGeom = [SCNSphere sphereWithRadius:rb.size[0]*2];
            break;
        case 1: // box
            childShape = new btBoxShape(btVector3(rb.size[0], rb.size[1], rb.size[2]));
            //debugGeom = [SCNBox boxWithWidth:rb.size[0]*2 height:rb.size[1]*2 length:rb.size[2]*2 chamferRadius:0];
            break;
        case 2:
            childShape = new btCapsuleShape(rb.size[0], rb.size[1]);
            //debugGeom = [SCNCapsule capsuleWithCapRadius:rb.size[0]*2 height:rb.size[1]*2];
            break;
        }
        /*
        // Work in bone space instead of model space?
        if (rb.target_bone >= 0 && rb.target_bone <= self.boneCount) {
            SCNMatrix4 boneTransform = [self.rootNode convertTransform:SCNMatrix4Identity fromNode:[self boneAtIndex:rb.target_bone]];
            SCNMatrix4 inverseBindTransform = SCNMatrix4Invert(boneTransform);
            transform = SCNMatrix4Mult(transform, inverseBindTransform);
        }
        */
        btTransform btransform(eulerToBtQuat(rb.orientation), arrToBtVec3(rb.position));
        btScalar mass(rb.mass);
        btVector3 localInertia(0, 0, 0);
        childShape->calculateLocalInertia(mass,localInertia);
        m_collisionShapes.push_back(childShape);
        //SCNMatrix4 boneInverseBind;
        btTransform boneInverseBind;
        if (rb.target_bone < 0 || rb.target_bone >= (int)bones.size()) {
            //boneInverseBind = SCNMatrix4Identity;
            boneInverseBind = btTransform();
        } else {
            MMDNode &bone = bones[rb.target_bone];
            //boneInverseBind = SCNMatrix4Invert([self.rootNode convertTransform:SCNMatrix4Identity fromNode:[self boneAtIndex:rb.target_bone]]); // FIXME: assumes rootNode 0,0,0
            //boneInverseBind = SCNMatrix4Invert([[self boneAtIndex:rb.target_bone] convertTransform:transform fromNode:self.rootNode.parentNode]); // FIXME: assumes rootNode 0,0,0
            boneInverseBind = btransform;
            btransform = bone.btTransform() * bone.btInitialTransform().inverse() * btransform;
            //boneInverseBind = bone.btTransform();
            //boneInverseBind.setOrigin(btVector3(10. + i * .003, -i * 0.001, i * .005 + .004));
           // boneInverseBind.setOrigin(boneInverseBind.getOrigin() + btVector3(-bone.parentNode->simdWorldPosition.x, -bone.parentNode->simdWorldPosition.y, -bone.parentNode->simdWorldPosition.z));
        }
        /*
        SCNMaterial *debugMater = [SCNMaterial material];
        debugMater.diffuse.contents = debugImage;
        debugGeom.materials = @[debugMater];
        SCNNode* rigidBodyDebugNode = [SCNNode nodeWithGeometry:debugGeom];
        [rigidBodyDebugNodes addObject:rigidBodyDebugNode];
        [rigidBodyDebugRoot addChildNode:rigidBodyDebugNode];
        */
        //SCNMatrix4 newMat = SCNMatrix4Mult(transform, boneInverseBind);
        rigidBodyBindTransforms.push_back(boneInverseBind);//newMat));
        //MMDMotionState* myMotionState = new MMDMotionState(this, i, rb, btransform);
        btDefaultMotionState* myMotionState = new btDefaultMotionState(btransform);
        myMotionState->m_userPointer = (void*)(intptr_t)i;
        btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, childShape, localInertia);
        // vs setDamping??
        cInfo.m_linearDamping = rb.move_attenuation;
        cInfo.m_angularDamping = rb.rotation_attenuation;
        cInfo.m_friction = rb.friction;
        // cInfo.m_rollingFriction = rb.friction; // fixme: necessary?
        // cInfo.m_spinningFriction = rb.friction; // fixme: necessary?
        cInfo.m_restitution = rb.repulsion; // recoil
        btRigidBody* body = new btRigidBody(cInfo);
        body->setUserIndex(i);
        body->setUserPointer(this);
        if (rb.physics_calc_type == 0) {
            body->setCollisionFlags(body-> getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
            body->setActivationState(DISABLE_DEACTIVATION); //?
            body->setCustomDebugColor(btVector3(1,0,0.5f));
        } else if (rb.physics_calc_type == 1) {
            // TODO: not pivoted to bone?
            body->setCustomDebugColor(btVector3(0,1,0));
        } else {
            body->setCustomDebugColor(btVector3(0,0,2.f/rb.physics_calc_type));
        }
        // body->setActivationState(DISABLE_DEACTIVATION); //not always?
        body->setSleepingThresholds(0, 0);
        body->setDamping(rb.move_attenuation, rb.rotation_attenuation);
        body->setWorldTransform(btransform);
        body->setGravity(btVector3(0, -9.8f, 0));
        m_dynamicsWorld->addRigidBody(body,
            (uint16_t)(1 << rb.group),
            rb.mask);
        rigidBodies.push_back(body);
    }
    for (int i = 0; i < model.joint_count; i++) {
        //pmx::PmxJointType &jtype = model.joints[i].joint_type;
        pmx::PmxJointParam &jt = model.joints[i].param;
        if (jt.rigid_body1 < 0 || jt.rigid_body1 >= model.rigid_body_count ||
            jt.rigid_body2 < 0 || jt.rigid_body2 >= model.rigid_body_count) {
            continue; // invalid data
        }
        btRigidBody* body1 = rigidBodies[jt.rigid_body1];
        btRigidBody* body2 = rigidBodies[jt.rigid_body2];
        if (!body1 || !body2) {
            continue;
        }
        btTransform btransform (eulerToBtQuat(jt.orientaiton), arrToBtVec3(jt.position));
        btTransform transform1, transform2;
        body1->getMotionState()->getWorldTransform( transform1 );
        body2->getMotionState()->getWorldTransform( transform2 );

        btTransform body1Trans = transform1.inverseTimes(btransform);
        btTransform body2Trans = transform2.inverseTimes(btransform);
        btTypedConstraint *constraint;
        // No implementations currently support type other than 0
        /*
        switch (jtype) {
        case pmx::PmxJointType::Generic6DofSpring:
        */
        {
            NSLog(@"joint %d[%d]<%f,%f,%f>-<%f,%f,%f>%d[%d] linear <%f,%f,%f>/<%f,%f,%f> movcoef:[%f,%f,%f] rotcoef:[%f,%f,%f]", jt.rigid_body1, model.rigid_bodies[jt.rigid_body1].target_bone, body1Trans.getOrigin().x(), body1Trans.getOrigin().y(), body1Trans.getOrigin().z(), body2Trans.getOrigin().x(), body2Trans.getOrigin().y(), body2Trans.getOrigin().z(), jt.rigid_body2, model.rigid_bodies[jt.rigid_body2].target_bone, jt.move_limitation_min[0], jt.move_limitation_min[1], jt.move_limitation_min[2], jt.move_limitation_max[0], jt.move_limitation_max[1], jt.move_limitation_max[2],
                jt.spring_move_coefficient[0],
                jt.spring_move_coefficient[1],
                jt.spring_move_coefficient[2],
                jt.spring_rotation_coefficient[0],
                jt.spring_rotation_coefficient[1],
                jt.spring_rotation_coefficient[2]);
            btGeneric6DofSpringConstraint *con6 = new btGeneric6DofSpringConstraint(
                *body1, *body2, body1Trans, body2Trans, true);
            constraint = con6;
            con6->setLinearLowerLimit(arrToBtVec3(jt.move_limitation_min));
            con6->setLinearUpperLimit(arrToBtVec3(jt.move_limitation_max));
            con6->setAngularLowerLimit(arrToBtVec3(jt.rotation_limitation_min));
            con6->setAngularUpperLimit(arrToBtVec3(jt.rotation_limitation_max));
            for (int j = 0; j < 3; j++) {
                if (jt.spring_move_coefficient[j]  > 7) {
                    con6->enableSpring(j, true); // FIXME: test
                    con6->setStiffness(j, jt.spring_move_coefficient[j]);
                }
                if (jt.spring_rotation_coefficient[j]) {
                    con6->enableSpring(j + 3, true); // FIXME: test
                    con6->setStiffness(j + 3, jt.spring_rotation_coefficient[j]);
                }
            }
            for (int j = 0; j < 6; j++) {
                // this parameter is from http://www20.atpages.jp/katwat/three.js_r58/examples/mytest37/mmd.three.js
                con6->setParam(BT_CONSTRAINT_STOP_ERP, 0.475, j);
            }
        }
        constraint->setDbgDrawSize(.1);
        /*
            break;
        case pmx::PmxJointType::Generic6Dof:
        {
            btGeneric6DofConstraint *con6 = new btGeneric6DofConstraint(
                *body1, *body2, body1Trans, body2Trans, true);
            constraint = con6;
            con6->setLinearLowerLimit(arrToVec3(jt.move_limitation_min));
            con6->setLinearUpperLimit(arrToVec3(jt.move_limitation_max));
            con6->setAngularLowerLimit(arrToVec3(jt.rotation_limitation_min));
            con6->setAngularUpperLimit(arrToVec3(jt.rotation_limitation_max));
            break;
        }
        case pmx::PmxJointType::Point2Point:
            constraint = new btPoint2PointConstraint(*body1, *body2, body1Trans.getOrigin(), body2Trans.getOrigin());
            break;
        case pmx::PmxJointType::ConeTwist:
            constraint = new btConeTwistConstraint(*body1, *body2, body1Trans, body2Trans);
            break;
        case (pmx::PmxJointType)4: // should be slider
        case pmx::PmxJointType::Slider:
        {
            btSliderConstraint *cons = new btSliderConstraint(*body1, *body2, body1Trans, body2Trans, true);
            constraint = cons;
            cons->setLowerLinLimit(jt.move_limitation_min[0]);
            cons->setUpperLinLimit(jt.move_limitation_max[0]);
            cons->setLowerAngLimit(jt.rotation_limitation_min[0]);
            cons->setUpperAngLimit(jt.rotation_limitation_max[0]);
            break;
        }
        case pmx::PmxJointType::Hinge:
        {
            btHingeConstraint *cons = new btHingeConstraint(*body1, *body2, body1Trans, body2Trans);
            constraint = cons;
            cons->setLimit(jt.rotation_limitation_min[0],
                    jt.rotation_limitation_max[0]);
            break;
        }
        }
        */
        m_dynamicsWorld->addConstraint(constraint);
    }
    // TODO: soft bodies
}

void MMDGeometryState::setupIK() {
    for (int i = 0; i < model.bone_count; i++) {
        pmx::PmxBone &pmxbone = model.bones[i];
        if (!(pmxbone.bone_flag & (1<<5))) {
            continue;
        }
        int effectorId = pmxbone.ik_target_bone_index;
        if (effectorId < 0 || effectorId >= (int)bones.size()) {
            continue;
        }
        int targetId = i;
        MMDNode &effector = bones[effectorId];
        MMDNode &target = bones[targetId];
        simd_float3 targetPos = target.simdWorldPosition; // just debugging
        NSLog(@"Starting %s -> %s [%f,%f,%f]", effector.bone.bone_name.c_str(), target.bone.bone_name.c_str(), targetPos.x, targetPos.y, targetPos.z);
        
        iks.emplace_back(effectorId, targetId, pmxbone.ik_loop, pmxbone.ik_loop_angle_limit);
        IKData &ik = iks.back();
        for (int j = 0; j < pmxbone.ik_link_count; j++) {
            pmx::PmxIkLink &pmxlink = pmxbone.ik_links[j];
            if (pmxlink.link_target < 0 || pmxlink.link_target >= (int)bones.size()) {
                continue;
            }
            MMDNode &linkNode = bones[pmxlink.link_target];
            bool angle_limitation = /*pmxlink.angle_lock ||*/ linkNode.bone.bone_name.find("ひざ") != std::string::npos;
            ik.links.emplace_back(pmxlink.link_target, true, angle_limitation);
            NSLog(@"IK Link %s <min:%f,%f,%f> <max:%f,%f,%f>", linkNode.bone.bone_name.c_str(),
                pmxlink.min_radian[0], pmxlink.min_radian[1], pmxlink.min_radian[2],
                pmxlink.max_radian[0], pmxlink.max_radian[1], pmxlink.max_radian[2]);
        }
    }
}
    
bool MMDGeometryState::parse(std::ifstream *ifs) {
    if (!model.Read(ifs) || model.bone_count < 1) {
        ifs->seekg(0, ios_base::beg);
        std::unique_ptr<pmd::PmdModel> pmdModel (
                    pmd::PmdModel::LoadFromStream(ifs));
        if (!pmdModel.get()) {
            return false;
        }
        if (!convertPmdToPmx(*pmdModel, model)) {
            return false;
        }
    }
    fixupModel();
    bones.reserve(model.bone_count);
    MMDNode *lastRootNode = nullptr;
    for (int i = 0; i < model.bone_count; i++) {
        pmx::PmxBone &pmxbone = model.bones[i];
        MMDNode *parentNode;
        if (i == 0) {
            parentNode = nullptr;
        } else if (pmxbone.parent_index >= i) {
            return false; // Parent nodes must be ordered.
        } else if (pmxbone.parent_index < 0) {
            parentNode = lastRootNode;
        } else {
            parentNode = &bones[pmxbone.parent_index];
        }
        simd_float3 simdPosition = {pmxbone.position[0], pmxbone.position[1], pmxbone.position[2]};
        simd_quatf simdOrientation = simd_quaternion(0.f, 0.f, 0.f, 1.f);
        if (i != 0) {
            // Assume all nodes have identity orientation.
            simdPosition = simdPosition - parentNode->simdInitialPosition;
        }
        bones.emplace_back(i, pmxbone, parentNode, simdOrientation, simdPosition);
        MMDNode &thisBone = bones[i];
        if (pmxbone.parent_index < 0) {
            lastRootNode = &thisBone;
        }
    }
    setupIK();
    // TODO: physics state?
    // TODO: tick function
    // TODO: position / orientation donation
    // TODO: should motion happen here rather than at the SCNNode layer?
    return true;
}

void MMDGeometryState::tick(double time) {
    float timeDiff = time - lastPhysicsTime;
    bool firstTick = false;
    if (lastPhysicsTime == 0) {
        lastPhysicsTime = time;
        timeDiff = 0;
        firstTick = true;
    }
    //std::set<int> updatedBones;
    // Taken from three.js CCDIKSolver.js
    //for (unsigned i = 0; i < iks.size(); i++) {
    //    const IKData &data = iks[i];
    for (auto &ik : iks) {
    // Requires lots of iterations: 600 -> 6300, 400->4800
      for (int foo=0; foo< ik.iterationCount/2; foo++) {
        MMDNode &effector = bones[ik.effector];
        MMDNode &target = bones[ik.target];
        target.updateWorld();
        effector.updateWorld();
        simd_float3 targetPos = target.simdWorldPosition;
        std::vector<simd_quatf> ikquats(ik.links.size());
        std::vector<simd_quatf> ikworldquats(ik.links.size());
        std::vector<simd_float3> ikpos(ik.links.size());
        for (size_t i = 0; i < ik.links.size(); i++) {
            auto &iklink = ik.links[i];
            MMDNode &linkNode = bones[iklink.index];
            linkNode.updateWorld();
            ikquats[i] = (linkNode.simdOrientation);
            ikworldquats[i] = (linkNode.simdWorldOrientation);
            ikpos[i] = (linkNode.simdPosition);
        }
        //NSLog(@"Starting %@ -> %@ [%f,%f,%f]", effector.name, target.name, targetPos.x, targetPos.y, targetPos.z);
        for (int iteration = 0; iteration < 2; iteration++) {//ik.iterationCount
            //bool rotated = false;
            int which = 0;
            //for (auto &iklink : ik.links) {
            for (size_t i = 0; i < ik.links.size(); i++) {
                auto &iklink = ik.links[i];
                simd_quatf &origQuat = ikquats[i];
                //simd_quatf &origWorldQuat = ikworldquats[i];
                //simd_float3 &origPos = ikpos[i];
                which++;
                if (!iklink.enabled) {
                    break;
                }
                MMDNode &linkNode = bones[iklink.index];
                linkNode.parentNode->updateWorld();
                linkNode.updateWorld();
                //NSLog(@"IK Link %@ [From %@ -> %@", linkNode.name, effector.name, target.name);
                simd_float3 linkPos = linkNode.simdWorldPosition;//simd_float3(simd_mul(linkNode.parentNode.simdWorldTransform, simd_float4{origPos.x, origPos.y, origPos.z, 0.f}).xyz));
                //simd_quatf invLinkQuat = simd_inverse(simd_mul(rootQuatInv, linkNode.simdWorldOrientation));
                //simd_quatf invLinkQuat = simd_inverse(simd_mul(rootQuatInv, origWorldQuat));
                //simd_quatf invLinkQuat = simd_inverse(simd_mul(linkNode.parentNode->simdWorldOrientation, linkNode.simdOrientation));
                simd_quatf invLinkQuat = simd_inverse(simd_mul(linkNode.parentNode->simdWorldOrientation, origQuat));
                simd_float3 effectorPos = effector.simdWorldPosition;
                simd_float3 effectorVec = simd_normalize(simd_act(invLinkQuat, effectorPos - linkPos));
                simd_float3 targetVec = simd_normalize(simd_act(invLinkQuat, targetPos - linkPos));
                float angle = acosf(std::min(1.0f, std::max(-1.0f, simd_dot(targetVec, effectorVec))));
                if (angle < 1e-5) {
                    continue;
                }
                angle = std::min(angle, ik.loopAngleLimit);
                simd_float3 axis = simd_normalize(simd_cross(effectorVec, targetVec));
                simd_quatf q = simd_quaternion(angle, axis);
                /*
                q = simd_mul(simd_inverse(origQuat), q);
                angle = simd_angle(q);
                axis = simd_axis(q);
                angle = std::min(angle, ik.loopAngleLimit);
                q = simd_quaternion(angle, axis);
                */
                simd_quatf newQuaternion = simd_mul(origQuat, q);
                //simd_quatf newQuaternion = simd_mul(linkNode.simdOrientation, q);
                if (iklink.limitation) {
                    float c = std::min(1.0f, fabs(newQuaternion.vector.w));
                    float c2 = sqrtf(1 - c * c);
                    // limitation is (x=1, y=0, z=0) by default. No way to override currently.
                    //iklink.quaternion = iklink.limitation * c2;
                    //iklink.quaternion.vector.w = c;
                    newQuaternion = simd_quaternion(c2, 0.f, 0.f, -c);
                }
                linkNode.simdOrientation = newQuaternion;
                linkNode.updateWorld();
                //updatedBones.insert(iklink.index);
                //rotated = true;
            }
            /*if (!rotated) {
                break;
            }*/
        }
      }
    }
    /*
    if (firstTick) {
        for (int idx = 0; idx < model.rigid_body_count; idx++) {
            pmx::PmxRigidBody &rb = model.rigid_bodies[idx];
            if (rb.target_bone < 0 || rb.target_bone >= self.boneCount) {
                continue;
            }
            //const btTransform &prevBulletTransform = rigidBodies[idx]->getWorldTransform();
            if (rb.physics_calc_type != 1) {
                SCNNode *targetBone = [self boneAtIndex:rb.target_bone];
                btTransform newBoneTransform = scnToBtTransform([self.rootNode.parentNode convertTransform:SCNMatrix4Identity fromNode:targetBone]);
                rigidBodies[idx]->setWorldTransform(rigidBodyBindTransforms[idx] * newBoneTransform);
            }
        }
    }*/
    // Apply grants (gra
    for (int idx = 0 ; idx < model.bone_count; idx++) {
        pmx::PmxBone &bone = model.bones[idx];
        // bool isLocal = (bone.bone_flag & 0x80); // local flag unimplemented
        bool grantRotation = (bone.bone_flag & 0x0100);
        bool grantPosition = (bone.bone_flag & 0x0200);
        if ((grantRotation || grantPosition) && bone.grant_parent_index >= 0 && bone.grant_parent_index < model.bone_count) {
            // this node:
            MMDNode &boneNode = bones[idx];
            // granted parent node:
            MMDNode &grantNode = bones[bone.grant_parent_index];
            if (grantRotation) {
                simd_quatf q = boneNode.simdOrientation; // simd_quaternion(0.f, 0.f, 0.f, 1.f);
                q = simd_slerp(q, grantNode.simdOrientation, bone.grant_weight);
                boneNode.simdOrientation = q; // interpolate? simd_mul(boneNode.simdOrientation, q);
            }
            if (grantPosition) {
                boneNode.simdPosition = grantNode.simdPosition * bone.grant_weight;
            }
            boneNode.updateWorld();
            //updatedBones.insert(idx);
        }
    }
    for (int idx = 0; idx < model.rigid_body_count; idx++) {
        pmx::PmxRigidBody &rb = model.rigid_bodies[idx];
        if (rb.target_bone < 0 || rb.target_bone >= (int)bones.size()) {
            continue;
        }
        if (!rigidBodies[idx]) {
            continue;
        }
    /*
        if (updatedBones.find(idx) != updatedBones.end() && rb.physics_calc_type != 0) {
            MMDNode &targetBone = bones[rb.target_bone];
            targetBone.updateWorld();
            btTransform newBoneTransform = targetBone.btTransform() * targetBone.btInitialTransform().inverse() * rigidBodyBindTransforms[idx];
            if (rb.physics_calc_type != 0) {
                btTransform currentTrans = rigidBodies[idx]->getWorldTransform();
                newBoneTransform.setBasis(currentTrans.getBasis());
            }
            rigidBodies[idx]->setWorldTransform(newBoneTransform);
        }*/
        if (rb.physics_calc_type == 0) {
            MMDNode &targetBone = bones[rb.target_bone];
            targetBone.updateWorld();
            //btTransform newBoneTransform = rigidBodyBindTransforms[idx] * targetBone.btTransform();
            btTransform newBoneTransform = targetBone.btTransform() * targetBone.btInitialTransform().inverse() * rigidBodyBindTransforms[idx];
            /*if (rb.physics_calc_type != 0 && !firstTick) {
                btTransform currentTrans = rigidBodies[idx]->getWorldTransform();
                newBoneTransform.setBasis(currentTrans.getBasis());
            }*/
            //rigidBodies[idx]->setWorldTransform(newBoneTransform);
            rigidBodies[idx]->setCenterOfMassTransform(newBoneTransform);
            rigidBodies[idx]->getMotionState()->setWorldTransform(newBoneTransform);
        }
    }
    //static int i  = 0;
    if (timeDiff > 0) {
        m_debugDrawer.clearNewFrame();
        //m_debugDrawer.setOverrideColor(btVector3(1.f,0.5.f,0.f));
        //m_dynamicsWorld->debugDrawWorld();
        //m_debugDrawer.clearOverrideColor();
        
        
        // Turn off physics for now
         ////////////// m_dynamicsWorld->stepSimulation(timeDiff * 60, 1, btScalar(1.)/btScalar(60.));
         ////////////// m_dynamicsWorld->debugDrawWorld();
        lastPhysicsTime += timeDiff;
    }
    for (int idx = 0; idx < model.rigid_body_count; idx++) {
        if (!rigidBodies[idx]) {
            continue;
        }
        pmx::PmxRigidBody &rb = model.rigid_bodies[idx];
        btTransform worldTrans;
        rigidBodies[idx]->getMotionState()->getWorldTransform(worldTrans);
        if (rb.target_bone < 0 || rb.target_bone >= bones.size()) {
            continue;
        }
        MMDNode &targetBone = bones[rb.target_bone];
        btTransform bindInverse = rigidBodyBindTransforms[idx].inverse();
        btTransform newTrans = worldTrans * bindInverse * targetBone.btInitialTransform();
        targetBone.setDesiredOrientation(btTransformToSimdQuat(newTrans));
        if (rb.physics_calc_type == 1) {
            //if (targetBone.bone.bone_flag & 4) {
                /////////////targetBone.setDesiredPosition(btTransformToSimdPosition(newTrans));
            //}
        }
    }
    ///// Turn off physics for now
    ///////for (auto &bone : bones) {
    ///////    bone.setToDesiredTransform();
    ///////    bone.updateWorld();
    ///////}
    for (int idx = 0; idx < model.rigid_body_count; idx++) {
        if (!rigidBodies[idx]) {
            continue;
        }
        pmx::PmxRigidBody &rb = model.rigid_bodies[idx];
        btTransform worldTrans;
        rigidBodies[idx]->getMotionState()->getWorldTransform(worldTrans);
        if (rb.target_bone < 0 || rb.target_bone >= bones.size()) {
            continue;
        }
        MMDNode &targetBone = bones[rb.target_bone];
        if (rb.physics_calc_type == 2) {
            btTransform newBoneTransform = targetBone.btTransform() * targetBone.btInitialTransform().inverse() * rigidBodyBindTransforms[idx];
            worldTrans.setOrigin(newBoneTransform.getOrigin());
            rigidBodies[idx]->setCenterOfMassTransform(worldTrans);
            rigidBodies[idx]->getMotionState()->setWorldTransform(worldTrans);
        }
    }
}


#define MAX_BONES 240 // seems to crash if >256

@interface MMDGeometryLoader ()
@end

@implementation MMDGeometryLoader {
    bool disableSkeleton;
    NSMutableArray *images;
    SCNGeometrySource *vertexSource;
    SCNGeometrySource *normalSource;
    SCNGeometrySource *tcoordSource;
    SCNGeometrySource *weightsSource;
    SCNGeometrySource *indicesSource;
    SCNGeometryElement *triangles;
    int *boneVertexCounts;
    uint16_t* boneIndexMapping;
    //uint16_t* boneIndexReverseMapping;
    int num_gfx_bones;
    id<MTLBuffer> metalIndicesData; // we need to retain this ourselves.
    //std::unique_ptr<MMDGeometryState> model;
    MMDGeometryState model;
    //REFLECT?! NSMutableArray<SCNNode*>* rigidBodyDebugNodes;
    //REFLECT?! SCNNode* rigidBodyDebugRoot;

    NSMutableArray *graphicsSkeletonNodesArr;
    NSMutableArray<SCNNode*> *physicsSkeletonNodesArr;

    NSMutableArray<SCNGeometryElement*> *elementsArr;
    NSMutableArray<SCNMaterial*> *materialsArr;

    NSMutableArray<SCNText*> *textArr;
    NSMutableArray<SCNNode*> *textNodeArr;
    NSData *debugLinesData;
    NSData *debugLinesIndicesData;
    SCNGeometrySource *debugLinesVertexSource;
    SCNGeometrySource *debugLinesColorSource;
    SCNGeometryElement *debugLinesElement;
    SCNGeometryElement *debugPointsElement;
    SCNGeometry *debugLinesGeometry;
    SCNNode *debugLinesNode;
}

- (int)boneCount {
    return (int)physicsSkeletonNodesArr.count;
}

- (SCNNode*)boneAtIndex:(int)index {
    if (index < 0 || index >= self.boneCount) {
        return nil;
    }
    return [physicsSkeletonNodesArr objectAtIndex:index];
}

/*
enum BuiltinTypes {
    BUILTIN_NONE = 0,
    BUILTIN_CUBE = 1,
    BUILTIN_CUBE_SKINNED = 2
};*/

- (int) builtinType {
    return 0;
}

- (instancetype)initFromFile:(NSString*)filename {
    if ((self = [super init])) {
        _filename = filename;
        //model.reset(new MMDGeometryState);
        if (!self.builtinType) {
            std::ifstream ifs (filename.UTF8String);
            if (!model.parse(&ifs)) {
                return nil;
            }
        }
    }
    return self;
}

int trimGraphicsBones(pmx::PmxModel &model, int *counts, uint16_t *boneIndexMapping) {
    // limit of 255 hardcoded in shader. Based on Metal limit of 16KB on most iPhones.
    constexpr int MAX_GFX_BONES = 255;
    int num_gfx_bones = 1;
    //boneIndexReverseMapping[0] = 0;
    for (int j = 0; j < model.bone_count; j++) {
        boneIndexMapping[j] = j == 0 ? 0 : boneIndexMapping[j - 1];
        NSLog(@"Bone %d used %d times", j, counts[j]);
        if (j != 0 && counts[j] != 0) {
            boneIndexMapping[j]++;
            //boneIndexReverseMapping[boneIndexMapping[j]] = j;
            num_gfx_bones++;
        }
    }
    if (num_gfx_bones <= MAX_GFX_BONES) {
        return num_gfx_bones;
    }
    int N = 10;
    std::map<std::string, std::vector<int>> boneBaseNameToList;
    for (int j = 0; j < model.bone_count; j++) {
        std::string boneName = model.bones[j].bone_name;
        while (boneName.size() > 3) {
            if (boneName[boneName.size() - 1] >= '0' &&
                    boneName[boneName.size() - 1] <= '9') {
                boneName.resize(boneName.size() - 1);
            } else if (boneName[boneName.size() - 3] == '\xef' &&
                        boneName[boneName.size() - 2] == '\xbc' &&
                        boneName[boneName.size() - 1] >= '\x90' &&
                        boneName[boneName.size() - 1] <= '\x99') {
                boneName.resize(boneName.size() - 3);
            } else {
                break;
            }
        }
        boneBaseNameToList[boneName].push_back(j);
    }
    std::map<int, int> bonesToDelete;
    uint32_t seed = num_gfx_bones;
    while (N > 3 && num_gfx_bones > MAX_GFX_BONES) {
        std::vector<std::vector<int>*> largeBoneLists;
        for (auto &val : boneBaseNameToList) {
            if (val.second.size() >= N) {
                largeBoneLists.push_back(&val.second);
            }
        }
        bool deletedBone = false;
        while (!largeBoneLists.empty()) {
            std::vector<std::pair<int, std::pair<int, int>>> trimmableBones;
            for (int i = 0; i < largeBoneLists.size(); i++) {
                auto &val = *largeBoneLists[i];
                for (int j = 1; j < val.size() - 1; j++) {
                    if (//model.bones[val[j + 1]].parent_index == val[j] &&
                            counts[val[j]] > 0 && counts[val[j + 1]] > 0) {
                        seed += val.size();
                        trimmableBones.push_back(std::make_pair(val[j], std::make_pair(val[j + 1], i)));
                    }
                }
            }
            if (trimmableBones.empty()) {
                break;
            }
            int minCount = counts[trimmableBones[0].first];
            int which = 1;
            for (int j = 1; j < trimmableBones.size(); j++) {
                int thisCount = counts[trimmableBones[j].first];
                if (thisCount < minCount) {
                    seed += thisCount;
                    minCount = thisCount;
                    which = 1;
                } else if (thisCount == minCount) {
                    which++;
                }
            }
            which = seed % which;
            for (int j = 0; j < trimmableBones.size(); j++) {
                if (which == 0) {
                    bonesToDelete[trimmableBones[j].first] = trimmableBones[j].second.first;
                    NSLog(@"Deleted bone at N=%d %d [%s, %dvert] ; switch to %d [%s, %dvert].", N, trimmableBones[j].first, model.bones[trimmableBones[j].first].bone_name.c_str(),
                        counts[trimmableBones[j].first],
                        trimmableBones[j].second.first,
                        model.bones[trimmableBones[j].second.first].bone_name.c_str(),
                        counts[trimmableBones[j].second.first]);
                    counts[trimmableBones[j].second.first] += counts[trimmableBones[j].first];
                    counts[trimmableBones[j].first] = 0;
                    largeBoneLists.erase(largeBoneLists.begin() + trimmableBones[j].second.second);
                    num_gfx_bones--;
                    deletedBone = true;
                    break;
                }
                if (counts[trimmableBones[j].first] == minCount) {
                    which--;
                }
            }
            if (!deletedBone) {
                // assert?
                break; // no progress...
            }
        }
        if (!deletedBone) {
            N--;
        }
    }
    if (num_gfx_bones > MAX_GFX_BONES) {
        NSLog(@"Hard dropping bones from %d to %d", num_gfx_bones, MAX_GFX_BONES);
        std::vector<std::pair<int, int>> sortedBones;
        for (int i = 0; i < model.bone_count; i++) {
            sortedBones.push_back(std::make_pair(counts[i], i));
        }
        std::sort(sortedBones.begin(), sortedBones.end());
        for (int i = 0; num_gfx_bones > MAX_GFX_BONES; i++) {
            int curBone = sortedBones[i].second;
            int parentIdx = model.bones[curBone].parent_index;
            if (parentIdx < 0 || parentIdx >= curBone) {
                parentIdx = 0;
            }
            bonesToDelete[curBone] = parentIdx;
            NSLog(@"Deleted bone at %d [%s, %dvert] ; switch to %d [%s, %dvert].", curBone, model.bones[curBone].bone_name.c_str(),
                counts[curBone],
                parentIdx,
                model.bones[parentIdx].bone_name.c_str(),
                counts[parentIdx]);
            counts[parentIdx] += counts[curBone];
            counts[curBone] = 0;
            num_gfx_bones--;
        }
    }
    num_gfx_bones = 1;
    int lastValidIndexMapping = 0;
    //boneIndexReverseMapping[0] = 0;
    for (int j = 0; j < model.bone_count; j++) {
        boneIndexMapping[j] = j == 0 ? 0 : lastValidIndexMapping;
        NSLog(@"Bone %d used %d times", j, counts[j]);
        auto it = bonesToDelete.find(j);
        if (it == bonesToDelete.end() && j != 0 && counts[j] != 0) {
            boneIndexMapping[j]++;
            //boneIndexReverseMapping[boneIndexMapping[j]] = j;
            lastValidIndexMapping = boneIndexMapping[j];
            num_gfx_bones++;
        }
    }
    for (int j = 0; j < model.bone_count; j++) {
        auto it = bonesToDelete.find(j);
        if (it != bonesToDelete.end()) {
            boneIndexMapping[j] = boneIndexMapping[it->second];
        }
    }
    return num_gfx_bones;
}

- (SCNGeometry*)constructGeometryWithDevice:(id<MTLDevice>)device {
#ifdef MACAPP
    device = nil;
#endif
    NSLog(@"counts: vert=%d ind=%d tc=%d mat=%d bone=%d morph=%d frame=%d rigid=%d joint=%d soft=%d",
        model.vertex_count,
        model.index_count,
        model.texture_count,
        model.material_count,
        model.bone_count,
        model.morph_count,
        model.frame_count,
        model.rigid_body_count,
        model.joint_count,
        model.soft_body_count);

    NSData *data = [NSData dataWithBytes:model.vertices.get() length:sizeof(pmx::PmxVertex) * model.vertex_count];

    vertexSource = [SCNGeometrySource geometrySourceWithData:data
                                                    semantic:SCNGeometrySourceSemanticVertex
                                                 vectorCount:model.vertex_count
                                             floatComponents:YES
                                         componentsPerVector:3 // x, y, z
                                           bytesPerComponent:sizeof(float)
                                                  dataOffset:offsetof(pmx::PmxVertexBase, position)
                                                  dataStride:sizeof(pmx::PmxVertex)];
    
    normalSource = [SCNGeometrySource geometrySourceWithData:data
                                                    semantic:SCNGeometrySourceSemanticNormal
                                                 vectorCount:model.vertex_count
                                             floatComponents:YES
                                         componentsPerVector:3 // nx, ny, nz
                                           bytesPerComponent:sizeof(float)
                                                  dataOffset:offsetof(pmx::PmxVertexBase, normal)
                                                  dataStride:sizeof(pmx::PmxVertex)];
    
    tcoordSource = [SCNGeometrySource geometrySourceWithData:data
                                                    semantic:SCNGeometrySourceSemanticTexcoord
                                                 vectorCount:model.vertex_count
                                             floatComponents:YES
                                         componentsPerVector:2 // s, t
                                           bytesPerComponent:sizeof(float)
                                                  dataOffset:offsetof(pmx::PmxVertexBase, uv)
                                                  dataStride:sizeof(pmx::PmxVertex)];

#define MAX_NUM_WEIGHTS 4

    typedef uint16_t indices_t;

    float *targetWeights = new float[model.vertex_count * MAX_NUM_WEIGHTS];
    indices_t *targetIndices = new indices_t[model.vertex_count * MAX_NUM_WEIGHTS];

    int maxComponents = 1;
    for (int i = 0; i < model.vertex_count; i++) {
        switch(model.vertices[i].skinning_type) {
            case pmx::PmxVertexSkinningType::BDEF2:
            case pmx::PmxVertexSkinningType::SDEF:
                if (maxComponents < 2) {
                    maxComponents = 2;
                }
                break;
            case pmx::PmxVertexSkinningType::BDEF4:
            case pmx::PmxVertexSkinningType::QDEF:
                maxComponents = 4;
                break;
            default:
                break;
        }
    }
    // To optimize models without BDEF4, we would need to change everything to use
    // maxComponents instead of allocating 4.
    maxComponents  = MAX_NUM_WEIGHTS; // [SceneKit] Error: SCNSkinner: bone indices stride must be equal to componentsPerVector * bytesPerComponent

    if (model.bone_count > 0) {
        boneVertexCounts = new int[model.bone_count];
        std::fill(boneVertexCounts, boneVertexCounts + model.bone_count, 0);
        boneVertexCounts[0] = 1; // always keep root node.

        for (int i = 0; i < model.vertex_count; i++) {
            float *wei = &targetWeights[i * MAX_NUM_WEIGHTS];
            indices_t *ind = &targetIndices[i * MAX_NUM_WEIGHTS];
            const pmx::PmxVertex &vx = model.vertices[i];
            std::fill(wei, wei + MAX_NUM_WEIGHTS, 0.0);
            std::fill(ind, ind + MAX_NUM_WEIGHTS, 0);
            switch(vx.skinning_type) {
                case pmx::PmxVertexSkinningType::BDEF1:
                default:
                {
                    const pmx::PmxVertexSkinningBDEF1 &skin = static_cast<pmx::PmxVertexSkinningBDEF1 &>(*vx.skinning);
                    wei[0] = 1.0;
                    ind[0] = skin.bone_index;
                    break;
                }
                case pmx::PmxVertexSkinningType::BDEF2:
                {
                    const pmx::PmxVertexSkinningBDEF2 &skin = static_cast<pmx::PmxVertexSkinningBDEF2 &>(*vx.skinning);
                    wei[0] = skin.bone_weight;
                    wei[1] = 1.0 - skin.bone_weight;
                    ind[0] = skin.bone_index1;
                    ind[1] = skin.bone_index2;
                    break;
                }
                case pmx::PmxVertexSkinningType::QDEF:
                case pmx::PmxVertexSkinningType::BDEF4:
                {
                    // QDEF is same structure layout as BDEF4
                    //const pmx::PmxVertexSkinningQDEF &skin = static_cast<pmx::PmxVertexSkinningQDEF &>(*vx.skinning);
                    const pmx::PmxVertexSkinningBDEF4 &skin = static_cast<pmx::PmxVertexSkinningBDEF4 &>(*vx.skinning);
                    // QDEF not implemented in spec, and parameters can be used the same as BDEF4.
                    wei[0] = skin.bone_weight1;
                    wei[1] = skin.bone_weight2;
                    ind[0] = skin.bone_index1;
                    ind[1] = skin.bone_index2;
                    if (MAX_NUM_WEIGHTS >= 3) {
                        wei[2] = skin.bone_weight3;
                        ind[2] = skin.bone_index3;
                    }
                    if (MAX_NUM_WEIGHTS >= 4) {
                        wei[3] = skin.bone_weight4;
                        ind[3] = skin.bone_index4;
                    }
                    break;
                }
                case pmx::PmxVertexSkinningType::SDEF:
                {
                    const pmx::PmxVertexSkinningSDEF &skin = static_cast<pmx::PmxVertexSkinningSDEF &>(*vx.skinning);
                    wei[0] = skin.bone_weight;
                    wei[1] = 1.0 - skin.bone_weight;
                    ind[0] = skin.bone_index1;
                    ind[1] = skin.bone_index2;
                    // TODO: emulate spherical bones
                    // Spherical bones are to handle joints that go from this: -- to this: /_
                    // we want to create two opposite bones like ><. In theory we should be able to
                    // approximate this by mapping the spherical coefficients to negative weights on
                    // the same two bone indices, as they describe basis vectors along axis of rotation,
                    break;
                }
            }
            float totalWeight = 0.0;
            int lastInd = 0;
            for (int j = 0; j < MAX_NUM_WEIGHTS; j++) {
                if (ind[j] >= model.bone_count || ind[j] < 0) {
                    NSLog(@"Bone @%d index[%d] invalid", i, j);
                    ind[j] = 0;
                }
                if (wei[j] > 0) {
                    boneVertexCounts[(uint16_t)ind[j]]++;
                }
                lastInd = ind[j];
                totalWeight += wei[j];
            }
            if (!(totalWeight <= 1.01 && totalWeight >= 0.99)) {
                NSLog(@"Bone @%d Weights do not add up %f %f %f %f", i, wei[0], wei[1], wei[2], wei[3]);
            }
        }
        boneIndexMapping = new uint16_t[model.bone_count];
        //boneIndexReverseMapping = new uint16_t[model.bone_count];
        num_gfx_bones = trimGraphicsBones(model, boneVertexCounts, boneIndexMapping);
        for (int j = 0; j < MAX_NUM_WEIGHTS * model.vertex_count; j++) {
            targetIndices[j] = boneIndexMapping[(uint16_t)targetIndices[j]];
        }
        NSData *weightData = [NSData dataWithBytes:targetWeights length:sizeof(*targetWeights) * MAX_NUM_WEIGHTS * model.vertex_count];
        weightsSource = [SCNGeometrySource geometrySourceWithData:weightData
                                                        semantic:SCNGeometrySourceSemanticBoneWeights
                                                     vectorCount:model.vertex_count
                                                 floatComponents:YES
                                             componentsPerVector:maxComponents // Theoretically we can choose 2 here if
                                               bytesPerComponent:sizeof(*targetWeights)
                                                      dataOffset:0
                                                      dataStride:sizeof(*targetWeights) * MAX_NUM_WEIGHTS];
        #if !TARGET_OS_SIMULATOR
        // Stupid API bug in SceneKit. Ugh.
        // BoneIndices must be 8 or 16 bit unsigned integer.
        // SceneKit has an API bug that only allows specifying floats or signed integers.
        // Metal is very strict aboutn signedness matching, and floats are not allowed, so no way
        // except using the MTLBuffer APIs directly, which is ugly in so many ways >_<
        if (device) {
            static_assert(std::is_same<indices_t, uint16_t>::value, "indices must be uint16_t");
            metalIndicesData = [device newBufferWithBytes:targetIndices length:sizeof(*targetIndices) * MAX_NUM_WEIGHTS * model.vertex_count options:MTLResourceCPUCacheModeDefaultCache];
            MTLVertexFormat vformat = MAX_NUM_WEIGHTS == 2 ? MTLVertexFormatUShort2 : (
                    MAX_NUM_WEIGHTS == 3 ? MTLVertexFormatUShort3 : MTLVertexFormatUShort4);
            indicesSource = [SCNGeometrySource geometrySourceWithBuffer:metalIndicesData vertexFormat:vformat semantic:SCNGeometrySourceSemanticBoneIndices vertexCount:model.vertex_count dataOffset:0 dataStride:sizeof(*targetIndices) * MAX_NUM_WEIGHTS];
        }
        else
        #endif
        // Luckily non-Metal systems are ok with signed int8 or int16, so we can use the NSData api here.
        {
            NSData *indexData = [NSData dataWithBytes:targetIndices length:sizeof(*targetIndices) * MAX_NUM_WEIGHTS * model.vertex_count];
            indicesSource = [SCNGeometrySource geometrySourceWithData:indexData
                                                        semantic:SCNGeometrySourceSemanticBoneIndices
                                                     vectorCount:model.vertex_count
                                                 floatComponents:NO
                             
                                             componentsPerVector:maxComponents // Theoretically we can choose 2 here if
                                               bytesPerComponent:sizeof(*targetIndices)
                                                      dataOffset:0
                                                      dataStride:sizeof(*targetIndices) * MAX_NUM_WEIGHTS];
        }
    }

    /*{
        float *targetIndexColors = new float[4 * model.vertex_count];
        memset(targetIndexColors, 0, sizeof(float)*4 * model.vertex_count);
        for (int j = 0; j < model.vertex_count; j++) {
            targetIndexColors[j * 4] = targetIndices[j * MAX_NUM_WEIGHTS] / 240.;
            targetIndexColors[j * 4 + 1] = targetIndices[j * MAX_NUM_WEIGHTS] / 240.;
            targetIndexColors[j * 4 + 2] = targetIndices[j * MAX_NUM_WEIGHTS] / 240.;
            targetIndexColors[j * 4 + 3] = targetIndices[j * MAX_NUM_WEIGHTS] / 240.;
            //targetIndexColors[j * 4 + 1] = targetIndices[j * MAX_NUM_WEIGHTS + 1] / float(MAX_BONES);
            //targetIndexColors[j * 4 + 2] = targetIndices[j * MAX_NUM_WEIGHTS + 2] / float(MAX_BONES);
            //targetIndexColors[j * 4 + 3] = 1;
        }
        NSData *colorData = [NSData dataWithBytes:targetIndexColors length:sizeof(*targetIndexColors) * MAX_NUM_WEIGHTS * model.vertex_count];
        tcoordSource = [SCNGeometrySource geometrySourceWithData:colorData
                                                    semantic:SCNGeometrySourceSemanticColor
                                                 vectorCount:model.vertex_count
                                             floatComponents:YES
                                         componentsPerVector:4 // Theoretically we can choose 2 here if
                                           bytesPerComponent:sizeof(*targetIndices)
                                                  dataOffset:0
                                                  dataStride:sizeof(*targetIndices) * 4];
    }*/

    /*
    geom.firstMaterial.diffuse.contents = [UIImage imageNamed:@"cham_models/服X.png"];
    geom.firstMaterial.diffuse.mipFilter = MTLSamplerMipFilterLinear;
    */
    
    images = [NSMutableArray arrayWithCapacity:model.texture_count];
    for (int i = 0; i < model.texture_count; i++) {
        std::string texStr = model.textures[i];
        std::replace(texStr.begin(), texStr.end(), '\\', '/');
        if (texStr.find("..") != -1) { // avoid going out of root dir.
            texStr.erase(std::remove(texStr.begin(), texStr.end(), '/'), texStr.end());
        }
        // Clean all slashes to avoid security risk.
        NSString *path = [[_filename stringByDeletingLastPathComponent] stringByAppendingPathComponent:
                                               [NSString stringWithUTF8String:texStr.c_str()]];
        NSLog(@"Loading texture %@", path);
#ifdef MACAPP
        NSImage *img = [[NSImage alloc] initWithContentsOfFile:path];
#else
        UIImage *img = [UIImage imageNamed:path];
#endif
        if (img == nil) {
            img = [UIImage imageNamed:@"floor.jpg"];
        }
        [images addObject:img];
    }
    /*
    UIImage *toons = [NSMutableArray arrayWithCapacity:11];
    [toons addObject:nil];
    for (int i = 1; i <= 10; i++) {
        [toons addObject:[UIImage imageNamed:[NSString stringWithFormat:@"cham_models/toon%02d.bmp", i]]];
    }
     */
    
    const auto& createUIColor = [](const float arr[]) {
        return [UIColor colorWithRed:arr[0] green:arr[1] blue:arr[2] alpha:1.0];
    };
    
    elementsArr = [NSMutableArray arrayWithCapacity:model.material_count];
    materialsArr = [NSMutableArray arrayWithCapacity:model.material_count];

    int elementOffset = 0;
    for (int i = 0; i < model.material_count && elementOffset < model.index_count; i++) {
        const pmx::PmxMaterial &pmxmat = model.materials[i];
        SCNMaterial *scnmat = [[SCNMaterial alloc] init];
        scnmat.name = [NSString stringWithUTF8String:pmxmat.material_name.c_str()];
        if (pmxmat.diffuse_texture_index >= 0 && pmxmat.diffuse_texture_index < model.texture_count) {
            UIImage *teximg = [images objectAtIndex:pmxmat.diffuse_texture_index];
            scnmat.diffuse.contents = teximg;
            scnmat.transparent.contents = teximg;
        } else {
            scnmat.diffuse.contents = createUIColor(pmxmat.diffuse);
        }
        scnmat.transparency = pmxmat.diffuse[3];
        scnmat.specular.contents = createUIColor(pmxmat.specular);
        scnmat.shininess = pmxmat.specularlity;
        scnmat.ambient.contents = createUIColor(pmxmat.ambient);
        if (pmxmat.sphere_texture_index >= 0 && pmxmat.sphere_texture_index < model.texture_count &&
                pmxmat.sphere_op_mode != 0) {
            scnmat.reflective.contents = [images objectAtIndex:pmxmat.sphere_texture_index];
            // if pmxmat.sphere_op_mode == 3, we need to provide a separate set of UV coords...
            // if pmxmat.sphere_op_mode == 2, it is additive while 1 is multiply
        }
        if (pmxmat.flag & (1 << 0)) {
            // Disable back face culling
            scnmat.doubleSided = YES;
        }
        // flag bits 1,2,3 relate to shadow
        /*
        if (pmxmat.common_toon_flag == 1 && pmxmat.toon_texture_index >= 1 && pmxmat.toon_texture_index <= 10) {
            UIImage *toon = [toons objectAtIndex:pmxmat.toon_texture_index];
        } else if (pmxmat.toon_texture_index >= 0 && pmxmat.toon_texture_index < model.texture_count  && pmxmat.common_toon_flag == 0) {
            UIImage *toon = [images objectAtIndex:pmxmat.toon_texture_index];
        }
        */
        // flag bit 4 relates to edge drawing
        // edge_color / edge_size

        // flag bit 5: Vertex colour as additional vec4.
        // (add SCNGeometrySource with semantic SCNGeometrySourceSemanticTexcoord)

        SCNGeometryPrimitiveType primType = SCNGeometryPrimitiveTypeTriangles;
        if (pmxmat.flag & (1 << 6)) {
            primType = SCNGeometryPrimitiveTypePoint;
        } else if (pmxmat.flag & (1 << 7)) {
            primType = SCNGeometryPrimitiveTypeLine;
        }
        int index_count = pmxmat.index_count;
        if (index_count > model.index_count || index_count + elementOffset > model.index_count) {
            index_count = model.index_count - elementOffset;
        }
        data = [NSData dataWithBytes:&(model.indices[elementOffset]) length:sizeof(int) * index_count];
        elementOffset += index_count;

        [elementsArr addObject:[SCNGeometryElement geometryElementWithData:data primitiveType:primType primitiveCount:index_count / 3 bytesPerIndex:sizeof(int)]];
        [materialsArr addObject:scnmat];
    }

    SCNGeometry *geom = [SCNGeometry geometryWithSources:@[vertexSource, normalSource, tcoordSource, weightsSource, indicesSource] elements:elementsArr];
    geom.materials = materialsArr;
    return geom;
}

- (SCNMorpher *)constructMorpher {
    //return nil;
    if (self.builtinType) {
        return nil;
    }
    int vert_morph_count = 0;
    for (int i = 0; i < model.morph_count; i++) {
        if (model.morphs[i].morph_type == pmx::MorphType::Vertex) {
            vert_morph_count++;
        }
    }
    if (vert_morph_count == 0) {
        return nil; // Creating an empty SCNMorpher causes glitchy meshes.
    }
    SCNMorpher *morph = [[SCNMorpher alloc] init];
    NSMutableArray *morphTargets = [NSMutableArray arrayWithCapacity:model.morph_count];
    morph.calculationMode = SCNMorpherCalculationModeAdditive;
    for (int i = 0; i < model.morph_count; i++) {
        const pmx::PmxMorph &pmxmorph = model.morphs[i];
        switch (pmxmorph.morph_type) {
            case pmx::MorphType::Vertex:
            {
                float *targetVertices = new float[model.vertex_count * 3];
                std::fill(targetVertices, targetVertices + model.vertex_count * 3, 0);
                for (int j = 0; j < pmxmorph.offset_count; j++) {
                    const pmx::PmxMorphVertexOffset &morphv = pmxmorph.vertex_offsets[j];
                    targetVertices[morphv.vertex_index * 3] = morphv.position_offset[0];
                    targetVertices[morphv.vertex_index * 3 + 1] = morphv.position_offset[1];
                    targetVertices[morphv.vertex_index * 3 + 2] = morphv.position_offset[2];
                }
                NSData *data = [NSData dataWithBytes:targetVertices length:sizeof(float) * 3 * model.vertex_count];
                delete []targetVertices;
                vertexSource = [SCNGeometrySource geometrySourceWithData:data
                                                                semantic:SCNGeometrySourceSemanticVertex
                                                             vectorCount:model.vertex_count
                                                         floatComponents:YES
                                                     componentsPerVector:3 // x, y, z
                                                       bytesPerComponent:sizeof(float)
                                                              dataOffset:0
                                                              dataStride:sizeof(float) * 3];
                SCNGeometry *geom = [SCNGeometry geometryWithSources:@[vertexSource] elements:elementsArr];
                geom.name = [NSString stringWithUTF8String:pmxmorph.morph_name.c_str()];
                NSLog(@"Morph %d name %@", (int)[morphTargets count], geom.name);
                [morphTargets addObject:geom];
            }
            break;
        }
    }
    morph.targets = morphTargets;
    return morph;
}

- (void)animateMorphNode:(SCNNode*)node name:(NSString*)name repeatCount:(float)repeatCount {
    for (int i = 0; i < node.morpher.targets.count; i++) {
        if ([node.morpher.targets[i].name isEqualToString:name]) {
            CABasicAnimation *animation = [CABasicAnimation animationWithKeyPath:[NSString stringWithFormat:@"morpher.weights[%d]", i]];
            animation.fromValue = @0.0;
            animation.toValue = @1.0;
            animation.autoreverses = YES;
            animation.repeatCount = repeatCount;
            animation.duration = 1;
            [node addAnimation:animation forKey:nil];
        }
    }
}

- (SCNSkinner*)constructSkinnerFromGeometry:(SCNGeometry*)geom attachedToNode:(SCNNode*)mmdNode {

    NSMutableArray<NSValue*>* boneInverseBindTransforms = [[NSMutableArray alloc] initWithCapacity:model.bone_count];

    physicsSkeletonNodesArr = [[NSMutableArray alloc] initWithCapacity:model.bone_count];
    graphicsSkeletonNodesArr = [[NSMutableArray alloc] initWithCapacity:num_gfx_bones];
    for (int i = 0; i < num_gfx_bones; i++) {
        [graphicsSkeletonNodesArr addObject:[NSNull null]];
    }
    SCNNode *lastRootNode = nil;
    //NSDictionary<NSString*, SCNNode*> bonesByName;
    //
    for (int i = 0; i < model.bone_count; i++) {
        pmx::PmxBone &pmxbone = model.bones[i].bone;
        SCNNode *bone = [[SCNNode alloc] init];
        bone.name = [NSString stringWithUTF8String:pmxbone.bone_name.c_str()];
        bone.position = SCNVector3Make(pmxbone.position[0], pmxbone.position[1], pmxbone.position[2]);
        /*
        simd_float3 orientx;
        orientx.x = pmxbone.local_axis_x_orientation[0];
        orientx.y = pmxbone.local_axis_x_orientation[1];
        orientx.z = pmxbone.local_axis_x_orientation[2];
        if (orientx.x == 0 && orientx.y == 0 && orientx.z == 0) {
            orientx = simd_make_float3(1, 0, 0);
        }
        simd_float3 orientz;
        orientz.x = pmxbone.local_axis_y_orientation[0];
        orientz.y = pmxbone.local_axis_y_orientation[1];
        orientz.z = pmxbone.local_axis_y_orientation[2];
        if (orientz.x == 0 && orientz.y == 0 && orientz.z == 0) {
            orientz = simd_make_float3(0, 0, 1);
        }
        simd_float3 orienty = simd_cross(orientx, orientx);
        simd_float3x3 mat3 = {orientx,orienty,orientz};
        simd_quatf quat = simd_normalize(simd_quaternion(mat3));
        NSLog(@"bone %@ orientx<%.3f,%.3f,%.3f> orienty<%.3f,%.3f,%.3f> orientZ<%.3f,%.3f,%.3f> quat<%f,%f,%f,%f>", bone.name, orientx.x, orientx.y, orientx.z, orienty.x, orienty.y, orienty.z, orientz.x, orientz.y, orientz.z, quat.vector.x, quat.vector.y, quat.vector.z, quat.vector.w);
        bone.orientation = SCNVector4Make(quat.vector.x, quat.vector.y, quat.vector.z, quat.vector.w);
        */
        [physicsSkeletonNodesArr addObject:bone];
        if (i == 0) {
            _rootNode = bone;
            lastRootNode = bone;
            [graphicsSkeletonNodesArr setObject:bone atIndexedSubscript:0]; //root bone can be shared?
        } else if (boneVertexCounts[i] > 0) {
            [graphicsSkeletonNodesArr setObject:bone atIndexedSubscript:boneIndexMapping[i]];
        }
        if (i != 0) {
            // bone 0 is the root and we would set parentBone = _rootNode in this case.
            if (pmxbone.parent_index >= i) {
                NSLog(@"parent indices must be ordered %d %d", pmxbone.parent_index, i);
                return nil;
            }
            //SCNVector3 pos = bone.position;
            SCNMatrix4 trans = bone.transform;
            SCNNode *parentBone;
            if (pmxbone.parent_index < 0) {
                // HACK: 操作中心 is often before 全ての親. See sm14956092
                // 操作中心 is only to aid motion editing and is unimportant
                // If this happens, _rootNode would point to a useless bone.
                parentBone = lastRootNode;
                lastRootNode = bone;
            } else {
                parentBone = [physicsSkeletonNodesArr objectAtIndex:pmxbone.parent_index];
            }
            [parentBone addChildNode:bone];
            bone.transform = [parentBone convertTransform:trans fromNode:_rootNode]; // FIXME: assumes rootNode at 0,0,0
        }
        if (bone.parentNode) {
            NSLog(@"Bone %d name %@ parent %@", i, bone.name, bone.parentNode.name);
        } else {
            NSLog(@"Bone %d name %@ rooted", i, bone.name);
        }
//#define SHOW_DEBUG_BONES
#ifdef SHOW_DEBUG_BONES
        SCNSphere *sph = [SCNSphere sphereWithRadius:0.1];
        SCNMaterial *material = [[SCNMaterial alloc] init];
        material.diffuse.contents = [UIColor colorWithRed:(i==0) green:(i%32)/32. blue:(i/32)/32. alpha:1];
        sph.materials = @[material];
        SCNNode *sphnode = [SCNNode nodeWithGeometry:sph];
        [bone addChildNode:sphnode];
#endif
    }
    for (int i =0; i < graphicsSkeletonNodesArr.count; i++) {
        SCNNode *gfx_node = [graphicsSkeletonNodesArr objectAtIndex:i];
        if (gfx_node.parentNode) {
            NSLog(@"GFX Bone %d name %@ parent %@", i, gfx_node.name, gfx_node.parentNode.name);
        } else {
            NSLog(@"GFX Bone %d name %@ rooted", i, gfx_node.name);
        }
        [boneInverseBindTransforms addObject:[NSValue valueWithSCNMatrix4:SCNMatrix4Invert(gfx_node.worldTransform)]];
    }
    model.setupIK();
    //[self tick:0];

    [mmdNode addChildNode:_rootNode];
    /*
    rigidBodyDebugRoot = [[SCNNode alloc] init];
    rigidBodyDebugRoot.opacity = 0.4;
    */
    //[mmdNode addChildNode:rigidBodyDebugRoot];

    model.setupPhysics();
    /*
    SCNNode *頭 = [_rootNode childNodeWithName:@"頭" recursively:YES];
    CABasicAnimation *anim = [CABasicAnimation animationWithKeyPath:@"rotation"];
    anim.fromValue = @0.0;
    anim.toValue = @(2 * M_PI);
    anim.duration = 3;
    anim.repeatCount = INFINITY;
    [頭 addAnimation:anim forKey:nil];
    */
    SCNSkinner *skinner = nil;
    if (geom != nil) {
        skinner = [SCNSkinner skinnerWithBaseGeometry:geom bones:graphicsSkeletonNodesArr boneInverseBindTransforms:boneInverseBindTransforms boneWeights:weightsSource boneIndices:indicesSource];
        skinner.skeleton = _rootNode;
    }
    /*
    CABasicAnimation *anim2 = [CABasicAnimation animationWithKeyPath:@"position"];
    anim2.fromValue = [NSValue valueWithSCNVector3:SCNVector3Make(0,0,0)];
    anim2.toValue = [NSValue valueWithSCNVector3:SCNVector3Make(3,1,5)];
    //anim.toValue = @(2 * M_PI);
    anim2.duration = 4;
    anim2.repeatCount = INFINITY;
    //[_rootNode addAnimation:anim2 forKey:nil];
    if (physicsSkeletonNodesArr.count > 3) {
        [[physicsSkeletonNodesArr objectAtIndex:3] addAnimation:anim2 forKey:nil];
    }
    CABasicAnimation *anim3 = [CABasicAnimation animationWithKeyPath:@"position"];
    anim3.fromValue = [NSValue valueWithSCNVector3:SCNVector3Make(0,0,0)];
    anim3.toValue = [NSValue valueWithSCNVector3:SCNVector3Make(2,1,0)];
    anim3.duration = 4;
    anim3.repeatCount = INFINITY;
    */
    /*
    CABasicAnimation *anim3 = [CABasicAnimation animationWithKeyPath:@"rotation"];
    anim3.fromValue = @0.0;
    anim3.toValue = @(2 * M_PI);
    anim3.duration = 3;
    anim3.repeatCount = INFINITY;
    if (physicsSkeletonNodesArr.count > 20) {
        [[physicsSkeletonNodesArr objectAtIndex:20] addAnimation:anim3 forKey:nil];
    }
    */
    return disableSkeleton ? nil : skinner;
}

inline btTransform scnToBtTransform(const SCNMatrix4 &transform) {
    return btTransform(btMatrix3x3(
            transform.m11, transform.m12, transform.m13,
            transform.m21, transform.m22, transform.m23,
            transform.m31, transform.m32, transform.m33),
            btVector3(transform.m41, transform.m42, transform.m43));
}
/*
inline SCNMatrix4 btTransformToScn(const btTransform&trans) {
    const btVector3 &xval = trans.getBasis()[0];
    const btVector3 &yval = trans.getBasis()[1];
    const btVector3 &zval = trans.getBasis()[2];
    const btVector3 &posval = trans.getOrigin();
    return {xval.x(), xval.y(), xval.z(), 0,
            yval.x(), yval.y(), yval.z(), 0,
            zval.x(), zval.y(), zval.z(), 0,
            posval.x(), posval.y(), posval.z(), 1};
}

*/

/*
inline btTransform scnToBtTransform(const SCNMatrix4 &transform) {
    return btTransform(btMatrix3x3(
            transform.m11, transform.m21, transform.m31,
            transform.m12, transform.m22, transform.m32,
            transform.m13, transform.m23, transform.m33),
            btVector3(transform.m41, transform.m42, transform.m43));
}*/

inline SCNMatrix4 posEulerToScn(float (&pos)[3], float (&orient)[3]) {
    return SCNMatrix4Translate(
        SCNMatrix4Rotate(
            SCNMatrix4Rotate(
                SCNMatrix4Rotate(
                    SCNMatrix4Identity,
                    orient[1], 0, 1, 0),
                orient[0], 1, 0, 0),
            orient[2], 0, 0, 1),
        pos[0], pos[1], pos[2]);
/*
    return SCNMatrix4Rotate(
            SCNMatrix4Rotate(
                SCNMatrix4Rotate(
                    SCNMatrix4MakeTranslation(pos[0], pos[1], pos[2]),
                    orient[1], 0, 1, 0),
                orient[0], 1, 0, 0),
            orient[2], 0, 0, 1);
            */
    //return SCNMatrix4MakeTranslation(pos[0], pos[1], pos[2]);
}

inline SCNMatrix4 btTransformToScn(const btTransform&trans) {
    const btVector3 &xval = trans.getBasis()[0];
    const btVector3 &yval = trans.getBasis()[1];
    const btVector3 &zval = trans.getBasis()[2];
    const btVector3 &posval = trans.getOrigin();
    return {xval.x(), yval.x(), zval.x(), 0,
            xval.y(), yval.y(), zval.y(), 0,
            xval.z(), yval.z(), zval.z(), 0,
            posval.x(), posval.y(), posval.z(), 1};
}

inline btVector3 arrToVec3(float (&pos)[3]) {
    return btVector3(pos[0], pos[1], pos[2]);
}

- (BOOL)showRigidBodies {
    return !!(model.m_debugDrawer.getDebugMode() & btIDebugDraw::DBG_DrawWireframe);
}

- (void)setShowRigidBodies:(BOOL)val {
    model.m_debugDrawer.changeFlag(btIDebugDraw::DBG_DrawWireframe, !!val);
}

- (BOOL)showConstraints {
    return !!(model.m_debugDrawer.getDebugMode() & btIDebugDraw::DBG_DrawConstraints);
}

- (void)setShowConstraints:(BOOL)val {
    model.m_debugDrawer.changeFlag(btIDebugDraw::DBG_DrawConstraints, !!val);
}

- (BOOL)showConstraintLimits {
    return !!(model.m_debugDrawer.getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits);
}

- (void)setShowConstraintLimits:(BOOL)val {
    model.m_debugDrawer.changeFlag(btIDebugDraw::DBG_DrawConstraintLimits, !!val);
}

- (BOOL)showText {
    return !!(model.m_debugDrawer.getDebugMode() & (btIDebugDraw::DBG_DrawText | btIDebugDraw::DBG_DrawFeaturesText | btIDebugDraw::DBG_DrawFrames));
}

- (void)setShowText:(BOOL)val {
    model.m_debugDrawer.changeFlag(btIDebugDraw::DBG_DrawText | btIDebugDraw::DBG_DrawFeaturesText | btIDebugDraw::DBG_DrawFrames, !!val);
}

- (BOOL)debugCollision {
    return !!(model.m_debugDrawer.getDebugMode() & (btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawContactPoints | btIDebugDraw::DBG_EnableCCD | btIDebugDraw::DBG_DrawNormals));
}

- (void)setDebugCollision:(BOOL)val {
    model.m_debugDrawer.changeFlag(
        btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawContactPoints | btIDebugDraw::DBG_EnableCCD | btIDebugDraw::DBG_DrawNormals, !!val);
}

- (BOOL)otherDebug {
    return !!(model.m_debugDrawer.getDebugMode() & (btIDebugDraw::DBG_NoDeactivation | btIDebugDraw::DBG_ProfileTimings | btIDebugDraw::DBG_EnableSatComparison | btIDebugDraw::DBG_DisableBulletLCP | btIDebugDraw::DBG_FastWireframe));
}

- (void)setOtherDebug:(BOOL)val {
    model.m_debugDrawer.changeFlag(btIDebugDraw::DBG_NoDeactivation | btIDebugDraw::DBG_ProfileTimings | btIDebugDraw::DBG_EnableSatComparison | btIDebugDraw::DBG_DisableBulletLCP | btIDebugDraw::DBG_FastWireframe, !!val);;
}

- (void)tick:(NSTimeInterval)time {
    for (int i = 0, boneCnt = self.boneCount; i < boneCnt; i++) {
        SCNNode *node = [self boneAtIndex:i];
        model.bones[i].simdPosition = node.simdPosition;
        model.bones[i].simdOrientation = node.simdOrientation;
        model.bones[i].updateWorld();
    }
    model.tick((double)time);
    for (int i = 0, boneCnt = self.boneCount; i < boneCnt; i++) {
        SCNNode *node = [self boneAtIndex:i];
        if (simd_any(model.bones[i].simdPosition != node.simdPosition)) {
            node.simdPosition = model.bones[i].simdPosition;
        }
        if (simd_any(model.bones[i].simdOrientation.vector != node.simdOrientation.vector)) {
            node.simdOrientation = model.bones[i].simdOrientation;
        }
    }
    if (1 || model.m_debugDrawer.getDebugMode()) {
        debugLinesData = [NSData dataWithBytes:&(model.m_debugDrawer.debugWire[0]) length:model.m_debugDrawer.debugWire.size() * sizeof(float)];
        debugLinesIndicesData = [NSData dataWithBytes:&(model.m_debugDrawer.wireIndices[0]) length:model.m_debugDrawer.wireIndices.size() * sizeof(int)];
        debugLinesVertexSource = [SCNGeometrySource geometrySourceWithData:debugLinesData
                                                        semantic:SCNGeometrySourceSemanticVertex
                                                     vectorCount:model.m_debugDrawer.debugWire.size() / 7
                                                 floatComponents:YES
                                             componentsPerVector:3 // x, y, z
                                               bytesPerComponent:sizeof(float)
                                                      dataOffset:0
                                                      dataStride:sizeof(float) * 7];
        debugLinesColorSource = [SCNGeometrySource geometrySourceWithData:debugLinesData
                                                        semantic:SCNGeometrySourceSemanticColor
                                                     vectorCount:model.m_debugDrawer.debugWire.size() / 7
                                                 floatComponents:YES
                                             componentsPerVector:4 // r, g, b, a
                                               bytesPerComponent:sizeof(float)
                                                      dataOffset:sizeof(float) * 3
                                                      dataStride:sizeof(float) * 7];
        debugLinesElement = [SCNGeometryElement geometryElementWithData:debugLinesIndicesData primitiveType:SCNGeometryPrimitiveTypeLine primitiveCount:model.m_debugDrawer.wireIndices.size()/2 bytesPerIndex:4];
        debugPointsElement = [SCNGeometryElement geometryElementWithData:debugLinesIndicesData primitiveType:SCNGeometryPrimitiveTypePoint primitiveCount:model.m_debugDrawer.wireIndices.size() bytesPerIndex:4];
        debugPointsElement.pointSize = 5;
        debugLinesGeometry = [SCNGeometry geometryWithSources:@[debugLinesVertexSource, debugLinesColorSource] elements:@[debugPointsElement, debugLinesElement]];

        if (!debugLinesNode) {
            debugLinesNode = [SCNNode node];
            [_rootNode.parentNode addChildNode:debugLinesNode];
        }
        debugLinesNode.geometry = debugLinesGeometry;

        NSMutableArray<SCNNode*>* newNodeArr = [[NSMutableArray alloc] initWithCapacity:textNodeArr.count];
        int nodeIdx = 0;
        NSMutableArray<SCNText*>* newArr = [[NSMutableArray alloc] initWithCapacity:textArr.count];
        for (int i = 0; i < textArr.count; i++) {
            auto it = model.m_debugDrawer.text.find(((NSString*)(textArr[i].string)).UTF8String);
            if (it != model.m_debugDrawer.text.end()) {
                [newArr addObject:textArr[i]];
                SCNNode *node;
                if (nodeIdx < textNodeArr.count) {
                    node = textNodeArr[nodeIdx];
                    nodeIdx++;
                } else {
                    node = [[SCNNode alloc] init];
                    [debugLinesNode addChildNode:node];
                }
                node.geometry = textArr[i];
                node.position = SCNVector3Make(it->second.x(), it->second.y(), it->second.z());
                [newNodeArr addObject:node];
                model.m_debugDrawer.text.erase(it);
            }
        }
        for (auto it : model.m_debugDrawer.text) {
            SCNText *text = [SCNText textWithString:[NSString stringWithUTF8String:it.first.c_str()] extrusionDepth:0.1];
            [newArr addObject:text];
            SCNNode *node;
            if (nodeIdx < textNodeArr.count) {
                node = textNodeArr[nodeIdx];
                nodeIdx++;
            } else {
                node = [[SCNNode alloc] init];
                [debugLinesNode addChildNode:node];
            }
            node.geometry = text;
            node.position = SCNVector3Make(it.second.x(), it.second.y(), it.second.z());
            [newNodeArr addObject:node];
        }
        for (; nodeIdx < textNodeArr.count; nodeIdx++) {
            [textNodeArr[nodeIdx] removeFromParentNode];
        }
        textNodeArr = newNodeArr;
        textArr = newArr;
    } else {
        [debugLinesNode removeFromParentNode];
    }
//    simd_float4x4 rootTransformMatrix = self.rootNode.presentationNode.simdWorldTransform;
//    simd_float4x4 rootTransformInv = simd_inverse(rootTransformMatrix);
//    simd_quatf rootQuatInv = simd_inverse(self.rootNode.simdWorldOrientation);
}

// TODO: Tick function callled from SCNSceneRendererDelegate.renderer(_:willRenderScene:atTime:)
// in order to

@end

