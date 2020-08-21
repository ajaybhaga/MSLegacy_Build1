//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
#pragma once

#include "ClientObj.h"

namespace Urho3D
{
class Controls;
class Node;
class RigidBody;
}

using namespace Urho3D;
//=============================================================================
//=============================================================================
static const unsigned NTWK_CTRL_FORWARD = (1<<0);
static const unsigned NTWK_CTRL_BACK    = (1<<1);
static const unsigned NTWK_CTRL_LEFT    = (1<<2);
static const unsigned NTWK_CTRL_RIGHT   = (1<<3);
static const unsigned NTWK_SWAP_MAT     = (1<<4);

static const unsigned NETWORKACTOR_COL_LAYER = 2;
static const int MAX_MAT_COUNT = 9;

//=============================================================================
//=============================================================================
class NetworkActor : public ClientObj
{
    URHO3D_OBJECT(NetworkActor, ClientObj)

public:
    NetworkActor(Context* context);
    ~NetworkActor();
   
    static void RegisterObject(Context* context);

    virtual void ApplyAttributes();
    virtual void DelayedStart();
    virtual void Create();

    void DebugDraw(const Color &color);
    void ComputeSteerForce();

    /// Parameter function
    int GetLife() { return life_; }
    void SetLife(int m_life) { life_ = m_life; }

    void SetWaypoints(const Vector<Vector3>* waypoints) {

        if (waypoints_) {
            delete waypoints_;
            waypoints_ = nullptr;
        }

        Vector<Vector3>* newWPs = new Vector<Vector3>();
        // Copy waypoints into local list
        for (int i = 0; i < waypoints->Size(); i++) {
            newWPs->Push(Vector3(waypoints->At(i)));
        }

        waypoints_ = newWPs;
    }

    void SetTarget(Vector3 toTarget) {
        toTarget_ = toTarget;
    }

    int getAgentIndex() const {
        return agentIndex;
    }

    float GetLastFire() { return lastFire_; }
    void SetLastFire(float lastFire) { lastFire_ = lastFire; };

    float GetMass() { return mass_; }
    void SetMass(float m_mass) { mass_ = m_mass; }

    Controls GetControls() { return controls_; }
    void SetControls(Controls m_controls) { controls_ = m_controls; }

    float GetSpeed() { return speed_; }
    void SetSpeed(float m_speed) { speed_ = m_speed; }

    float GetMaxSpeed() { return maxSpeed_; }
    void SetMaxSpeed(float m_maxSpeed) { maxSpeed_ = m_maxSpeed; }

    float GetDamping() { return damping_; }
    void SetDamping(float m_damping) { damping_ = m_damping; }
    void Damping() { speed_ -= damping_; if (speed_ <= 0) speed_ = 0; }

    float GetAcceleration() { return acceleration_; }
    void SetAcceleration(float m_acceleration) { acceleration_ = m_acceleration; }
    void Accelerate() { speed_ += acceleration_; if (speed_ > maxSpeed_) speed_ = maxSpeed_; }

    float GetBrake() { return brake_; }
    void SetBrake(float m_brake) { brake_ = m_brake; }
    void Brake() { speed_ -= brake_; if (speed_ < 0) speed_ = 0; }

    Vector3 GetTowards() { return towards_; }
    void SetTowards(Vector3 m_towards) { towards_ = m_towards; }

    float GetTurningVelocity() { return turningVelocity_; }
    void SetTurningVelocity(float m_turningVelocity) { turningVelocity_ = m_turningVelocity; }

    String GetBulletType() { return bulletType_; }
    void SetBulletType(String m_bulletType) { bulletType_ = m_bulletType; }

    SharedPtr<Vehicle> GetVehicle() { return vehicle_; }

    /// Fight
    void Fire(Vector3 target);
    void Fire();

    // Player node collision
    void HandlePlayerCollision(StringHash eventType, VariantMap &eventData);



protected:
    void SwapMat();
    virtual void FixedUpdate(float timeStep);
   
public:
    WeakPtr<RigidBody> pRigidBody_;
    WeakPtr<Node> nodeInfo_;
    Controls prevControls_;
    ////
    /// The controllable vehicle component.
    SharedPtr<Vehicle> vehicle_;

    /// Flag when player is dead.
    bool killed_;
    bool isAI_;
    int agentIndex;
    int id_;
    int type_;
    unsigned int wpActiveIndex_;
    int targetAgentIndex_;


    /// Bullets
    String bulletType_;
    Vector<Vector3>* waypoints_ = nullptr;
    Vector3 toTarget_ = Vector3::ZERO;


    //    btCollisionShape                    *sphShape_;

    /// parameter

    float mass_;
    float speed_;
    float maxSpeed_;
    float damping_;
    float acceleration_;
    float brake_;
    Vector3 towards_;
    Vector2 towards2d_;
    float turningVelocity_;
    float lastFire_;
    float heading_;
    bool isReady_;
    int life_;
    int score_;
    int health_;

    Vector3 force_;
    Vector3 offset_;

    float changeTargetTime_;
    bool autoSteering_;

    bool doJump_;
};

