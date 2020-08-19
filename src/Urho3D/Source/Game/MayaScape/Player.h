#pragma once

#include <Urho3D/Scene/LogicComponent.h>
#include <Urho3D/Graphics/BillboardSet.h>

#include "GameObject.h"
#include "Vehicle.h"

#pragma once
#include <Urho3D/Engine/Application.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Controls.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/Graphics/ParticleEffect.h>
#include <Urho3D/Graphics/ParticleEmitter.h>

#include "Missile.h"

namespace Urho3D
{
    class Node;
    class Scene;
    class RigidBody;
    class CollisionShape;
    class ResourceCache;
}
// All Urho3D classes reside in namespace Urho3D
using namespace Urho3D;


const float MOVE_SPEED = 30.0f;
const float CHAR_YAW_SENSITIVITY = 0.1f;
const int LIFES = 3;

struct PlayerState {
    bool onGround;
    bool jump;
    float lastJump;
    bool walk;
    float lastWalk;
    bool kick;
    float lastKick;
    Vector3 moveDir;
};

class Player : public GameObject
{
	URHO3D_OBJECT(Player, LogicComponent);

	/// parameter
    unsigned int wpActiveIndex_;
    int targetAgentIndex_;
public:
	float mass_;
	float speed_;
	float maxSpeed_;
	float damping_;
	float acceleration_;
	float brake_;
	Vector3 towards_;
	Vector2 towards2d_;
	float turningVelocity_;
	/// test
	int testcnt_;
	/// 4test
	int nettest1209_;

    btCollisionShape                    *sphShape_;

    /// The controllable vehicle component.
    SharedPtr<Vehicle> vehicle_;
    SharedPtr<Node> vehicleHeadLamp_;

    float lastFire_;

    // Previous state
    PlayerState prevState_;
    // Current state
    PlayerState currState_;

    /// Flag when player is facing forward.
    bool forward_;
    float heading_;

    int id_;
    int type_;

    bool isReady_;
    int life_;

    int score_;
    int health_;

    Vector3 force_;
    Vector3 offset_;

    Node* pNode_;
    RigidBody* pRigidBody_;
    CollisionShape* pCollisionShape_;
    StaticModel* pObject_;
    //Missile playerMissile_;

    float changeTargetTime_;
    bool autoSteering_;

    bool doJump_;

    /// The controllable vehicle component.
//    WeakPtr<Vehicle> vehicle_;

    WeakPtr<AnimationController> animCtrl_;

    AnimationState* walkState_;
    AnimationState* idleState_;
    AnimationState* jumpState_;
    AnimationState* kickState_;

    /// Flag when player is wounded.
    bool wounded_;
    /// Flag when player is dead.
    bool killed_;
    /// Timer for particle emitter duration.
    float timer_;
    /// Number of coins in the current level.
    int maxCoins_;
    /// Counter for remaining coins to pick.
    int remainingCoins_;
    /// Counter for remaining lifes.
    int remainingLifes_;
    /// Indicate when the player is climbing a ladder or a rope.
    bool isClimbing_;
    /// Used only for ropes, as they are split into 2 shapes.
    bool climb2_;
    /// Indicate when the player is above a climbable object, so we can still jump anyway.
    bool aboveClimbable_;
    /// Indicate when the player is climbing a slope, so we can apply force to its body.
    bool onSlope_;
    bool isAI_;

    int agentIndex;


    SharedPtr<Node> genotypeNode_; // Scene node displaying genotype
    SharedPtr<BillboardSet> genotypeBBSet_; // Billboard set for genotype

    SharedPtr<Node> powerbarNode_; // Scene node displaying powerbar
    SharedPtr<BillboardSet> powerbarBBSet_; // Billboard set for powerbar



    static float Range_FAttract;
    static float Range_FRepel;
    static float Range_FAlign;
    static float FAttract_Factor;
    static float FRepel_Factor;
    static float FAlign_Factor;
    static float FAttract_Vmax;


    //
private:
	/// Member variable
	Node* node_;
//	RigidBody2D* rigibody2d_;
//	CollisionBox2D* collisionbox2d_;
	/// Player Controls
	Controls controls_;
	/// Bullets
	//Vector<Bullet*> bullets_;
	String bulletType_;

    Vector<Vector3>* waypoints_ = nullptr;
    // For minimap coordinate conversion
    int miniMapWidth_;
    int miniMapHeight_;
    int mapDim_;
    Vector3 toTarget_ = Vector3::ZERO;


    ///Life-cycle functions
public:
    void Init();
    void Start();
	void Update(float timeStep);
	void FixedUpdate(float timeStep);
	/// Customized functions
	void Destroyed();
	void ComputeSteerForce();
    void DebugDraw(const Color &color);


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

    void setAgentIndex(int agentIndex) {
        Player::agentIndex = agentIndex;
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
    SharedPtr<Node> GetVehicleHeadLamp() { return vehicleHeadLamp_; }

	/// Fight
    void Fire(Vector3 target);
    void Fire();

	/// Function
	Player(Context* context);
	/// Register
	static void RegisterObject(Context* context);
	/// initiate weapons
	void InitiateWeapons();

};

