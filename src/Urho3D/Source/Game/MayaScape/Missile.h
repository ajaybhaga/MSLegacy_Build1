#pragma once

#include <Urho3D/Scene/LogicComponent.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/ParticleEmitter.h>
#include "GameObject.h"
#include "Player.h"

using namespace Urho3D;

class Player;

class Missile : public GameObject
{
	URHO3D_OBJECT(Missile, LogicComponent);

private:
	/// Rocket thrust of the missile
	float thrust;
	/// The detection range of the missile
	float detectionRange;
	/// I really dont know how to describe this in English. Just boom range!!!
	float boomRange;
	/// Boom Damage
	float damage;
	/// Missile Node
	SharedPtr<Node> node_;
    SharedPtr<RigidBody> pRigidBody_;
    SharedPtr<CollisionShape> pCollisionShape_;
    SharedPtr<StaticModel> pObject_;
    SharedPtr<ParticleEmitter> pParticleEmitter_;

    SharedPtr<Player> owner_;

	/// Target Node
	Vector<SharedPtr<Node>>targetnodes_;
	/// Owner of the missile
//	SharedPtr<Node>producer_;
	/// Owner ID of the missile
	int producerId_;
	/// Time the node has lasted for
	float duration_;

    Vector3 force_;

//    float timer;
    bool active_;

public:

    void AddTarget(SharedPtr<Node> targetnode) { targetnodes_.Push(targetnode); }

	float GetThrust() { return thrust; }
	void SetThrust(float m_thrust) { thrust = m_thrust; }
	float GetDetectionRange() { return detectionRange; }
	void SetDetectionRange(float m_detectionRange) { detectionRange = m_detectionRange; }
	float GetBoomRange() { return boomRange; }
	void SetBoomRange(float m_boomRange) { boomRange = m_boomRange; }
	float GetDamage() { return damage; }
	void SetDamage(float m_damage) { damage = m_damage; }
	int GetProducer() { return producerId_; }
    void SetProducer(int producerId) {
        producerId_ = producerId;
    }

    void AssignProducer(int producerId, Vector3 spawnLoc);

    /// Life-cycle function
	/// Construct.
	Missile(Context* context);
	/// Register object factory and attributes.
	static void RegisterObject(Context* context);
	/// Handle startup. Called by LogicComponent base class.
	void Start();
	/// Handle physics world update. Called by LogicComponent base class.
	void FixedUpdate(float timeStep);
	/// Handle update
	void Update(float timeStep);

	/// Event handle functions
	/// When the missile has detected some heatsource, add it into the tracking queue.
//	void HandleContactBegin(StringHash eventType, VariantMap& eventData);
	/// When the missle lost the target
//	void HandleContactEnd(StringHash eventType, VariantMap& eventData);

    // Player node collision
    void HandleNodeCollision(StringHash eventType, VariantMap & eventData);
    void HandleMissileCollision(StringHash eventType, VariantMap &eventData);

};