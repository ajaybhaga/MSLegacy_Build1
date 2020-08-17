#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Urho2D/Sprite2D.h>
#include <Urho3D/Urho2D/CollisionBox2D.h>
#include <Urho3D/Urho2D/PhysicsWorld2D.h>
#include <Urho3D/Urho2D/PhysicsEvents2D.h>
#include <Urho3D/Urho2D/RigidBody2D.h>

#include <Urho3D/Network/Network.h>
#include <Urho3D/Network/NetworkEvents.h>

#include <Urho3D/Urho2D/Sprite2D.h>
#include <Urho3D/Urho2D/AnimatedSprite2D.h>
#include <Urho3D/Urho2D/AnimationSet2D.h>

#include "Toolkit.h"

#include "Player.h"
#include "RaycastVehicle.h"
#include "Vehicle.h"
#include "Missile.h"
#include "types.h"
#include "Bullet.h"
#include "AP.h"
#include "BUFF.h"
#include "HeatSource.h"
#include "Skill.h"
#include "Skill_Missile.h"
#include "Skill_Flare.h"
#include "Skill_Blink.h"

#define PI 3.1415926


Player::Player(Context* context) : GameObject(context)
{
	///Set original staus
	SetMaxHp(100.0f);
	SetHp(GetMaxHp());
	SetMass(10.0f);
	SetCollisionSize(Vector2(0.32f, 0.32f));
	SetInvincible(false);
	SetSpeed(0.0f);
	SetMaxSpeed(5.0f);
	SetDamping(0.015f);
	SetAcceleration(0.03f);
	SetBrake(0.05f);
	SetTowards(Vector3(0.0f, 1.0f, 0.0f));
	SetTurningVelocity(100.0f);
	SetBulletType("AP");
	///Set bullets type
	///Test bullets
	testcnt_ = 0;
	///4test
	nettest1209_ = 0;

    bulletType_ = "CB";
    lastFire_ = 0;
//    bulletType_ = "AP";
}

void Player::RegisterObject(Context* context)
{
	context->RegisterFactory<Player>();
}

//=============================================================================
// This function is called only from the main program when initially creating
// the vehicle, not on scene load
//=============================================================================
void Player::Init()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node* adjNode = GetScene()->CreateChild("AdjNode");
    adjNode->SetRotation(Quaternion(0.0, 0.0, -90.0f));

    // Init vehicle
    Node* vehicleNode = GetScene()->CreateChild("Vehicle");

    // Place on track
    vehicleNode->SetPosition(Vector3(-814.0f+Random(-400.f, 400.0f), 400.0f, -595.0f+Random(-400.f, 400.0f)));

    // Create the vehicle logic component
    vehicle_ = vehicleNode->CreateComponent<Vehicle>();
    vehicle_->Init();
//    SetControls(ve)

    // Store initial vehicle position as focus
//    focusObjects_.Push(vehicle_->GetNode()->GetPosition());

    // Create a directional light with cascaded shadow mapping
    vehicleHeadLamp_ = GetScene()->CreateChild("DirectionalLight");
    vehicleHeadLamp_->SetPosition(Vector3(vehicleNode->GetPosition().x_, 100.0f, vehicleNode->GetPosition().z_));
    vehicleHeadLamp_->SetDirection(Vector3(vehicleNode->GetRotation().x_, vehicleNode->GetRotation().y_, vehicleNode->GetRotation().z_));


    Light* light = vehicleHeadLamp_->CreateComponent<Light>();
    light->SetRadius(0.03f);
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetBrightness(0.24);
    light->SetCastShadows(true);
    //   light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
//    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    //  light->SetSpecularIntensity(0.01f);


    // Place on at corner of map
 //   TerrainPatch* p = terrain_->GetPatch(0, 0);
//    IntVector2 v = p->GetCoordinates();
//    vehicleNode->SetPosition(Vector3(v.x_-1400.0f, 40.0f, v.y_-1400.0f));
//    vehicleNode->SetPosition(Vector3(v.x_-1400.0f, 40.0f, v.y_+1400.0f));

    vehicleNode->SetRotation(Quaternion(0.0, -90.0, 0.0));

}

void Player::InitiateWeapons()
{
	///Initiate bullets

}

void Player::Start()
{
	Node* node = GetNode();
/*
	/// sprite
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	Sprite2D* sprite2d = cache->GetResource<Sprite2D>("Urho2D/Player.png");
	StaticSprite2D* staticsprite = node->CreateComponent<StaticSprite2D>();
	staticsprite->SetColor(Color(Random(1.0f), Random(1.0f), Random(1.0f)));
	staticsprite->SetSprite(sprite2d);
*/
	/// Add rigidbody
	/*rigibody2d_ = node->CreateComponent<RigidBody2D>(LOCAL);
	rigibody2d_->SetBodyType(BT_DYNAMIC);
	rigibody2d_->SetMass(GetMass());

	/// Add collisionbox
	collisionbox2d_ = node->CreateComponent<CollisionBox2D>(LOCAL);
	collisionbox2d_->SetSize(GetCollisionSize());
*/

/*	/// BUFF
	// HeatSource
	HeatSource* heatsource_ = node->CreateComponent<HeatSource>(LOCAL);
	heatsource_->SetOwner(SharedPtr<Node>(node));
	heatsource_->SetAttraction(3.0f);

	/// Skills
	// Missile
	Skill_Missile* skillMissile = node->CreateComponent<Skill_Missile>(LOCAL);
	skillMissile->SetTriggerKey(CTRL_Q);
	// Flare
	Skill_Flare* skillFlare = node->CreateComponent<Skill_Flare>(LOCAL);
	skillFlare->SetTriggerKey(CTRL_W);	
	// Blink
	Skill_Blink* skillBlink = node->CreateComponent<Skill_Blink>(LOCAL);
	skillBlink->SetTriggerKey(CTRL_E);
	/// Add some static buff to the Player
	//node->CreateComponent<HeatSource>();*/
}

void Player::Update(float timeStep)
{
	//nettest1209_++;
	//Toolkit::Print(String(nettest1209_));
}

void Player::FixedUpdate(float timeStep)
{
    lastFire_ += timeStep;

	/// Clients should not update the component on its own
	controls_ = vehicle_->controls_;


	Node* node = GetNode();
	///Acceleration
	if (controls_.IsDown(CTRL_UP)) {
		Accelerate();
	}
	///Damping
	else {
		Damping();
	}
	///Braking
	if (controls_.IsDown(CTRL_DOWN)) {
		Brake();
	}
	///Turn left
	if (controls_.IsDown(CTRL_LEFT)) {
		//Turn left
		//towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) - towards_.y_*sin(turningVelocity_*timeStep), towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);		
		node->Rotate2D(turningVelocity_*timeStep);
		// The angle between rotation2d and x-axis
		float angle = 90.0f + node->GetRotation2D();
		// The towards vector according to the angle
		towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
	}
	///Turn right
	if (controls_.IsDown(CTRL_RIGHT)) {
		//Turn right
		//towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) + towards_.y_*sin(turningVelocity_*timeStep), -towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);	
		node->Rotate2D(-turningVelocity_*timeStep);
		// The angle between rotation2d and x-axis
		float angle = 90.0f + node->GetRotation2D();
		// The towards vector according to the angle
		towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
	}



	///Rotate the Player
	//node->SetRotation(Quaternion(Vector3(0.0f, 1.0f, 0.0f), towards_));

	///Impulse the Player(may be not true)
	//rigibody2d_->SetLinearVelocity(Vector2(towards_.x_, towards_.y_).Normalized() * speed_);

	/// If the Player'hp <=0 ,then destroy it.
	if (GetHp() <= 0.0f) this->Destroyed();
}


void Player::Fire(Vector3 target)
{
    node_ = GetNode();
	Scene* scene = GetScene();

    URHO3D_LOGDEBUG("Player::Fire()");

    // 4test
	// Only for test
	if (bulletType_ == "AP") {
/*
		testcnt_++;
		Node* bullet0 = scene->CreateChild("bullet", REPLICATED);
		bullet0->CreateComponent<AP>(LOCAL);
		// Set the ownership of the bullet to the Player
	//	bullet0->GetComponent<AP>()->SetProducer(node);
		// Set the position and rotation of the bullet
		bullet0->SetWorldPosition(node->GetPosition() + towards_.Normalized()*0.2f);
		bullet0->SetWorldRotation(Quaternion(Vector3::UP, towards_));
//		bullet0->GetComponent<RigidBody2D>()->SetLinearVelocity(Vector2(towards_.x_, towards_.y_).Normalized() * 10.0f);
        URHO3D_LOGDEBUGF("Player::Fire() -> %d", testcnt_);*/
    } else {
	    //
        // bulletType_ = "CB"
        testcnt_++;

        /*
        if (testcnt_ > 4) {
            return;
        }*/


        SharedPtr<Node> n;
        Node* bullet0 = scene->CreateChild("bullet", REPLICATED);
        Missile* newM = bullet0->CreateComponent<Missile>(LOCAL);
        Node* tgt = scene->CreateChild("missileTarget", LOCAL);
        //tgt->>SetPosition(0f,0f,0f);
        tgt->SetPosition(target);
        newM->AddTarget(SharedPtr<Node>(tgt));
        // Assign the producer node
//        n = node_;
        // Set the ownership of the bullet to the Player
        bullet0->GetComponent<Missile>()->SetProducer(SharedPtr<Node>(vehicle_->GetNode()));
        newM->SnapToProducer();
        // Set the position and rotation of the bullet
//        bullet0->SetWorldPosition(node_->GetPosition() + towards_.Normalized()*0.2f);
 //       bullet0->SetWorldRotation(Quaternion(Vector3::UP, towards_));
//		bullet0->GetComponent<RigidBody2D>()->SetLinearVelocity(Vector2(towards_.x_, towards_.y_).Normalized() * 10.0f);
        URHO3D_LOGDEBUGF("Player::Fire() -> %d, [%f,%f,%f]", testcnt_, newM->GetNode()->GetPosition().x_,
                         newM->GetNode()->GetPosition().y_,
                         newM->GetNode()->GetPosition().z_);


    }
}

void Player::Destroyed()
{
	Node* node = GetNode();
	node->Remove();
}