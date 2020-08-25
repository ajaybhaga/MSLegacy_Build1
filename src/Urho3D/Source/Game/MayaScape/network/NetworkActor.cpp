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
#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/UI/Text3D.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Math/MathDefs.h>
#include <Urho3D/Graphics/DebugRenderer.h>

#include "NetworkActor.h"

#include <Urho3D/DebugNew.h>
#include <MayaScape/Missile.h>

#define PI 3.1415926

//=============================================================================
//=============================================================================
NetworkActor::NetworkActor(Context* context)
    : ClientObj(context)
    , mass_(10.0f)
{
    SetUpdateEventMask(0);
}

NetworkActor::~NetworkActor()
{
    if (nodeInfo_)
    {
        nodeInfo_->Remove();
    }
}

void NetworkActor::RegisterObject(Context* context)
{
    context->RegisterFactory<NetworkActor>();

    URHO3D_COPY_BASE_ATTRIBUTES(ClientObj);
}

void NetworkActor::ApplyAttributes()
{
}

void NetworkActor::DelayedStart()
{
    Create();
}

void NetworkActor::Create()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node* adjNode = GetScene()->CreateChild("AdjNode", LOCAL);
    adjNode->SetRotation(Quaternion(0.0, 0.0, -90.0f));


    // Init vehicle
    Node* vehicleNode = GetScene()->CreateChild("Vehicle", LOCAL);

    // Default at (0,300,0) above terrain before we set location
    GetNode()->SetPosition(Vector3(0,300,0));

    // Place on track
//    vehicleNode->SetPosition(Vector3(-814.0f+Random(-400.f, 400.0f), 150.0f, -595.0f+Random(-400.f, 400.0f)));
    vehicleNode->SetPosition(Vector3(0,100,0));

    // Create the vehicle logic component
    vehicle_ = vehicleNode->CreateComponent<Vehicle>(LOCAL);
    vehicle_->Init(isServer_);
//    SetControls(ve)

    wpActiveIndex_ = 0;
    targetAgentIndex_ = 0;

    // physics components
//    pRigidBody_->SetUseGravity(false);

    pRigidBody_ = vehicleNode->GetOrCreateComponent<RigidBody>();
   /* pRigidBody_->SetCollisionLayer(NETWORKACTOR_COL_LAYER);
    pRigidBody_->SetMass(mass_);
    pRigidBody_->SetFriction(1.0f);
    pRigidBody_->SetLinearDamping(0.5f);
    pRigidBody_->SetAngularDamping(0.5f);
    CollisionShape* shape = vehicleNode->GetOrCreateComponent<CollisionShape>(LOCAL);
    shape->SetSphere(1.0f);*/
    vehicleNode->SetRotation(Quaternion(0.0, -90.0, 0.0));


    // create text3d client info node LOCALLY
    nodeInfo_ = GetScene()->CreateChild("light", LOCAL);
    floatingText_ = nodeInfo_->CreateComponent<Text3D>();
    floatingText_->SetColor(Color::GREEN);
    floatingText_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 20);
    floatingText_->SetFaceCameraMode(FC_ROTATE_XYZ);
    // create text3d client info node LOCALLY
//    nodeInfo_ = GetScene()->CreateChild("light", LOCAL);


/*

    Text3D *text3D = nodeInfo_->CreateComponent<Text3D>();
    text3D->SetColor(Color::GREEN);
    text3D->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 40);
    text3D->SetText(userName_);
    text3D->SetFaceCameraMode(FC_ROTATE_XYZ);
*/
    // register
    SetUpdateEventMask(USE_FIXEDUPDATE);
}

void NetworkActor::SwapMat()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    int idx = Random(MAX_MAT_COUNT);
    while (idx == colorIdx_)
    {
        idx = Random(MAX_MAT_COUNT);
    }

    // update serializable of the change
    SetAttribute("Color Index", Variant(idx));

    String matName = ToString("NetDemo/ballmat%i.xml", colorIdx_);
    StaticModel* ballModel = node_->GetComponent<StaticModel>();
    ballModel->SetMaterial(cache->GetResource<Material>(matName));
}



void NetworkActor::FixedUpdate(float timeStep)
{
    if (!pRigidBody_ || !nodeInfo_)
    {
        return;
    }


    // Snap network actor position to vehicle
    GetNode()->SetPosition(vehicle_->GetNode()->GetPosition());


    // TODO: 3d text not showing up
    nodeInfo_->SetPosition(GetNode()->GetPosition() + Vector3(0.0f, 1.1f, 0.0f));
//    floatingText_->SetEnabled(true);
    // update text pos
//    nodeInfo_->SetPosition(node_->GetPosition() + Vector3(0.0f, 0.7f, 0.0f));


    // OLD SIMPLE NETWORK SAMPLE CODE
    const float MOVE_TORQUE = 3.0f;
    Quaternion rotation(0.0f, controls_.yaw_, 0.0f);

        if (controls_.buttons_ & NTWK_CTRL_FORWARD)
    {
        Accelerate();
    //    pRigidBody_->ApplyTorque(rotation * Vector3::RIGHT * MOVE_TORQUE);
    }
    if (controls_.buttons_ & NTWK_CTRL_BACK)
    {
     //   pRigidBody_->ApplyTorque(rotation * Vector3::LEFT * MOVE_TORQUE);
    }
    if (controls_.buttons_ & NTWK_CTRL_LEFT)
    {
       // pRigidBody_->ApplyTorque(rotation * Vector3::FORWARD * MOVE_TORQUE);
    }
    if (controls_.buttons_ & NTWK_CTRL_RIGHT)
    {
     //   pRigidBody_->ApplyTorque(rotation * Vector3::BACK * MOVE_TORQUE);
    }

    /*
    if (controls_.IsPressed(NTWK_SWAP_MAT, prevControls_))
    {
        SwapMat();
    }*/
////



    lastFire_ += timeStep;

    /// Clients should not update the component on its own
//    controls_ = vehicle_->controls_;

    /*

    Node* node = GetNode();
    ///Acceleration
    if (controls_.IsDown(NTWK_CTRL_FORWARD)) {
        Accelerate();
    }
        ///Damping
    else {
        Damping();
    }
    ///Braking
    if (controls_.IsDown(NTWK_CTRL_BACK)) {
        Brake();
    }
    ///Turn left
    if (controls_.IsDown(NTWK_CTRL_LEFT)) {
        //Turn left
        //towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) - towards_.y_*sin(turningVelocity_*timeStep), towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);
        node->Rotate2D(turningVelocity_*timeStep);
        // The angle between rotation2d and x-axis
        float angle = 90.0f + node->GetRotation2D();
        // The towards vector according to the angle
        towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
    }
    ///Turn right
    if (controls_.IsDown(NTWK_CTRL_RIGHT)) {
        //Turn right
        //towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) + towards_.y_*sin(turningVelocity_*timeStep), -towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);
        node->Rotate2D(-turningVelocity_*timeStep);
        // The angle between rotation2d and x-axis
        float angle = 90.0f + node->GetRotation2D();
        // The towards vector according to the angle
        towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
    }
*/
/* AUTO-STEERING CODE
    if (toTarget_ != Vector3::ZERO) {
        // Only pass once rigid body is setup
        if (pRigidBody_) {
            // Compute steer force
//            ComputeSteerForce();
            if (force_ != Vector3::ZERO) {

//            force_ = Vector3(1.0f, 0.0f, 1.0f);

                if (wpActiveIndex_ < 0)
                    return;

//                float wpOffsetX = -mapDim_ / 2;
//                float wpOffsetY = -mapDim_ / 2;
                // Convert marker position to world position for track
                //          float wpPosX = (((float)waypoints_->At(wpActiveIndex_).x_ / (float)miniMapWidth_)*mapDim_)+wpOffsetX;
//            float wpPosZ = (((float)waypoints_->At(wpActiveIndex_).z_ / (float)miniMapHeight_)*mapDim_)+wpOffsetY;


                //Vector3 tgt = Vector3(waypoints_->At(wpActiveIndex_).x_, waypoints_->At(wpActiveIndex_).y_, waypoints_->At(wpActiveIndex_).z_);


                // Calculate distance to waypoint
                Vector3 v = vehicle_->GetNode()->GetPosition() - toTarget_;// + Vector3(-1500,0,-1500);
                float steering = v.Normalized().DotProduct((vehicle_->GetNode()->GetDirection()))+0.4f;
/*
                if (steering > 1.0f) {
                    steering = -1.0f;
                }

                if (steering < -1.0f) {
                    steering = 1.0f;
                }
*/


            /*    if (autoSteering_) {
                    URHO3D_LOGINFOF("***** Player AUTO-STEERING ENABLED - Vehicle Steer [%f]", vehicle_->GetSteering());
                    URHO3D_LOGINFOF("***** Player - waypoint [%d]=[%f,%f,%f,%f]", wpActiveIndex_,
                                    waypoints_->At(wpActiveIndex_).x_, waypoints_->At(wpActiveIndex_).y_,
                                    waypoints_->At(wpActiveIndex_).z_, steering);
                    URHO3D_LOGINFOF("***** Player - target =[%f,%f,%f]", toTarget_.x_, toTarget_.y_, toTarget_.z_);

                    // Enable auto-steering
                    vehicle_->UpdateSteering(steering);
                }*/
                //vehicle_->GetRigidBody()->
//            pRigidBody_->ApplyForce(force_);
/*

            Vector3 vel = pRigidBody_->GetLinearVelocity();

            float d = vel.Length();
            if (d < 10.0f) {
                d = 10.0f;
                pRigidBody_->SetLinearVelocity(vel.Normalized() * d);
            } else if (d > 50.0f) {
                d = 50.0f;
                pRigidBody_->SetLinearVelocity(vel.Normalized() * d);
            }

            Quaternion endRot = Quaternion(0, 0, 0);
            Vector3 nVel = vel.Normalized();
            endRot.FromLookRotation(nVel, Vector3::UP);
            endRot = endRot * Quaternion(90, 0, 0);
            pRigidBody_->SetRotation(endRot);

            Vector3 p = pRigidBody_->GetPosition();
            if (p.y_ < 10.0f) {
                p.y_ = 10.0f;
                pRigidBody_->SetPosition(p);
            } else if (p.y_ > 150.0f) {
                p.y_ = 150.0f;
                pRigidBody_->SetPosition(p);
            }
            }


        }
    }

*/
    ////

    // update prev
    prevControls_ = controls_;

    // update text pos
    nodeInfo_->SetPosition(node_->GetPosition() + Vector3(0.0f, 0.7f, 0.0f));
}


void NetworkActor::ComputeSteerForce() {

    //set the force member variable to zero
    force_ = Vector3(0, 0, 0);

    if (!waypoints_)
        return;

    if ((!pRigidBody_) || (waypoints_->Empty())) {
        return;
    }
    //Attraction force
/*
    Vector3 CoM; //centre of mass, accumulated total
    int nAttract = 0; //count number of neighbours
    //set the force member variable to zero
    force_ = Vector3(0, 0, 0);
    //Search Neighbourhood
    for (int i = 0; i < numberOfBoids; i++)
    {
        //the current boid?
        if (this == &boidList[i]) continue;
        //sep = vector position of this boid from current oid
        Vector3 sep = pRigidBody->GetPosition() - boidList[i].pRigidBody->GetPosition();
        float d = sep.Length(); //distance of boid
        if (d < Range_FAttract)
        {
            //with range, so is a neighbour
            CoM += boidList[i].pRigidBody->GetPosition();
            nAttract++;
        }
    }
    if (nAttract > 0)
    {
        CoM /= nAttract;
        Vector3 dir = (CoM - pRigidBody->GetPosition()).Normalized();
        Vector3 vDesired = dir * FAttract_Vmax;
        force += (vDesired - pRigidBody->GetLinearVelocity())*FAttract_Factor;
    }
    if (nAttract > 5)
    {
        // stop checking once 5 neighbours have been found
        return;
    }

    //seperation force
    Vector3 sepForce;
    int nRepel = 0;
    for (int i = 0; i < numberOfBoids; i++)
    {
        //the current boid?
        if (this == &boidList[i]) continue;
        //sep = vector position of this boid from current oid
        Vector3 sep = pRigidBody->GetPosition() - boidList[i].pRigidBody->GetPosition();
        float d = sep.Length(); //distance of boid
        if (d < Range_FRepel)
        {
            sepForce += (sep / sep.Length());
            nRepel++;
        }
    }
    if (nRepel > 0)
    {
        sepForce *= FRepel_Factor;
        force += sepForce;
    }
    if (nRepel > 5)
    {
        // stop checking once 5 neighbours have been found
        return;
    }
    */

    //Allignment direction
    Vector3 align;
    int nAlign = 0;


    //sep = vector position of this boid from current oid

//    Vector3 sep = pRigidBody->GetPosition() - boidList[i].pRigidBody->GetPosition();
//    float d = sep.Length(); //distance of boid
//    if (d < Range_FRepel)


    Vector3 toTarget;
    if (!waypoints_->Empty()) {

//        URHO3D_LOGDEBUGF("Player::ComputeSteerForce() waypoints -> [%d] -> set to  %d", waypoints_->Size(), wpActiveIndex_);

        // Calculate distance to waypoint
        toTarget = pRigidBody_->GetPosition() - waypoints_->At(wpActiveIndex_);
    } else {
        return;
    }


    //    Vector3 dir = sep.Normalized();
//    float d = sep.Length(); // distance of boid

    //float steer = toTarget * speed / d;

    float speed = 1.0f;
    Vector3 desiredVelocity = toTarget.Normalized() * speed;


    /*
    if (d < Range_FAlign)
    {
        align += boidList[i].pRigidBody->GetLinearVelocity();
        nAlign++;
    }*/

    force_ += (desiredVelocity - pRigidBody_->GetLinearVelocity());

    /*
    if (nAlign > 0)
    {
        align /= nAlign;

        Vector3 finalVel = align;

//        force_ += (finalVel - pRigidBody_->GetLinearVelocity()) * FAlign_Factor;
    }
    if (nAlign > 5)
    {
        // stop checking once 5 neighbours have been found
        return;
    }*/
}


void NetworkActor::Fire() {
    Fire(toTarget_);
}

void NetworkActor::Fire(Vector3 target)
{
    node_ = GetNode();
    Scene* scene = GetScene();


    URHO3D_LOGDEBUG("NetworkActor::Fire()");

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

        /*
        if (testcnt_ > 4) {
            return;
        }*/

/*
        VariantMap& eventData = GetNode()->GetEventDataMap();
        eventData[P_DATA] = GetNode()->GetWorldPosition();
        SendEvent(StringHash("Blast"), eventData);

        GetNode()->Remove();
*/

        SharedPtr<Node> n;
        Node* bullet0 = scene->CreateChild("bullet", REPLICATED);
        Missile* newM = bullet0->CreateComponent<Missile>(LOCAL);
        newM->SetProducer(vehicle_->GetNode()->GetID());


        // Store local missile list
        //missileList_.Push(vehicle_->GetNode()->GetID());

//        VariantMap& eventData = GetNode()->GetEventDataMap();
//        eventData["owner"] = SharedPtr<Player>(this);

//        eventData["missileOwner"] = this->GetID();
        //       vehicle_->SendEvent(E_NODECOLLISION, eventData);



        // Set the ownership of the bullet to the Player
//        bullet0->GetComponent<Missile>()->SetProducer(SharedPtr<Node>(vehicle_->GetNode()));

        Node* tgt = scene->CreateChild("missileTarget", LOCAL);
        //tgt->>SetPosition(0f,0f,0f);
        tgt->SetPosition(target);
        newM->AddTarget(SharedPtr<Node>(tgt));
        // Assign the producer node
        newM->AssignProducer(vehicle_->GetNode()->GetID(), vehicle_->GetNode()->GetPosition()+Vector3(0.0f, 2.0f, 0.0f));
        URHO3D_LOGDEBUGF("NetworkActor::Fire() [%d] -> [%f,%f,%f]", vehicle_->GetNode()->GetID(), newM->GetNode()->GetPosition().x_,
                         newM->GetNode()->GetPosition().y_,
                         newM->GetNode()->GetPosition().z_);


    }
}


void NetworkActor::DebugDraw(const Color &color)
{
    if (!vehicle_)
        return;

    DebugRenderer *dbgRenderer = GetScene()->GetComponent<DebugRenderer>();
    node_ = GetNode();


    if ( dbgRenderer )
    {

        // draw compound shape bounding box (the inertia bbox)
        Vector3 localExtents = vehicle_->GetRaycastVehicle()->GetCompoundLocalExtents();
        Vector3 localCenter  = vehicle_->GetRaycastVehicle()->GetCompooundLocalExtentsCenter();
        BoundingBox bbox(-localExtents, localExtents);

        btTransform trans;
        vehicle_->GetRaycastVehicle()->getWorldTransform(trans);
        Vector3 posWS = ToVector3(trans.getOrigin());
        Vector3 centerWS = ToQuaternion(trans.getRotation()) * localCenter;
        posWS += centerWS;
        Matrix3x4 mat34(posWS, ToQuaternion(trans.getRotation()), 1.0f);

        /*
        dbgRenderer->AddBoundingBox(bbox, mat34, color);
        dbgRenderer->AddLine(posWS, posWS + node_->GetUp(), color);
        dbgRenderer->AddLine(posWS, posWS + node_->GetRight(), color);
*/

//        vehicle_->GetRaycastVehicle()->DrawDebugGeometry(dbgRenderer, false);

//        ToQuaternion(trans.getRotation()),
        // dbgRenderer->AddBoundingBox(bbox, mat34, color);
//        dbgRenderer->AddLine(posWS, posWS + node_->R, color);
        dbgRenderer->AddLine(posWS, posWS + this->vehicle_->GetNode()->GetDirection()*40.0f, Color::CYAN);
        dbgRenderer->AddLine(posWS, toTarget_, Color::YELLOW);
    }
}