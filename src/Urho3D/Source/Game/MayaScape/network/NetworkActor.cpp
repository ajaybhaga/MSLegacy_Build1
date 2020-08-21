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

#include "NetworkActor.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
NetworkActor::NetworkActor(Context* context)
    : ClientObj(context)
    , mass_(1.0f)
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
    //

    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node* adjNode = GetScene()->CreateChild("AdjNode");
    adjNode->SetRotation(Quaternion(0.0, 0.0, -90.0f));

    // Init vehicle
    Node* vehicleNode = GetScene()->CreateChild("Vehicle");

    // Place on track
    vehicleNode->SetPosition(Vector3(-814.0f+Random(-400.f, 400.0f), 150.0f, -595.0f+Random(-400.f, 400.0f)));
    vehicleNode->SetEnabled(false);
    // Create the vehicle logic component
    vehicle_ = vehicleNode->CreateComponent<Vehicle>();
    vehicle_->Init();
//    SetControls(ve)

    wpActiveIndex_ = 0;
    targetAgentIndex_ = 0;

    pRigidBody_ = vehicleNode->CreateComponent<RigidBody>();
//    pRigidBody_->SetMass(1.0f);
//    pRigidBody_->SetUseGravity(false);
    //   pRigidBody_->SetTrigger(true);


//    pRigidBody_ = vehicle_->GetRigidBody();

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

    ///



    ResourceCache* cache = GetSubsystem<ResourceCache>();

    // model
    StaticModel* ballModel = node_->GetOrCreateComponent<StaticModel>();
    ballModel->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
    String matName = ToString("NetDemo/ballmat%i.xml", colorIdx_);
    ballModel->SetMaterial(cache->GetResource<Material>(matName));
    ballModel->SetCastShadows(true);

    // physics components
    hullBody_ = node_->GetOrCreateComponent<RigidBody>();
    hullBody_->SetCollisionLayer(NETWORKACTOR_COL_LAYER);
    hullBody_->SetMass(mass_);
    hullBody_->SetFriction(1.0f);
    hullBody_->SetLinearDamping(0.5f);
    hullBody_->SetAngularDamping(0.5f);
    CollisionShape* shape = node_->GetOrCreateComponent<CollisionShape>();
    shape->SetSphere(1.0f);

    // create text3d client info node LOCALLY
    nodeInfo_ = GetScene()->CreateChild("light", LOCAL);
    nodeInfo_->SetPosition(node_->GetPosition() + Vector3(0.0f, 1.1f, 0.0f));
    Text3D *text3D = nodeInfo_->CreateComponent<Text3D>();
    text3D->SetColor(Color::GREEN);
    text3D->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 12);
    text3D->SetText(userName_);
    text3D->SetFaceCameraMode(FC_ROTATE_XYZ);

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
    if (!hullBody_ || !nodeInfo_)
    {
        return;
    }

    const float MOVE_TORQUE = 3.0f;
    Quaternion rotation(0.0f, controls_.yaw_, 0.0f);

    if (controls_.buttons_ & NTWK_CTRL_FORWARD)
    {
        hullBody_->ApplyTorque(rotation * Vector3::RIGHT * MOVE_TORQUE);
    }
    if (controls_.buttons_ & NTWK_CTRL_BACK)
    {
        hullBody_->ApplyTorque(rotation * Vector3::LEFT * MOVE_TORQUE);
    }
    if (controls_.buttons_ & NTWK_CTRL_LEFT)
    {
        hullBody_->ApplyTorque(rotation * Vector3::FORWARD * MOVE_TORQUE);
    }
    if (controls_.buttons_ & NTWK_CTRL_RIGHT)
    {
        hullBody_->ApplyTorque(rotation * Vector3::BACK * MOVE_TORQUE);
    }

    if (controls_.IsPressed(NTWK_SWAP_MAT, prevControls_))
    {
        SwapMat();
    }
////



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

    if (toTarget_ != Vector3::ZERO) {
        // Only pass once rigid body is setup
        if (pRigidBody_) {
            // Compute steer force
            ComputeSteerForce();
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


                if (autoSteering_) {
                    URHO3D_LOGINFOF("***** Player AUTO-STEERING ENABLED - Vehicle Steer [%f]", vehicle_->GetSteering());
                    URHO3D_LOGINFOF("***** Player - waypoint [%d]=[%f,%f,%f,%f]", wpActiveIndex_,
                                    waypoints_->At(wpActiveIndex_).x_, waypoints_->At(wpActiveIndex_).y_,
                                    waypoints_->At(wpActiveIndex_).z_, steering);
                    URHO3D_LOGINFOF("***** Player - target =[%f,%f,%f]", toTarget_.x_, toTarget_.y_, toTarget_.z_);

                    // Enable auto-steering
                    vehicle_->UpdateSteering(steering);
                }
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
            }*/
            }


        }
    }


    ////

    // update prev
    prevControls_ = controls_;

    // update text pos
    nodeInfo_->SetPosition(node_->GetPosition() + Vector3(0.0f, 0.7f, 0.0f));
}


