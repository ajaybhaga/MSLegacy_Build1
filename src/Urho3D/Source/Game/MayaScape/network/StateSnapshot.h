#pragma once


#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Quaternion.h>
#include <Urho3D/IO/VectorBuffer.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/IO/MemoryBuffer.h>

using namespace Urho3D;

struct ObjectState
{
    Node* node_;
    VectorBuffer* message_;
    Scene* scene_;

    void write_state(VectorBuffer* message, Scene* scene) {
        message_ = message;
        scene_ = scene;
    }

    void read_state(MemoryBuffer &message, Scene* scene) {

        unsigned int buttons_ = message.ReadUInt();
        float yaw_ = message.ReadFloat();
        float pitch_ = message.ReadFloat();
        VariantMap extraData_ = message.ReadVariantMap();

        scene_ = scene;
    }

    void add_node(Node *node) {
        node_ = node;
    }
};

typedef struct ObjectState StateSnapshot;

