//
// Copyright (c) 2008-2018 the Urho3D project.
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

#include "../Precompiled.h"

#include "../Core/Context.h"
#include "../Graphics/DebugRenderer.h"
#include "../Resource/ResourceCache.h"
#include "../Scene/Node.h"
#include "../Urho2D/StaticSprite2D.h"
#include "../Urho2D/StaticSprite3D.h"
#include "../Urho2D/TileMap3D.h"
#include "../Urho2D/TileMapLayer3D.h"
#include "../Urho2D/TmxFile2D.h"

#include "../Graphics/Material.h"
#include "../Graphics/Model.h"
#include "../Graphics/StaticModel.h"
#include "../Graphics/Octree.h"

#include "..//IO/File.h"
#include "../IO/FileSystem.h"
#include "../IO/Log.h"


#include "../DebugNew.h"
#include <string>

namespace Urho3D
{

TileMapLayer3D::TileMapLayer3D(Context* context) :
    Component(context)
{
}

TileMapLayer3D::~TileMapLayer3D() = default;

void TileMapLayer3D::RegisterObject(Context* context)
{
    context->RegisterFactory<TileMapLayer3D>();
}

// Transform vector from node-local space to global space
static Vector2 TransformNode2D(const Matrix3x4& transform, Vector2 local)
{
    Vector3 transformed = transform * Vector4(local.x_, local.y_, 0.f, 1.f);
    return Vector2(transformed.x_, transformed.y_);
}

void TileMapLayer3D::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
{
    if (!debug)
        return;

    if (objectGroup_)
    {
        const Matrix3x4 transform = GetTileMap()->GetNode()->GetTransform();
        for (unsigned i = 0; i < objectGroup_->GetNumObjects(); ++i)
        {
            TileMapObject2D* object = objectGroup_->GetObject(i);
            const Color& color = Color::YELLOW;
            const Vector2& size = object->GetSize();
            const TileMapInfo2D& info = tileMap_->GetInfo();


            switch (object->GetObjectType())
            {
            case OT_RECTANGLE:
                {
                    Vector<Vector2> points;

                    switch (info.orientation_)
                    {
                    case O_ORTHOGONAL:
                    case O_HEXAGONAL:
                    case O_STAGGERED:
                        {
                            points.Push(Vector2::ZERO);
                            points.Push(Vector2(size.x_, 0.0f));
                            points.Push(Vector2(size.x_, -size.y_));
                            points.Push(Vector2(0.0f, -size.y_));
                            break;
                        }
                    case O_ISOMETRIC:
                        {
                            float ratio = (info.tileWidth_ / info.tileHeight_) * 0.5f;
                            points.Push(Vector2::ZERO);
                            points.Push(Vector2(size.y_ * ratio, size.y_ * 0.5f));
                            points.Push(Vector2((size.x_ + size.y_) * ratio, (-size.x_ + size.y_) * 0.5f));
                            points.Push(Vector2(size.x_ * ratio, -size.x_ * 0.5f));
                            break;
                        }
                    }

                    for (unsigned j = 0; j < points.Size(); ++j)
                        debug->AddLine(Vector3(TransformNode2D(transform, points[j] + object->GetPosition())),
                            Vector3(TransformNode2D(transform, points[(j + 1) % points.Size()] + object->GetPosition())), color,
                            depthTest);
                }
                break;

            case OT_ELLIPSE:
                {
                    const Vector2 halfSize = object->GetSize() * 0.5f;
                    float ratio = (info.tileWidth_ / info.tileHeight_) * 0.5f; // For isometric only

                    Vector2 pivot = object->GetPosition();
                    if (info.orientation_ == O_ISOMETRIC)
                    {
                        pivot += Vector2((halfSize.x_ + halfSize.y_) * ratio, (-halfSize.x_ + halfSize.y_) * 0.5f);
                    }
                    else
                    {
                        pivot += halfSize;
                    }

                    for (unsigned i = 0; i < 360; i += 30)
                    {
                        unsigned j = i + 30;
                        float x1 = halfSize.x_ * Cos((float)i);
                        float y1 = halfSize.y_ * Sin((float)i);
                        float x2 = halfSize.x_ * Cos((float)j);
                        float y2 = halfSize.y_ * Sin((float)j);
                        Vector2 point1 = Vector2(x1, - y1);
                        Vector2 point2 = Vector2(x2, - y2);

                        if (info.orientation_ == O_ISOMETRIC)
                        {
                            point1 = Vector2((point1.x_ + point1.y_) * ratio, (point1.y_ - point1.x_) * 0.5f);
                            point2 = Vector2((point2.x_ + point2.y_) * ratio, (point2.y_ - point2.x_) * 0.5f);
                        }

                        debug->AddLine(Vector3(TransformNode2D(transform, pivot + point1)),
                            Vector3(TransformNode2D(transform, pivot + point2)), color, depthTest);
                    }
                }
                break;

            case OT_POLYGON:
            case OT_POLYLINE:
                {
                    for (unsigned j = 0; j < object->GetNumPoints() - 1; ++j)
                        debug->AddLine(Vector3(TransformNode2D(transform, object->GetPoint(j))),
                            Vector3(TransformNode2D(transform, object->GetPoint(j + 1))), color, depthTest);

                    if (object->GetObjectType() == OT_POLYGON)
                        debug->AddLine(Vector3(TransformNode2D(transform, object->GetPoint(0))),
                            Vector3(TransformNode2D(transform, object->GetPoint(object->GetNumPoints() - 1))), color, depthTest);
                    // Also draw a circle at origin to indicate direction
                    else
                        debug->AddCircle(Vector3(TransformNode2D(transform, object->GetPoint(0))), Vector3::FORWARD, 0.05f, color,
                            64, depthTest);
                }
                break;

            default: break;
            }
        }
    }
}

void TileMapLayer3D::Initialize(TileMap3D* tileMap, const TmxLayer2D* tmxLayer)
{

//    URHO3D_LOGINFOF("TileMapLayer3D::Initialize -> loading tile layer models...", 0);

    if (tileMap == tileMap_ && tmxLayer == tmxLayer_)
        return;

//    URHO3D_LOGINFOF("TileMapLayer3D::Initialize -> tile layers %.2f", tileMap->GetNumLayers());

    if (tmxLayer_)
    {
        for (unsigned i = 0; i < nodes_.Size(); ++i)
        {
            if (nodes_[i])
                nodes_[i]->Remove();
        }

        nodes_.Clear();
    }

    tileLayer_ = nullptr;
    objectGroup_ = nullptr;
    imageLayer_ = nullptr;

    tileMap_ = tileMap;
    tmxLayer_ = tmxLayer;

    if (!tmxLayer_)
        return;

    switch (tmxLayer_->GetType())
    {
    case LT_TILE_LAYER:
        SetTileLayer((const TmxTileLayer2D*)tmxLayer_);
        break;

    case LT_OBJECT_GROUP:
        SetObjectGroup((const TmxObjectGroup2D*)tmxLayer_);
        break;

    case LT_IMAGE_LAYER:
        SetImageLayer((const TmxImageLayer2D*)tmxLayer_);
        break;

    default:
        break;
    }

    SetVisible(tmxLayer_->IsVisible());
}

void TileMapLayer3D::SetDrawOrder(int drawOrder)
{
    if (drawOrder == drawOrder_)
        return;

    drawOrder_ = drawOrder;

    for (unsigned i = 0; i < nodes_.Size(); ++i)
    {
        if (!nodes_[i])
            continue;

        auto* staticSprite = nodes_[i]->GetComponent<StaticSprite3D>();
        if (staticSprite)
            staticSprite->SetLayer(drawOrder_);
    }
}

void TileMapLayer3D::SetVisible(bool visible)
{
    if (visible == visible_)
        return;

    visible_ = visible;

    for (unsigned i = 0; i < nodes_.Size(); ++i)
    {
        if (nodes_[i])
            nodes_[i]->SetEnabled(visible_);
    }
}

TileMap3D* TileMapLayer3D::GetTileMap() const
{
    return tileMap_;
}

bool TileMapLayer3D::HasProperty(const String& name) const
{
    if (!tmxLayer_)
        return false;

    return tmxLayer_->HasProperty(name);
}

const String& TileMapLayer3D::GetProperty(const String& name) const
{
    if (!tmxLayer_)
        return String::EMPTY;
    return tmxLayer_->GetProperty(name);
}

TileMapLayerType2D TileMapLayer3D::GetLayerType() const
{
    return tmxLayer_ ? tmxLayer_->GetType() : LT_INVALID;
}

int TileMapLayer3D::GetWidth() const
{
    return tmxLayer_ ? tmxLayer_->GetWidth() : 0;
}

int TileMapLayer3D::GetHeight() const
{
    return tmxLayer_ ? tmxLayer_->GetHeight() : 0;
}

Tile2D* TileMapLayer3D::GetTile(int x, int y) const
{
    if (!tileLayer_)
        return nullptr;

    return tileLayer_->GetTile(x, y);
}

Node* TileMapLayer3D::GetTileNode(int x, int y) const
{
    if (!tileLayer_)
        return nullptr;

    if (x < 0 || x >= tileLayer_->GetWidth() || y < 0 || y >= tileLayer_->GetHeight())
        return nullptr;

    return nodes_[y * tileLayer_->GetWidth() + x];
}

unsigned TileMapLayer3D::GetNumObjects() const
{
    if (!objectGroup_)
        return 0;

    return objectGroup_->GetNumObjects();
}

TileMapObject2D* TileMapLayer3D::GetObject(unsigned index) const
{
    if (!objectGroup_)
        return nullptr;

    return objectGroup_->GetObject(index);
}

Node* TileMapLayer3D::GetObjectNode(unsigned index) const
{
    if (!objectGroup_)
        return nullptr;

    if (index >= nodes_.Size())
        return nullptr;

    return nodes_[index];
}

Node* TileMapLayer3D::GetImageNode() const
{
    if (!imageLayer_ || nodes_.Empty())
        return nullptr;

    return nodes_[0];
}

enum direction {
    WEST, EAST, NORTH, SOUTH
};

float xScale = 28.0f;
float yScale = 28.0f;

Vector3 TileMapLayer3D::CalculateTileShift(const TmxTileLayer2D* tileLayer, const Tile2D* tile, int x, int y) {
    unsigned int adjWTileId = -1;
    unsigned int adjETileId = -1;
    unsigned int adjNTileId = -1;
    unsigned int adjSTileId = -1;
    float xShift, yShift, zShift;
    xShift = yShift = zShift = 0.0f;

    unsigned int tileId = tile->GetGid();
    Tile2D* adjTile = nullptr;

    adjTile = tileLayer->GetTile(x-1, y);
    if (adjTile)
        adjWTileId = adjTile->GetGid();
    adjTile = nullptr;

    adjTile = tileLayer->GetTile(x+1, y);
    if (adjTile)
        adjETileId = adjTile->GetGid();
    adjTile = nullptr;

    adjTile = tileLayer->GetTile(x, y-1);
    if (adjTile)
        adjNTileId = adjTile->GetGid();
    adjTile = nullptr;

    adjTile = tileLayer->GetTile(x, y+1);
    if (adjTile)
        adjSTileId = adjTile->GetGid();
    adjTile = nullptr;


    // Adjcent W is same
    if (adjWTileId == tileId) {

    } else {

        // DETECTED - twice
        // Top left
        if (adjWTileId == 3) {
            // Squeeze in
            xShift = 0.01f;
            yShift = -yScale;
            zShift = 0.0f;

            URHO3D_LOGINFOF("W TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjWTileId, xShift, yShift);
        }

        // Top right
        if (adjWTileId == 4) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            //    xShift = 300.0f;
            URHO3D_LOGINFOF("W TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjWTileId, xShift, yShift);
        }

        // DETECTED - twice
        // Bottom left
        if (adjWTileId == 5) {
            // Squeeze in
            xShift = 0.01f;
            yShift = yScale;
            zShift = 0.0f;

            URHO3D_LOGINFOF("W TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjWTileId, xShift, yShift);

        }

        // Bottom right
        if (adjWTileId == 6) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("W TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjWTileId, xShift, yShift);
        }

    }


    // Adjcent E is same
    if (adjETileId == tileId) {

    } else {

        // Top left
        if (adjETileId == 3) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("E TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjETileId, xShift, yShift);
        }

        // DETECTED - twice
        // Top right
        if (adjETileId == 4) {
            // Squeeze in
            xShift = 0.01f;
            yShift = -yScale;
            zShift = 0.0f;

            URHO3D_LOGINFOF("E TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjETileId, xShift, yShift);
        }

        // Bottom left
        if (adjETileId == 5) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("E TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjETileId, xShift, yShift);

        }

        // DETECTED - twice
        // Bottom right
        if (adjETileId == 6) {
            // Squeeze in
            xShift = 0.01f;
            yShift = yScale;
            zShift = 0.0f;

            URHO3D_LOGINFOF("E TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjETileId, xShift, yShift);
        }

    }


    // Adjcent N is same
    if (adjNTileId == tileId) {

    } else {

        // DETECTED - twice
        // Top left
        if (adjNTileId == 3) {
            // Squeeze in
            xShift = -xScale;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("N TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjNTileId, xShift, yShift);
        }

        // DETECTED - twice
        // Top right
        if (adjNTileId == 4) {
            // Squeeze in
            xShift = xScale;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("N TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjNTileId, xShift, yShift);
        }

        // Bottom left
        if (adjNTileId == 5) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("N TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjNTileId, xShift, yShift);

        }

        // Bottom right
        if (adjNTileId == 6) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("N TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjNTileId, xShift, yShift);
        }

    }


    // Adjcent S is same
    if (adjSTileId == tileId) {

    } else {

        // Top left
        if (adjSTileId == 3) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("S TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjSTileId, xShift, yShift);
        }

        // DETECTED - twice
        // Top right
        if (adjSTileId == 4) {
            // Squeeze in
            xShift = 0.01f;
            yShift = 0.01f;
            zShift = 0.0f;

            //    xShift = 300.0f;
            URHO3D_LOGINFOF("S TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjSTileId, xShift, yShift);
        }

        // Bottom left
        if (adjSTileId == 5) {
            // Squeeze in
            xShift = -xScale;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("S TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjSTileId, xShift, yShift);

        }

        // DETECTED - twice
        // Bottom right
        if (adjSTileId == 6) {
            // Squeeze in
            xShift = xScale;
            yShift = 0.01f;
            zShift = 0.0f;

            URHO3D_LOGINFOF("S TileMapLayer3D::SetTileLayer -> Squeeze in [%d, %f, %f]", adjSTileId, xShift, yShift);
        }

    }

    return Vector3(xShift, yShift, zShift);
}

void TileMapLayer3D::SetTileLayer(const TmxTileLayer2D* tileLayer) {
    bool track = false;
    bool land = false;
    bool plant = false;
    bool building = false;

//    URHO3D_LOGINFOF("TileMapLayer3D::SetTileLayer -> loading tile layer models...", 0);

    auto *cache = GetSubsystem<ResourceCache>();

    tileLayer_ = tileLayer;
    URHO3D_LOGINFOF("TileMapLayer3D::SetTileLayer -> name = %s", tileLayer->GetName().CString());

    if (tileLayer_->GetName() == "Track") {
        track = true;
        URHO3D_LOGINFOF("TileMapLayer3D::SetTileLayer -> Track", 0);
    }

    if (tileLayer_->GetName() == "Land") {
        land = true;
        URHO3D_LOGINFOF("TileMapLayer3D::SetTileLayer -> Land", 0);
    }
    if (tileLayer_->GetName() == "Plant") {
        plant = true;
        URHO3D_LOGINFOF("TileMapLayer3D::SetTileLayer -> Plant", 0);
    }
    if (tileLayer_->GetName() == "Building") {
        building = true;
        URHO3D_LOGINFOF("TileMapLayer3D::SetTileLayer -> Building", 0);
    }


    int width = tileLayer->GetWidth();
    int height = tileLayer->GetHeight();
    nodes_.Resize((unsigned) (width * height));

    char buffer[100];
    std::string path = "Models/Tracks/Models/";

    const TileMapInfo2D &info = tileMap_->GetInfo();
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {

//            URHO3D_LOGINFOF("TileMapLayer3D::tile loop -> (x,y) : (%d, %d)", x, y);

            const Tile2D *tile = tileLayer->GetTile(x, y);

            bool doGrid = false;
            unsigned int tileId = -1;

            // On no tile, default to grid
            if (!tile) {
                //doGrid = true;
            } else {
                tileId = tile->GetGid();

                SharedPtr<Node> tileNode(GetNode()->CreateTemporaryChild("Tile"));
                auto *staticObject = tileNode->CreateComponent<StaticModel>();

                // Calculate tile shift based on adjacent neighbors
                Vector3 tileShift = CalculateTileShift(tileLayer, tile, x, y);

//
//            std::string path = "Models/";
                std::string filler = "";
                std::string terrainType = "Land";
                if (tileId > 9)
                    filler = "";
                else
                    filler = "0";
//            std::string tileStr = path + terrainType + "Tile" + filler + "%d.mdl";       
                std::string tileStr = path + "square.mdl";
                std::string matStr = "";
//            std::string tileStr;
                float xoffset;
                float elevation;
                float depth;

                float scale = 25.0f;
                Quaternion rot = Quaternion(0.0f, 0.0f, 0.0f);
                std::string str = "";

                // Set tile location
//            tileNode->SetPosition(Vector3(info.TileIndexToPosition(x, y))+tile->GetModelOffset()+Vector3(0,0,-1));
                if (track) {
                    xoffset = 0.0f;
                    depth = 0.0f;
                    elevation = 0.0f;
                    std::string str = path + "square.mdl";
                    sprintf(buffer, str.c_str(), tileId);
                    matStr = path + "square.txt";
                } else return;

                if (land) {
                    xoffset = 0.3f;
                    depth = 1.0f;
                }
                if (plant) {
                    xoffset = 0.0f;
                    depth = -0.5f;
                    elevation = 0.0f;
                }
                if (building) {
                    xoffset = 0.0f;
                    depth = 1.5f;
                    elevation = -0.6f;
                }

                if (doGrid) {
//                tileStr = path + "grid.mdl";
                    tileStr = path + "square.mdl";
                    matStr = path + "square.txt";
//                scale = 0.12f;
                    //           elevation = 4.7f;
                } else {

                    if (track) {
                        switch (tileId) {
                            case 1:
                                // Straight horizontal
                                tileStr = path + "1.mdl";
                                matStr = path + "1.txt";

                                scale = 0.04f;
                                elevation = 2.0f;
                                rot = Quaternion(180.0f, 90.0f, 90.0f);
                                break;
                            case 2:
                                // Straight vertical
                                tileStr = path + "1.mdl";
                                matStr = path + "1.txt";
                                scale = 0.04f;
                                elevation = 2.0f;
                                rot = Quaternion(90.0f, 90.0f, 90.0f);

                                break;
                            case 3:
                                // Top left
                                tileStr = path + "6.mdl";
                                matStr = path + "6.txt";

//                            xShift = 32.0f;
//                            yShift = 32.0f;

                                scale = 0.04f;
                                elevation = 2.0f;
                                rot = Quaternion(90.0f, 90.0f, 90.0f);

                                break;
                            case 4:
                                // Top right
                                tileStr = path + "6.mdl";
                                matStr = path + "6.txt";

//                            xShift = -32.0f;
//                            yShift = 32.0f;

                                scale = 0.04f;
                                elevation = 2.0f;
                                rot = Quaternion(0.0f, 90.0f, 90.0f);

                                break;
                            case 5:
                                // Bottom left
                                tileStr = path + "6.mdl";
                                matStr = path + "6.txt";

                                //                          xShift = 32.0f;
                                //                          yShift = -32.0f;

                                scale = 0.04f;
                                elevation = 2.0f;
                                rot = Quaternion(180.0f, 90.0f, 90.0f);

                                break;
                            case 6:
                                // Bottom right
                                tileStr = path + "6.mdl";
                                matStr = path + "6.txt";

                                //                        xShift = -32.0f;
                                //                        yShift = -32.0f;

                                scale = 0.04f;
                                elevation = 2.0f;
                                rot = Quaternion(270.0f, 90.0f, 90.0f);

                                break;
                            case 7:
                                break;
                            case 8:
                                break;
                        };
                    }

                    if (land) {
                        switch (tileId) {
                            case 1:
                                tileStr = path + "AssetPack/castle-wall_stone.mdl";
                                matStr = path + "AssetPack/castle-wall_stone.txt";
                                scale = 0.12f;
                                break;
                            case 2:
                                tileStr = path + "AssetPack/terrain-world-plain.mdl";
                                matStr = path + "AssetPack/terrain-world-plain.txt";
                                scale = 0.07f;
                                elevation = 0.7f;
                                depth = -0.9f;
                                break;
                            case 3:
                                break;
                            case 4:
                                break;
                            case 5:
                                break;
                            case 6:
                                break;
                            case 7:
                                break;
                            case 8:
                                break;
                        };
                    }

                    if (plant) {
                        switch (tileId) {
                            case 33:
                                tileStr = path + "AssetPack/tree-forest.mdl";
                                matStr = path + "AssetPack/tree-forest.txt";
                                break;
                            case 34:
                                tileStr = path + "AssetPack/tree-baobab.mdl";
                                matStr = path + "AssetPack/tree-baobab.txt";
                                break;
                            case 35:
                                tileStr = path + "AssetPack/tree-birch02.mdl";
                                matStr = path + "AssetPack/tree-birch02.txt";
                                break;
                            case 36:
                                tileStr = path + "AssetPack/tree-oak_T.mdl";
                                matStr = path + "AssetPack/tree-oak_T.txt";
                                break;
                            case 37:
                                tileStr = path + "AssetPack/tree-lime.mdl";
                                matStr = path + "AssetPack/tree-lime.txt";
                                break;
                            case 38:
                                tileStr = path + "AssetPack/grass01.mdl";
                                matStr = path + "AssetPack/grass01.txt";
                                break;
                            case 39:
                                tileStr = path + "AssetPack/flower01.mdl";
                                matStr = path + "AssetPack/flower01.txt";
                                break;
                            case 40:
                                tileStr = path + "AssetPack/flower02.mdl";
                                matStr = path + "AssetPack/flower02.txt";
                                break;
                        };
                    }

                    if (building) {
                        switch (tileId) {
                            case 9:
                                tileStr = path + "AssetPack/castle-tower.mdl";
                                matStr = path + "AssetPack/castle-tower.txt";
                                scale = 0.17f;
                                break;
                            case 10:
                                tileStr = path + "AssetPack/castle-tower-square.mdl";
                                matStr = path + "AssetPack/castle-tower-square.txt";
                                scale = 0.17f;
                                break;
                            case 11:
                                tileStr = path + "AssetPack/castle-gate_small.mdl";
                                matStr = path + "AssetPack/castle-gate_small.txt";
                                scale = 0.17f;
                                break;
                            case 12:
                                tileStr = path + "AssetPack/castle.mdl";
                                matStr = path + "AssetPack/castle.txt";
                                scale = 0.15f;
                                break;
                            case 13:
                                tileStr = path + "AssetPack/castle-gate.mdl";
                                matStr = path + "AssetPack/castle-gate.txt";
                                scale = 0.17f;
                                break;
                            case 14:
                                break;
                            case 15:
                                break;
                            case 16:
                                break;
                        };
                    }

                    const char *cstr = tileStr.c_str();
                    sprintf(buffer, cstr, tileId);
                }


                float tileSpacing = 80.0f;

                Vector3 tilePos = Vector3(x * tileSpacing, y * tileSpacing,
                                          0.0f);//Vector3(info.TileIndexToPosition(x, y));//)*4.0f+Vector3(xoffset,height,depth);
//            URHO3D_LOGINFOF("TileMapLayer3D::tilePos -> (x,y) : (%f, %f)", tilePos.x_, tilePos.z_);

//            URHO3D_LOGINFOF("TileMapLayer3D:: Setting Model -> (model) : (%s)", buffer);

                tileNode->SetRotation(rot);

                if (tileShift.x_ > 0.0f || tileShift.y_ > 0.0f) {
//                tileShift.z_ = 4.0f;
                }
                tileNode->SetPosition(Vector3(tileShift.x_ + tilePos.x_ + xoffset, tileShift.y_ + tilePos.y_,
                                              tileShift.z_ + elevation));
                tileNode->SetScale(Vector3(scale, scale, scale));
//
                staticObject->SetModel(cache->GetResource<Model>(buffer));
                //    String matFile = GetSubsystem<FileSystem>()->GetProgramDir() + "Data/" + matStr.c_str();
                //           staticObject->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY-COLORS.xml"));
                staticObject->ApplyMaterialList(matStr.c_str());
                //          URHO3D_LOGINFOF("TileMapLayer3D::SetTileLayer -> material = %s", matStr.c_str());

//            hullObject->ApplyMaterialList("Models/Vehicles/Offroad/Models/body-car.txt");

                // staticObject->SetMaterial(cache->GetResource<Material>("Materials/BROWN-DARK.xml"));
                //  staticObject->SetMaterial(cache->GetResource<Material>("Materials/GREEN.xml"));

//
/*
        auto* modelObject = modelNode->CreateComponent<AnimatedModel>();
        modelObject->SetModel(cache->GetResource<Model>("Models/Kachujin/Kachujin.mdl"));
        modelObject->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY-COLORS.xml"));
        modelObject->SetCastShadows(true);
*/



                //terrain-world-plain.mdl

//            staticObject->SetMaterial(cache->GetResource<Material>("Models/3dtile01.mtl"));

                //             tileMap->SetTmxFile(cache->GetResource<TmxFile2D>("Models/3dtile01.obj"));


/*
            staticSprite->SetSprite(tile->GetSprite());
            staticSprite->SetFlip(tile->GetFlipX(), tile->GetFlipY(), tile->GetSwapXY());
            staticSprite->SetLayer(drawOrder_);
            staticSprite->SetOrderInLayer(y * width + x);*/

                /*      Node* mushroomNode = scene_->CreateChild("Mushroom");
                      mushroomNode->SetPosition(Vector3(Random(90.0f) - 45.0f, 0.0f, Random(90.0f) - 45.0f));
                      mushroomNode->SetRotation(Quaternion(0.0f, Random(360.0f), 0.0f));
                      mushroomNode->SetScale(0.5f + Random(2.0f));*/
                nodes_[y * width + x] = tileNode;
            }
        }
    }
}

void TileMapLayer3D::SetObjectGroup(const TmxObjectGroup2D* objectGroup)
{
    objectGroup_ = objectGroup;

    TmxFile2D* tmxFile = objectGroup->GetTmxFile();
    nodes_.Resize(objectGroup->GetNumObjects());

    for (unsigned i = 0; i < objectGroup->GetNumObjects(); ++i)
    {
        const TileMapObject2D* object = objectGroup->GetObject(i);

        // Create dummy node for all object
        SharedPtr<Node> objectNode(GetNode()->CreateTemporaryChild("Object"));
        objectNode->SetPosition(Vector3(object->GetPosition()));

        // If object is tile, create static sprite component
        if (object->GetObjectType() == OT_TILE && object->GetTileGid() && object->GetTileSprite())
        {
            auto* staticSprite = objectNode->CreateComponent<StaticSprite3D>();
            staticSprite->SetSprite(object->GetTileSprite());
            staticSprite->SetFlip(object->GetTileFlipX(), object->GetTileFlipY(), object->GetTileSwapXY());
            staticSprite->SetLayer(drawOrder_);
            staticSprite->SetOrderInLayer((int)((10.0f - object->GetPosition().y_) * 100));

            if (tmxFile->GetInfo().orientation_ == O_ISOMETRIC)
            {
                staticSprite->SetUseHotSpot(true);
                staticSprite->SetHotSpot(Vector2(0.5f, 0.0f));
            }
        }

        nodes_[i] = objectNode;
    }
}

void TileMapLayer3D::SetImageLayer(const TmxImageLayer2D* imageLayer)
{
    imageLayer_ = imageLayer;

    if (!imageLayer->GetSprite())
        return;

    SharedPtr<Node> imageNode(GetNode()->CreateTemporaryChild("Tile"));
    imageNode->SetPosition(Vector3(imageLayer->GetPosition()));

    auto* staticSprite = imageNode->CreateComponent<StaticSprite3D>();
    staticSprite->SetSprite(imageLayer->GetSprite());
    staticSprite->SetOrderInLayer(0);

    nodes_.Push(imageNode);
}

}
