//
// Created by nitro-kitty on 2020-08-19.
//
#pragma once

#include "Object.h"

EVENT(E_SERVER_CHATMESSAGE, ServerChatMessage)
{
PARAM(P_CHANNEL, Channel);  //int
PARAM(P_SENDER, Sender);  //string
PARAM(P_MESSAGE, Message);  //string
}

Urho3D::VariantMap map;
map[ServerChatMessage::P_CHANNEL] = channel;
map[ServerChatMessage:] = sender;
map[ServerChatMessage::P_MESSAGE] = message;
//connection->SendRemoteEvent(E_SERVER_CHATMESSAGE, true, map);
SendEvent(E_SERVER_CHATMESSAGE, map);