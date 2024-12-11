//
//  landingZone.cpp
//  Project2Subdivision
//
//  Created by Dennis Shih on 12/10/24.
//
#include "Box.h"
#include "LandingZone.h"
LandingZone::LandingZone(){
    
    pos=ofVec3f(10,0,0);
    radius=5;
    height=5;
}

void LandingZone::setup(){
    LandingZone();
    if (lzTexture.load("img/lzTexture.png")){
        cout << "lz texture loaded" <<endl;
    }
    c.set(radius*1.1, height*1.95,8,2);
    c.setPosition(pos.x, pos.y, pos.z);
    c.mapTexCoordsFromTexture(lzTexture.getTexture());
    
}

void LandingZone::draw(){
    lzTexture.getTexture().bind();
    c.draw();
    lzTexture.getTexture().unbind();
    
    ofDrawLine(pos.x, pos.y, pos.z, pos.x+radius, pos.y, pos.z+radius);
}

bool LandingZone::overlap(Box &box) {
    bool overlapX = pos.x-radius <=box.parameters[1].x() && pos.x+radius >=box.parameters[0].x();
    bool overlapY = height <=box.parameters[1].y() && height >=box.parameters[0].y();
    bool overlapZ = pos.z-radius <=box.parameters[1].z() && pos.z+radius >=box.parameters[0].z();
    return overlapX && overlapY && overlapZ;
}
