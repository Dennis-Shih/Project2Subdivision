//
//  landingZone.h
//  Project2Subdivision
//
//  Created by Dennis Shih on 12/10/24.
//
#pragma once
#include "ofMain.h"
class LandingZone {
    public:
    LandingZone();
    void setup();
    void draw();
    bool overlap(Box &b);
    ofCylinderPrimitive c;
    ofImage lzTexture;
    ofVec3f pos;
    float radius;
    float height;
    
    
};
