#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxAssimpModelLoader.h"
#include "Octree.h"
#include "LandingZone.h"
#include <glm/gtx/intersect.hpp>
#include "ParticleEmitter.h"/*
 Dennis Shih
 Final proj
 12/12/2024
 */

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent2(ofDragInfo dragInfo);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);
		bool raySelectWithOctree(ofVec3f &pointRet);
		glm::vec3 getMousePointOnPlane(glm::vec3 p , glm::vec3 n);
        
       
		ofEasyCam cam;
        
    
		ofxAssimpModelLoader mars, lander;
		ofLight light;
		Box boundingBox, landerBounds;
        LandingZone lz;

		vector<Box> colBoxList;
		bool bLanderSelected = false;
		Octree octree;
		TreeNode selectedNode;
		glm::vec3 mouseDownPos, mouseLastPos;
		bool bInDrag = false;


		ofxIntSlider numLevels;
        ofxToggle timeInfo;
		ofxPanel gui;

		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
		bool bHide;
		bool pointSelected = false;
		bool bDisplayLeafNodes = false;
		bool bDisplayOctree = false;
		bool bDisplayBBoxes = false;
		
		bool bLanderLoaded;
		bool bTerrainSelected;
    
        
        bool showAgl;
        bool isShipThrusting;
        float moveXDir;
        float moveYDir;
        float moveZDir;
        float rotDir;
        float speed;
        float rotSpeed;
    
        float mass;
        float damping;
        float gravMag;
        float restitution;
    
        
        int camView;
        ofCamera trackingCam;
        ofCamera onboardCam;
    
        ofVec3f forceX;
        ofVec3f forceY;
        ofVec3f forceZ;
        ofVec3f forceGrav;
        ofVec3f forceImp;
        //ofVec3f forceTurb;
        ofVec3f tmin;
        ofVec3f tmax;
    
        ofVec3f accel;
        ofVec3f forward;
        ofVec3f velocity;
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;
        
        
    
        ofSoundPlayer thrust;
        ofSoundPlayer explosion;
        ofImage bgImg;
        ofSpherePrimitive bg;
        //thrust and explosion particle emitters
        ParticleEmitter tem;
        ParticleEmitter explEm;
		vector<Box> bboxList;
        vector<ofVec3f> landerForces;
    
        

        
        void checkCollisions();
        void integrate();
        float getAgl(ofVec3f p0);
        float agl;
        void exit();
        void loadVbo();
        void loadVboExpl();
        void resetGame();
        ofTexture particleTex;
        ofVbo vbo;
        ofVbo vboExpl;
        ofShader shader;
        ImpulseRadialForce *radialForce;
        float tPartRadius;
    
        float tFuelUsed;
        float fuelLimit;
        bool isGameRunning;
        bool isGameWon;
        bool isGameLost;
    
};

