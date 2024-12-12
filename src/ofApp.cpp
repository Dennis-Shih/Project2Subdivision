
/*
 Dennis Shih
 Final proj
 due: 12/12/2024
 */


#include "ofApp.h"
#include "Util.h"


//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){
    if (bgImg.load("img/background.jpg")){
        cout << "background image loaded" <<endl;
        bg.set(1000,64);
        bg.mapTexCoordsFromTexture(bgImg.getTexture());
    }
    
    bWireframe = false;
    bDisplayPoints = false;
    bAltKeyDown = false;
    bCtrlKeyDown = false;
    bLanderLoaded = false;
    bTerrainSelected = true;
    //	ofSetWindowShape(1024, 768);
    cam.setDistance(10);
    cam.setNearClip(.1);
    cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
    ofSetVerticalSync(true);
    cam.disableMouseInput();
    ofEnableSmoothing();
    ofEnableDepthTest();
    
    // setup rudimentary lighting
    //
    initLightingAndMaterials();
    //moon-houdini.obj
    if (!mars.load("geo/moon-houdini.obj")){
        cout << "Failed to load terrain" << endl;
    }
    mars.setScaleNormalization(false);
    
    //load spacecraft
    if (!lander.load("geo/LEM-combined.obj")){
        cout << "Failed to load lander" << endl;
    } else {
        bLanderLoaded=true;
    }
    speed = 10;
    //rotSpeed = 0.5;
    mass=2.0;
    damping= 0.99;
    velocity = ofVec3f(0,0,0);
    accel = ofVec3f(0,0,0);
    gravMag=1;
    restitution=1;
    lInitPos=ofVec3f(0,30,0);
    lander.setScaleNormalization(false);
    lander.setPosition(lInitPos.x, lInitPos.y, lInitPos.z);
    
    trackingCam.setPosition(-40,5,50);
    
    //trackingCam.setPosition(0,lander.getPosition().y,50);
    onboardCam.rotateDeg(-90, 1, 0, 0);
    camView=1;
    
    
    ofDisableArbTex();
    if (!ofLoadImage(particleTex, "img/dot.png")) {
        cout << "Particle Texture File: img/dot.png not found" << endl;
        ofExit();
    }

    #ifdef TARGET_OPENGLES
        shader.load("shaders_gles/shader");
    #else
        shader.load("shaders/shader");
    #endif
    
    gui.setup();
    gui.add(numLevels.setup("Number of Octree Levels", 1, 1, 10));
    gui.add(timeInfo.setup("Timing info", false));
    bHide = false;
    
    
    ofResetElapsedTimeCounter();
    octree.create(mars.getMesh(0), 20);
    
    cout << "tree build time (ms): "<<ofGetElapsedTimeMillis()<<endl;
    cout << "Number of Verts: " << mars.getMesh(0).getNumVertices() << endl;
    
    lz.setup();
    
    landingLight.setup();
    landingLight.setSpotlight();
    landingLight.setSpotlightCutOff(30);
    landingLight.rotateDeg(-90, 1, 0, 0);
    landingLight.setAmbientColor(ofFloatColor(100, 100, 1));
    landingLight.setDiffuseColor(ofFloatColor(30, 30, 1));
    landingLight.setSpecularColor(ofFloatColor(1, 1, 1));
    
    tem.setEmitterType(DiscEmitter);
    tem.sys->addForce(new ThrusterForce(velocity));
    tem.setOneShot(true);
    tem.setGroupSize(20);
    
    radialForce = new ImpulseRadialForce(10);
    radialForce->setHeight(100);
    explEm.setEmitterType(RadialEmitter);
    explEm.sys->addForce(radialForce);
    explEm.setGroupSize(500);
    explEm.setVelocity(ofVec3f(0, 10, 0));
    explEm.setParticleRadius(100);
    explEm.setOneShot(true);
    
    if (thrust.load("audio/thrusters-loop.wav")){
        cout << "thrust sound loaded" << endl;
    }
    
    if (explosion.load("audio/explosion.wav")){
        cout << "explosion sound loaded" << endl;
    }

    //thrust.setLoop(true);
    //ofSoundStreamSetup(2, 0, 44100, 4000, 8);
    forceX =ofVec3f(1,0,0)* moveXDir* speed;
    forceY =  ofVec3f(0,1,0)* moveYDir* speed;
    forceZ =  ofVec3f(0,0,1)* moveZDir* speed;
    forceGrav =  ofVec3f(0,-1,0) * gravMag;
    //turbulence
    tmin=ofVec3f(-2, -2, -2);
    tmax=ofVec3f(2, 2, 2);
    
    landerForces.push_back(forceX);
    landerForces.push_back(forceY);
    landerForces.push_back(forceZ);
    landerForces.push_back(forceGrav);
    landerForces.push_back(forceImp);
    
    //ParticleSystem *sys = explEm.sys;
    tPartRadius=15;
    fuelLimit=120;
    
    isGameLost=false;
    isGameWon=false;
    
    
    
}

void ofApp::loadVbo() {
    if (tem.sys->particles.size() < 1) return;

    vector<ofVec3f> sizes;
    vector<ofVec3f> points;
    
   
    for (int i = 0; i < tem.sys->particles.size(); i++) {
        points.push_back(tem.sys->particles[i].position);
        sizes.push_back(ofVec3f(tPartRadius));
    }
    
    // upload the data to the vbo
    //
    int total = (int)points.size();
    
    vbo.clear();
    vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
    vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}

void ofApp::loadVboExpl() {
    if (explEm.sys->particles.size() < 1) return;

    vector<ofVec3f> sizes;
    vector<ofVec3f> points;
    
   
    for (int i = 0; i < explEm.sys->particles.size(); i++) {
        points.push_back(explEm.sys->particles[i].position);
        sizes.push_back(ofVec3f(30));
    }
    
    // upload the data to the vbo
    //
    int total = (int)points.size();
    
    vboExpl.clear();
    vboExpl.setVertexData(&points[0], total, GL_STATIC_DRAW);
    vboExpl.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}

/*
 Dennis Shih
 */
void ofApp::integrate(){
    float framerate = ofGetFrameRate();
    if (framerate < 1.0) return;

    float dt = 1.0 / framerate;
    
    
    glm::vec3 landerPos = lander.getPosition();
    float dx=velocity.x * dt;
    float dy=velocity.y * dt;
    float dz=velocity.z * dt;
    lander.setPosition(landerPos.x+dx, landerPos.y+dy, landerPos.z+dz);
    accel=(1/mass)*(forceX+forceY+forceZ+forceGrav);
    
    velocity=velocity +accel*dt;
    velocity *=damping;
    
    rotSpeed+=rotDir*dt;
    
    lander.setRotation(0, lander.getRotationAngle(0) + rotSpeed, 0, 1, 0);
    onboardCam.rotateDeg(rotSpeed, 0, 1, 0);
    rotSpeed*=damping;
    
}

void ofApp::update() {
    explEm.update();
    if (!isGameRunning && !isGameLost) return;
    /*
     if (!isGameRunning) return;
    if (isGameWon || isGameLost) {
        camView=1;
        isGameRunning=false;
        ofClear(0, 0, 0);
        return;
    }
    */
    //ship physics
    tem.setPosition(lander.getPosition());
    forceX =ofVec3f(1,0,0)* moveXDir* speed + ofRandom(tmin.x, tmax.x);
    forceY =  ofVec3f(0,1,0)* moveYDir* speed + ofRandom(tmin.y, tmax.y);
    forceZ =  ofVec3f(0,0,1)* moveZDir* speed + ofRandom(tmin.z, tmax.z);;
    forceGrav=ofVec3f(0,-1,0)*gravMag * mass;
    
  
    
    integrate();
    if (!isGameRunning) return;
    if (isGameWon || isGameLost) {
        camView=1;
        isGameRunning=false;
        ofClear(0, 0, 0);
        return;
    }
    //check if lander intersect octree boxes
    ofVec3f min = lander.getSceneMin() + lander.getPosition();
    ofVec3f max = lander.getSceneMax() + lander.getPosition();
    
    Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
    
    
    colBoxList.clear();
    //terain collision
    if (octree.intersect(bounds, octree.root, colBoxList)){
        //cout << "intersect" << endl;
        checkCollisions();
    //landing zone collision
    } else if (lz.overlap(bounds)){
        checkCollisions();
        if (isGameLost) return;
        //float d = glm::length(max - lz.pos);
        cout <<lander.getPosition().y << endl;
        cout << lz.height<<endl;
        if (lander.getPosition().y>=lz.pos.y+lz.height/2){
            lz.lState=LANDED;
            isGameWon=true;
        }
    } else if (abs(min.x- lz.pos.x) <=lz.radius*2 && abs(min.y- (lz.pos.y + lz.height)) <=lz.radius*4 && abs(min.z- lz.pos.z) <=lz.radius*2){
        
        lz.lState=APPROACH;
    } else lz.lState=FAR;
        
    if (showAgl) agl=getAgl(min);
    //cout<<"lz contact:" << lz.overlap(bounds)<<endl;
    onboardCam.setPosition(ofVec3f(max.x, max.y+3.25, max.z));
    tem.update();
    
    //fuel
    if (isShipThrusting){
        tFuelUsed+= ofGetLastFrameTime();
    }
    landingLight.setPosition(lander.getPosition());
    lz.update();
}
/*Ray collision for AGL
 */
float ofApp::getAgl(ofVec3f p0){
    
     //glm::vec3 landerPos = lander.getPosition();
     glm::vec3 landerPos = glm::vec3(p0.x,p0.y,p0.z);
     glm::vec3 dir = glm::vec3(0,-1,0);
     
     //Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
     
     Ray agl=Ray(Vector3(landerPos.x, landerPos.y, landerPos.z),
     Vector3(dir.x, dir.y, dir.z));
     
     bool aboveGround=octree.intersect(agl, octree.root, selectedNode);
     if (aboveGround){
         ofVec3f p = octree.mesh.getVertex(selectedNode.points[0]);
         //display on screen
         /*lander terrain agl dist:
          */
         return glm::length(landerPos - p);
         
     } else {
         return -1;
     }
     
     
}

/*
 Func to apply impulse force if ship collides terrain, checks if ship collides too fast
 */
void ofApp::checkCollisions() {
    glm::vec3 v=glm::vec3(velocity.x,velocity.y,velocity.z);
    if (abs(glm::length(v))>7) {
        
        explEm.setPosition(lander.getPosition());
        explEm.start();
        explosion.play();
        isGameLost=true;
        rotSpeed=30;
        restitution=10;
        
    }
    
    ofVec3f norm = ofVec3f(0, 1, 0);
    ofVec3f f = (restitution + 1.0)*((-velocity.dot(norm))*norm);
    forceImp=f;
    velocity += forceImp;
    
    
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    ofBackground(ofColor::black);
    
    
    switch (camView){
        case 1:
            cam.begin();
            break;
        case 2:
            trackingCam.begin();
            trackingCam.lookAt(lander.getPosition());
            break;
        case 3:
            onboardCam.begin();
            break;
        default:
            cam.begin();
            break;
    }
    //cam.begin();
    ofPushMatrix();
    ofSetColor(ofColor::white);
    bgImg.getTexture().bind();
    bg.draw();
    bgImg.getTexture().unbind();
    if (bWireframe) {                    // wireframe mode  (include axis)
        ofDisableLighting();
        ofSetColor(ofColor::slateGray);
        mars.drawWireframe();
        if (bLanderLoaded) {
            lander.drawWireframe();
            if (!bTerrainSelected) drawAxis(lander.getPosition());
        }
        if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
    }
    else {
        ofEnableLighting();              // shaded mode
        mars.drawFaces();
        ofMesh mesh;
        if (bLanderLoaded) {
            lander.drawFaces();
            //drawAxis(lander.getPosition());
            
            
            if (bLanderSelected) {
                
                ofVec3f min = lander.getSceneMin() + lander.getPosition();
                ofVec3f max = lander.getSceneMax() + lander.getPosition();
                
                Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
                ofSetColor(ofColor::white);
                Octree::drawBox(bounds);
                
                // draw colliding boxes
                //
                ofSetColor(ofColor::lightBlue);
                for (int i = 0; i < colBoxList.size(); i++) {
                    Octree::drawBox(colBoxList[i]);
                }
            }
        }
    }
    if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
    
    
    
    if (bDisplayPoints) {                // display points as an option
        glPointSize(3);
        ofSetColor(ofColor::green);
        mars.drawVertices();
    }
    
    
    // recursively draw octree
    //
    ofDisableLighting();
    int level = 0;
    //	ofNoFill();
    
    if (bDisplayLeafNodes) {
        octree.drawLeafNodes(octree.root);
        cout << "num leaf: " << octree.numLeaf << endl;
    }
    else if (bDisplayOctree) {
        ofNoFill();
        ofSetColor(ofColor::white);
        octree.draw(numLevels, 0);
    }
    //draw LZ
    ofSetColor(ofColor::white);
    lz.draw();
    ofPopMatrix();
    
    switch (camView){
        case 1:
            cam.end();
            break;
        case 2:
            trackingCam.end();
            break;
        case 3:
            onboardCam.end();
            break;
        default:
            cam.end();
            break;
    }
    
    loadVbo();
    loadVboExpl();
    glDepthMask(GL_FALSE);
    ofSetColor(255, 100, 90);
    
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofEnablePointSprites();
    shader.begin();
    switch (camView){
        case 1:
            cam.begin();
            break;
        case 2:
            trackingCam.begin();
            trackingCam.lookAt(lander.getPosition());
            break;
        case 3:
            onboardCam.begin();
            break;
        default:
            cam.begin();
            break;
    }
    
    
    particleTex.bind();
    vbo.draw(GL_POINTS, 0, (int)tem.sys->particles.size());
    vboExpl.draw(GL_POINTS, 0, (int)explEm.sys->particles.size());
    //vbo.draw(GL_POINTS, 0, (int)explEm.sys->particles.size());
    particleTex.unbind();
    
    switch (camView){
        case 1:
            cam.end();
            break;
        case 2:
            trackingCam.end();
            break;
        case 3:
            onboardCam.end();
            break;
        default:
            cam.end();
            break;
    }
    shader.end();
    
    
    ofDisablePointSprites();
    ofDisableBlendMode();
    ofEnableAlphaBlending();
    glDepthMask(GL_TRUE);
    //cam.end();
    glDepthMask(false);
    if (!bHide) gui.draw();
    glDepthMask(true);
    
    ofDisableLighting();
    
    
    if (!isGameRunning) {
        if (isGameLost){
            string gameOver="GAME OVER\n Press space to restart";
            ofDrawBitmapString(gameOver, ofGetWindowWidth()/2, 200);
        } else if (isGameWon){
            string gameWon="Landing successful\n Press space to restart";
            ofDrawBitmapString(gameWon, ofGetWindowWidth()/2, 200);
        } else {
            string space="Press space to start";
            ofDrawBitmapString(space, ofGetWindowWidth()/2, 200);
        }
        string instructions = "Instructions: \n Arrow Keys to fly horizontally"
        "\n Tab/Shift to increase/decrease altitude\n g/h keys to rotate vehicle \n"
        "t to toggle spacecraft light\n "
        "Number keys for cameras: \n 1: Default easyCam\n"
        " 2: tracking cam\n 3: Onboard cam\n "
        "X key to retarget easyCam on spacecraft\n Q to quit";
        ofDrawBitmapString(instructions, ofGetWindowWidth()/2, 40);
    } else {
        
        string str;
        str += "Frame Rate: " + std::to_string(ofGetFrameRate());
        ofSetColor(ofColor::white);
        ofDrawBitmapString(str, ofGetWindowWidth() -200, 15);
        float f=fuelLimit-tFuelUsed;
        if (f <0) f = 0;
        string fuel = "Fuel remaining: " + ofToString(f) + "s";
        ofDrawBitmapString(fuel, ofGetWindowWidth() -200, 30);
        if (showAgl){
            string aglStr;
            aglStr += "AGL: " + std::to_string(agl) + "m";
            
            ofDrawBitmapString(aglStr, ofGetWindowWidth() -200, 45);
        }
        float v=velocity.length();
        string velocity = "Velocity: " +std::to_string(v)+ "m/s";
        ofDrawBitmapString(velocity, ofGetWindowWidth() -200, 60);
    }
}


//
// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {
    
    ofPushMatrix();
    ofTranslate(location);
    
    ofSetLineWidth(1.0);
    
    // X Axis
    ofSetColor(ofColor(255, 0, 0));
    ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
    
    
    // Y Axis
    ofSetColor(ofColor(0, 255, 0));
    ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));
    
    // Z Axis
    ofSetColor(ofColor(0, 0, 255));
    ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));
    
    ofPopMatrix();
}

void ofApp::resetGame(){
    ofResetElapsedTimeCounter();
    ofClear(0, 0, 0);
    tem.sys->reset();
    explEm.sys->reset();
    explEm.sys->removeAll();
    lander.setPosition(lInitPos.x, lInitPos.y, lInitPos.z);
    
    velocity=ofVec3f(0);
    accel=ofVec3f(0);
    rotSpeed=0;
    tFuelUsed=0;
    lz.lState=FAR;
    isGameWon=false;
    isGameLost=false;
    //setup();
}


void ofApp::keyPressed(int key) {
    
    switch (key) {
        //toggle agl calculation
        case 'A':
        case 'a':
            showAgl=!showAgl;
            break;
        case 'B':
        case 'b':
            bDisplayBBoxes = !bDisplayBBoxes;
            break;
        case 'C':
        case 'c':
            if (camView==1){
                if (cam.getMouseInputEnabled()) cam.disableMouseInput();
                else cam.enableMouseInput();
            }
            break;
        case 'F':
        case 'f':
            ofToggleFullscreen();
            break;
        
        case 'L':
        case 'l':
            bDisplayLeafNodes = !bDisplayLeafNodes;
            break;
        case 'O':
        case 'o':
            bDisplayOctree = !bDisplayOctree;
            break;
        case 'r':
            if (camView==1) cam.reset();
            break;
        case 's':
            savePicture();
            break;
        case 'x':
            if (camView==1) setCameraTarget();
            break;
        /*
         Camera views:
         1 - ofEasyCam
         2 - tracking camera
         3 - onboard camera
         4 - top-down camera (maybe moveable)
         */
        case '1':
            camView=1;
            break;
        case '2':
            camView=2;
            break;
        case '3':
            camView=3;
            break;
        case '4':
            camView=4;
            break;
        case 'v':
            togglePointsDisplay();
            break;
        case 'w':
            toggleWireframeMode();
            break;
        case OF_KEY_ALT:
            cam.enableMouseInput();
            bAltKeyDown = true;
            break;
        case OF_KEY_CONTROL:
            bCtrlKeyDown = true;
            
            break;
        /* Keys for ship movement:
         Tab/Shift to go up/down
         Arrow keys to move for/back/left/right
         h and g to rotate
         */
        case OF_KEY_SHIFT:
            if (fuelLimit>=tFuelUsed){
                isShipThrusting=true;
                if (!thrust.isPlaying()) {
                    thrust.play();
                }
                moveYDir=-1;
            }
            break;
        case OF_KEY_TAB:
            if (fuelLimit>=tFuelUsed){
                tem.start();
                
                isShipThrusting=true;
                if (!thrust.isPlaying()) {
                    thrust.play();
                }
                moveYDir=1;
            }
            break;
        case OF_KEY_UP:
            if (fuelLimit>=tFuelUsed){
                tem.start();
                
                isShipThrusting=true;
                if (!thrust.isPlaying()) {
                    thrust.play();
                }
                moveZDir = -1;
            }
            break;
        case OF_KEY_DOWN:
            if (fuelLimit>=tFuelUsed){
                tem.start();
                isShipThrusting=true;
                if (!thrust.isPlaying()) {
                    thrust.play();
                }
                moveZDir = 1;
            }
            break;
        case OF_KEY_LEFT:
            if (fuelLimit>=tFuelUsed){
                tem.start();
                isShipThrusting=true;
                if (!thrust.isPlaying()) {
                    thrust.play();
                }
                moveXDir = -1;
            }
            break;
        case OF_KEY_RIGHT:
            if (fuelLimit>=tFuelUsed){
                tem.start();
                isShipThrusting=true;
                if (!thrust.isPlaying()) {
                    thrust.play();
                }
                moveXDir = 1;
            }
            //player.rot+=pRotationSpeed;
            break;
        case 'H':
        case 'h':
            //isShipThrusting=true;
            rotDir=1;
            break;
        case 'g':
            //isShipThrusting=true;
            rotDir=-1;
            break;
        case 'j':
            
            break;
        case 't':
            
            if (landingLight.getIsEnabled()) landingLight.disable();
            else landingLight.enable();
            break;
        case 'q':
            
            exit();
            break;
        case ' ':
            if (!isGameRunning){
                resetGame();
            }
            isGameRunning=true;
            
           
            break;
        case OF_KEY_DEL:
            break;
        default:
            break;
    }
}

void ofApp::toggleWireframeMode() {
    bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
    bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
    bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {
    
    switch (key) {
            
        case OF_KEY_ALT:
            cam.disableMouseInput();
            bAltKeyDown = false;
            break;
        case OF_KEY_CONTROL:
            bCtrlKeyDown = false;
            break;
        case OF_KEY_SHIFT:
        case OF_KEY_TAB:
            isShipThrusting=false;
            moveYDir=0;
            thrust.stop();
            break;
        case OF_KEY_UP:
            /*
            isShipThrusting=false;
            moveZDir = 0;
            break;
            */
        case OF_KEY_DOWN:
            isShipThrusting=false;
            moveZDir = 0;
            break;
        case OF_KEY_LEFT:
            /*
            isShipThrusting=false;
            moveXDir = 0;
            break;
             */
        case OF_KEY_RIGHT:
            isShipThrusting=false;
            moveXDir = 0;
            break;
        case 'h':
            /*
            isShipThrusting=false;
            rotDir=0;
            break;
             */
        case 'g':
            isShipThrusting=false;
            rotDir=0;
            break;
        default:
            break;
            
    }
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
    
}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    
    // if moving camera, don't allow mouse interaction
    //
    if (cam.getMouseInputEnabled()) return;
    
    
    // if rover is loaded, test for selection
    //
    if (bLanderLoaded) {
        glm::vec3 origin = cam.getPosition();
        glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
        glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
        
        ofVec3f min = lander.getSceneMin() + lander.getPosition();
        ofVec3f max = lander.getSceneMax() + lander.getPosition();
        
        Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
        bool hit = bounds.intersect(Ray(Vector3(origin.x, origin.y, origin.z), Vector3(mouseDir.x, mouseDir.y, mouseDir.z)), 0, 10000);
        if (hit) {
            bLanderSelected = true;
            mouseDownPos = getMousePointOnPlane(lander.getPosition(), cam.getZAxis());
            mouseLastPos = mouseDownPos;
            bInDrag = true;
        }
        else {
            bLanderSelected = false;
        }
    }
    else {
        /*
        ofVec3f p;
        raySelectWithOctree(p);
        */
    }
}
/*
bool ofApp::raySelectWithOctree(ofVec3f &pointRet) {
    ofVec3f mouse(mouseX, mouseY);
    ofVec3f rayPoint = cam.screenToWorld(mouse);
    ofVec3f rayDir = rayPoint - cam.getPosition();
    rayDir.normalize();
    Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
                  Vector3(rayDir.x, rayDir.y, rayDir.z));
    
    ofResetElapsedTimeCounter();
    pointSelected = octree.intersect(ray, octree.root, selectedNode);
    
    if (timeInfo){
        cout << "ray search time (ms): "<<ofGetElapsedTimeMillis()<<endl;
    }
    
    
    if (pointSelected) {
        pointRet = octree.mesh.getVertex(selectedNode.points[0]);
        
    }
    return pointSelected;
}
*/



//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
    
    // if moving camera, don't allow mouse interaction
    //
    if (cam.getMouseInputEnabled()) return;
    
    if (bInDrag) {
        
        glm::vec3 landerPos = lander.getPosition();
        
        glm::vec3 mousePos = getMousePointOnPlane(landerPos, cam.getZAxis());
        glm::vec3 delta = mousePos - mouseLastPos;
        
        landerPos += delta;
        lander.setPosition(landerPos.x, landerPos.y, landerPos.z);
        mouseLastPos = mousePos;
        
        ofVec3f min = lander.getSceneMin() + lander.getPosition();
        ofVec3f max = lander.getSceneMax() + lander.getPosition();
        
        Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
        
        colBoxList.clear();
        //check if lander intersect octree boxes
        if (octree.intersect(bounds, octree.root, colBoxList)){
            cout << "intersect" << endl;
        } else cout << "no intersect" << endl;
        
       
        
    }
    else {
        /*
        ofVec3f p;
        raySelectWithOctree(p);
        */
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
    bInDrag = false;
    pointSelected=false;
}



// Set the camera to use the selected point as it's new target
//
void ofApp::setCameraTarget() {
    cam.lookAt(lander.getPosition());
}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {
    
    static float ambient[] =
    { .5f, .5f, .5, 1.0f };
    static float diffuse[] =
    { 1.0f, 1.0f, 1.0f, 1.0f };
    
    static float position[] =
    {5.0, 5.0, 5.0, 0.0 };
    
    static float lmodel_ambient[] =
    { 1.0f, 1.0f, 1.0f, 1.0f };
    
    static float lmodel_twoside[] =
    { GL_TRUE };
    
    
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, position);
    
    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    //	glEnable(GL_LIGHT1);
    glShadeModel(GL_SMOOTH);
}

void ofApp::savePicture() {
    ofImage picture;
    picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
    picture.save("screenshot.png");
    cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent2(ofDragInfo dragInfo) {
    
    ofVec3f point;
    mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);
    if (lander.load(dragInfo.files[0])) {
        lander.setScaleNormalization(false);
        //		lander.setScale(.1, .1, .1);
        //	lander.setPosition(point.x, point.y, point.z);
        lander.setPosition(1, 1, 0);
        
        bLanderLoaded = true;
        for (int i = 0; i < lander.getMeshCount(); i++) {
            bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
        }
        
        cout << "Mesh Count: " << lander.getMeshCount() << endl;
    }
    else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
    ofVec2f mouse(mouseX, mouseY);
    ofVec3f rayPoint = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
    ofVec3f rayDir = rayPoint - cam.getPosition();
    rayDir.normalize();
    return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {
    if (lander.load(dragInfo.files[0])) {
        bLanderLoaded = true;
        lander.setScaleNormalization(false);
        lander.setPosition(0, 0, 0);
        cout << "number of meshes: " << lander.getNumMeshes() << endl;
        bboxList.clear();
        for (int i = 0; i < lander.getMeshCount(); i++) {
            bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
        }
        
        //		lander.setRotation(1, 180, 1, 0, 0);
        
        // We want to drag and drop a 3D object in space so that the model appears
        // under the mouse pointer where you drop it !
        //
        // Our strategy: intersect a plane parallel to the camera plane where the mouse drops the model
        // once we find the point of intersection, we can position the lander/lander
        // at that location.
        //
        
        // Setup our rays
        //
        glm::vec3 origin = cam.getPosition();
        glm::vec3 camAxis = cam.getZAxis();
        glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
        glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
        float distance;
        
        bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);
        if (hit) {
            // find the point of intersection on the plane using the distance
            // We use the parameteric line or vector representation of a line to compute
            //
            // p' = p + s * dir;
            //
            glm::vec3 intersectPoint = origin + distance * mouseDir;
            
            // Now position the lander's origin at that intersection point
            //
            glm::vec3 min = lander.getSceneMin();
            glm::vec3 max = lander.getSceneMax();
            float offset = (max.y - min.y) / 2.0;
            lander.setPosition(intersectPoint.x, intersectPoint.y - offset, intersectPoint.z);
            
            // set up bounding box for lander while we are at it
            //
            landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
        }
    }
    
    
}

//  intersect the mouse ray with the plane normal to the camera
//  return intersection point.   (package code above into function)
//
glm::vec3 ofApp::getMousePointOnPlane(glm::vec3 planePt, glm::vec3 planeNorm) {
    // Setup our rays
    //
    glm::vec3 origin = cam.getPosition();
    glm::vec3 camAxis = cam.getZAxis();
    glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
    glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
    float distance;
    
    bool hit = glm::intersectRayPlane(origin, mouseDir, planePt, planeNorm, distance);
    
    if (hit) {
        // find the point of intersection on the plane using the distance
        // We use the parameteric line or vector representation of a line to compute
        //
        // p' = p + s * dir;
        //
        glm::vec3 intersectPoint = origin + distance * mouseDir;
        
        return intersectPoint;
    }
    else return glm::vec3(0, 0, 0);
}

void ofApp::exit(){
    thrust.stop();
    ofExit();
}
