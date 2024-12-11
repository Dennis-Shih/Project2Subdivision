
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
    lander.setScaleNormalization(false);
    lander.setPosition(0, 30, 0);
    
    trackingCam.setPosition(-40,5,50);
    
    //trackingCam.setPosition(0,lander.getPosition().y,50);
    onboardCam.rotate(-90, 1, 0, 0);
    camView=1;
    
    
    
    gui.setup();
    gui.add(numLevels.setup("Number of Octree Levels", 1, 1, 10));
    gui.add(timeInfo.setup("Timing info", false));
    bHide = false;
    
    
    ofResetElapsedTimeCounter();
    octree.create(mars.getMesh(0), 20);
    
    cout << "tree build time (ms): "<<ofGetElapsedTimeMillis()<<endl;
    cout << "Number of Verts: " << mars.getMesh(0).getNumVertices() << endl;
    
    lz.setup();
    
    if (thrust.load("audio/thrusters-loop.wav")){
        cout << "thrust sound loaded" << endl;
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
    
    ParticleSystem *sys = explEm.sys;
    
    //sys->addForce(new TurbulenceForce(ofVec3f(-3, -1, -1), ofVec3f(3, 1, 1)));
    
    
    
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
    
    rotSpeed*=damping;
    
}

void ofApp::update() {
    //ship physics
    
    forceX =ofVec3f(1,0,0)* moveXDir* speed + ofRandom(tmin.x, tmax.x);
    forceY =  ofVec3f(0,1,0)* moveYDir* speed + ofRandom(tmin.y, tmax.y);
    forceZ =  ofVec3f(0,0,1)* moveZDir* speed + ofRandom(tmin.z, tmax.z);;
    forceGrav=ofVec3f(0,-1,0)*gravMag * mass;
    
  
    
    integrate();
    
    //check if lander intersect octree boxes
    ofVec3f min = lander.getSceneMin() + lander.getPosition();
    ofVec3f max = lander.getSceneMax() + lander.getPosition();
    
    Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
    colBoxList.clear();
    if (octree.intersect(bounds, octree.root, colBoxList)){
        //cout << "intersect" << endl;
        checkCollisions();
        
    } else if (lz.overlap(bounds)){
        //win
        
    } else if (showAgl) agl=getAgl(min);
    cout<<"lz contact:" << lz.overlap(bounds)<<endl;
    onboardCam.setPosition(ofVec3f(max.x, max.y+3.25, max.z));
    

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
    
    if (velocity.y>=30) {
        //explode or smth
    }
    
    ofVec3f norm = ofVec3f(0, 1, 0);
    ofVec3f f = (restitution + 1.0)*((-velocity.dot(norm))*norm);
    forceImp=f;
    velocity += forceImp;
    cout << forceImp << endl;
    cout << velocity << endl;
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    ofBackground(ofColor::black);
    glDepthMask(false);
    if (!bHide) gui.draw();
    glDepthMask(true);
    
    switch (camView){
        case 1:
            cam.begin();
            break;
        case 2:
            trackingCam.begin();
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
    
    // highlight selected point (draw sphere around selected point)
    //
    /*if (bPointSelected) {
        ofSetColor(ofColor::blue);
        ofDrawSphere(selectedPoint, .1);
    }*/
    
    
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
    
    //cam.end();
    
    string str;
    str += "Frame Rate: " + std::to_string(ofGetFrameRate());
    if (showAgl){
        string aglStr;
        aglStr += "AGL: " + std::to_string(agl) + "m";
        ofSetColor(ofColor::white);
        ofDrawBitmapString(str, ofGetWindowWidth() -170, 15);
        ofDrawBitmapString(aglStr, ofGetWindowWidth() -170, 30);
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
        case 't':
            setCameraTarget();
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
            isShipThrusting=true;
            if (!thrust.isPlaying()) {
                thrust.play();
            }
            moveYDir=-1;
            break;
        case OF_KEY_TAB:
            isShipThrusting=true;
            if (!thrust.isPlaying()) {
                thrust.play();
            }
            moveYDir=1;
            break;
        case OF_KEY_UP:
            isShipThrusting=true;
            moveZDir = -1;
            break;
        case OF_KEY_DOWN:
            isShipThrusting=true;
            moveZDir = 1;
            
            break;
        case OF_KEY_LEFT:
            isShipThrusting=true;
            moveXDir = -1;
            break;
        case OF_KEY_RIGHT:
            isShipThrusting=true;
            moveXDir = 1;
            //player.rot+=pRotationSpeed;
            break;
        case 'H':
        case 'h':
            isShipThrusting=true;
            rotDir=1;
            break;
        case 'g':
            isShipThrusting=true;
            rotDir=-1;
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
            isShipThrusting=false;
            moveZDir = 0;
            break;
        case OF_KEY_DOWN:
            isShipThrusting=false;
            moveZDir = 0;
            break;
        case OF_KEY_LEFT:
            isShipThrusting=false;
            moveXDir = 0;
            break;
        case OF_KEY_RIGHT:
            isShipThrusting=false;
            moveXDir = 0;
            break;
        case 'h':
            isShipThrusting=false;
            rotDir=0;
            break;
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
