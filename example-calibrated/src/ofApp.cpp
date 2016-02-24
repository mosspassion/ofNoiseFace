#include "ofApp.h"

#include "ofMeshUtils.h"
#include "ofxPS3EyeGrabber.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
	ofSetVerticalSync(true);
    
    // ps3EyeCamera
    cout << "elapsed time: " << ofGetElapsedTimeMillis() << endl;
    ps3cam.setGrabber(std::make_shared<ofxPS3EyeGrabber>());
    ps3cam.setDesiredFrameRate(45);
    ps3cam.setup(640, 480);
    ps3cam.getGrabber<ofxPS3EyeGrabber>()->setAutogain(true);
    ps3cam.getGrabber<ofxPS3EyeGrabber>()->setAutoWhiteBalance(false);
    pixelsRGB.allocate(ps3cam.getWidth(), ps3cam.getHeight(), 3);
    
    // tracker
	tracker.setup();
	calibration.load("mbp-isight.yml");
	
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	light.setPosition(100, 100, 1000);
    
    // noise
    noise.getPixels().allocate(640, 480, 1);
    whiteBlack = 255;
    ofSetColor(255);
    swapBackground = false;
    bigBrother = true;
    
    //xmas
//    xmas.load("xmasPatternFrick.jpg");
}

void ofApp::updatePhysicalMesh() {
	// 1 load object and image points as Point2f/3f
	vector<Point3f> objectPoints;
	vector<Point2f> imagePoints;
	for(int i = 0; i < tracker.size(); i++) {
		objectPoints.push_back(toCv(tracker.getObjectPoint(i)));
		imagePoints.push_back(toCv(tracker.getImagePoint(i)));
	}
	
	// 2 guess for the rotation and translation of the face
	Mat cameraMatrix = calibration.getDistortedIntrinsics().getCameraMatrix();
	Mat distCoeffs = calibration.getDistCoeffs();
	Mat rvec, tvec;
	solvePnP(Mat(objectPoints),  Mat(imagePoints),
					 cameraMatrix,  distCoeffs,
					 rvec, tvec);
	
	// 3 reproject using guess, and fit to the actual image location
	vector<ofVec3f> fitWorldPoints;
	Mat cameraMatrixInv = cameraMatrix.inv();
	Mat rmat;
	Rodrigues(rvec, rmat);
	for(int i = 0; i < objectPoints.size(); i++) {
		Point2d imgImg = imagePoints[i];
		Point3d objObj = objectPoints[i];
		Point3d imgHom(imgImg.x, imgImg.y, 1.); // img->hom
		Point3d imgWor = (Point3f) Mat(cameraMatrixInv * Mat(imgHom)); // hom->wor
		Point3d objWor = (Point3d) Mat(tvec + rmat * Mat(objObj)); // obj->wor
		Point3d fitWor = intersectPointRay(objWor, imgWor); // scoot it over
		// if it was projected on the wrong side, flip it over
		if(fitWor.z < 0) {
			fitWor *= -1;
		}
		fitWorldPoints.push_back(toOf(fitWor));
		// convert down to image space coordinates
		//Point3d fitHom = (Point3d) Mat(cameraMatrix * Mat(fitWor)); // wor->hom
		//Point2d fitImg(fitHom.x / fitHom.z, fitHom.y / fitHom.z); // hom->img
	}
	
	// 4 use the resulting 3d points to build a mesh with normals
	physicalMesh = convertFromIndices(tracker.getMesh(fitWorldPoints));
	physicalMesh.setMode(OF_PRIMITIVE_TRIANGLES);
	buildNormals(physicalMesh);
}

void ofApp::update() {
	ps3cam.update();
	
    if(ps3cam.isFrameNew()) {
        tracker.update(toCv(ps3cam));
        
        // face tracker mesh
		if(tracker.getFound()) {
			updatePhysicalMesh();
		}
	}
    
    // image noise
    lineY = ofRandom(ofGetHeight());
    whiteBlack = ofRandom(256);
    
    for (int x = 0; x < noise.getWidth(); ++x)
    {
        for (int y = 0; y < noise.getHeight(); ++y)
        {
            int index = noise.getPixels().getPixelIndex(x, y);
            
            if (y != lineY)
            {
                noise.getPixels()[index] = ofRandom(256);
            }
            else
            {
                noise.getPixels()[index] = whiteBlack;
            }
        }
    }
    noise.update();
    
//    // swap time and reset time
//    if (ofGetMinutes() <= 5){
//        swapBackground = true;
//    }
//    else if (ofGetMinutes() > 5 && ofGetMinutes() <= 10){
//        swapBackground = false;
//    }
//    else if (ofGetMinutes() > 10 && ofGetMinutes() <= 15){
//        swapBackground = true;
//    }
//    else if (ofGetMinutes() > 15 && ofGetMinutes() <= 20){
//        swapBackground = false;
//    }
//    else if (ofGetMinutes() > 20 && ofGetMinutes() <= 25){
//        swapBackground = true;
//    }
//    else if (ofGetMinutes() > 25 && ofGetMinutes() <= 30){
//        swapBackground = false;
//    }
//    else if (ofGetMinutes() > 30 && ofGetMinutes() <= 35){
//        swapBackground = true;
//    }
//    else if (ofGetMinutes() > 35 && ofGetMinutes() <= 40){
//        swapBackground = false;
//    }
//    else if (ofGetMinutes() > 40 && ofGetMinutes() <= 45){
//        swapBackground = true;
//    }
//    else if (ofGetMinutes() > 45 && ofGetMinutes() <= 50){
//        swapBackground = false;
//    }
//    else if (ofGetMinutes() > 50 && ofGetMinutes() <= 55){
//        swapBackground = true;
//    }
//    else if (ofGetMinutes() > 55){
//        swapBackground = false;
//    }
    
    if (ofGetElapsedTimeMillis() >= 10000){
        tracker.reset();
        ofResetElapsedTimeCounter();
    }
    
    cout << "elapsed time: " << ofGetElapsedTimeMillis() << endl;
}

void ofApp::draw() {
    
    if (swapBackground == false){
        ps3cam.draw(0, 0);
        // frames per second displayed
        std::stringstream ss;
        //
        //    ss << " App FPS: " << ofGetFrameRate() << std::endl;
        //    ss << " Cam FPS: " << ps3cam.getGrabber<ofxPS3EyeGrabber>()->getFPS()  << std::endl;
        //    ss << "Real FPS: " << ps3cam.getGrabber<ofxPS3EyeGrabber>()->getActualFPS() << std::endl;
//        ss << "HAPPY HOLIDAYS!" << std::endl;
        ss << "#ofNoiseFace" << std::endl;
        ss << "#openFrameworks" << std::endl;
        ss << "@mosspassion"; // << std::endl;
//        ss << "twitch.tv/mosspassion";
//        
        ofDrawBitmapStringHighlight(ss.str(), ofPoint(10, 20));
        
        if (bigBrother == true){
        std::stringstream sss;
        sss << "I'm Not Your Big Brother";
        ofDrawBitmapStringHighlight(sss.str(), ofPoint((ofGetWidth()*0.5-100),(ofGetHeight()*0.5)));
        }
        else{}
//
    }
    else if (swapBackground == true){
        noise.draw(0, 0);
        // frames per second displayed
        std::stringstream ss;
        //
        //    ss << " App FPS: " << ofGetFrameRate() << std::endl;
        //    ss << " Cam FPS: " << ps3cam.getGrabber<ofxPS3EyeGrabber>()->getFPS()  << std::endl;
        //    ss << "Real FPS: " << ps3cam.getGrabber<ofxPS3EyeGrabber>()->getActualFPS() << std::endl;
//        ss << "HAPPY HOLIDAYS!" << std::endl;
        ss << "#ofNoiseFace" << std::endl;
        ss << "#openFrameworks" << std::endl;
        ss << "@mosspassion"; // << std::endl;
//        ss << "twitch.tv/mosspassion";
        
        ofDrawBitmapStringHighlight(ss.str(), ofPoint(10, 20));

        if (bigBrother == true){
        std::stringstream sss;
        sss << "I'm Not Your Big Brother";
        ofDrawBitmapStringHighlight(sss.str(), ofPoint((ofGetWidth()*0.5-100),(ofGetHeight()*0.5)));
        }
        else{}
        
    }
    
    // tracker
	if(tracker.getFound()) {
        ofMesh objectMesh = tracker.getObjectMesh();
        ofMesh meanMesh = tracker.getMeanObjectMesh();
        
        ofSetupScreenOrtho(640, 480, -1000, 1000);
        ofScale(5,5,5);
        
		calibration.getDistortedIntrinsics().loadProjectionMatrix();

        // swap background and face
        if (swapBackground == false){
            ofEnableLighting();
            light.enable();
            noise.getTexture().bind();
//            xmas.getTexture().bind();
            physicalMesh.drawFaces();
//            xmas.getTexture().unbind();
            noise.getTexture().unbind();
            ofDisableLighting();
        }
        
        else if (swapBackground == true){
            ps3cam.bind();
            physicalMesh.drawFaces();
            ps3cam.unbind();
        }
	}
}

void ofApp::keyPressed(int key) {
	if(key == 'r') {
		tracker.reset();
	}
    if (key == 'f'){
        ofToggleFullscreen();
    }
    if (key == 's'){
        swapBackground = !swapBackground;
    }
    if (key == 'b'){
        bigBrother = !bigBrother;
    }
}
