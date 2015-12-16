#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxFaceTracker.h"
#include "ofxPS3EyeGrabber.h"


class ofApp : public ofBaseApp {
public:
	void setup();
	void updatePhysicalMesh();
	void update();
	void draw();
	void keyPressed(int key);
    void imageNoise();
    
    ofImage noise;
    int whiteBlack;
    int lineY;
    
	ofVideoGrabber ps3cam;
	ofxFaceTracker tracker;
	ofxCv::Calibration calibration;
	ofLight light;
	
	ofMesh physicalMesh;
    
    ofPixels pixelsRGB;
    
    bool swapBackground;
    
};
