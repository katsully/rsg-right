#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "cinder/osc/Osc.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define USE_UDP 1

#if USE_UDP
using Sender = osc::SenderUdp;
#else
using sender = osc::SenderTcp;
#endif

const std::string destinationHost = "10.0.1.2";
const uint16_t destinationPort = 10001;

// RIGHT CAMERA
class RSGApp : public App {
public:
	RSGApp();
	void setup() override;
	void update() override;
	void mouseDown(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void draw() override;
	float mapIt(float s, float a1, float a2, float b1, float b2);
	void shutdown();

	bool mFullScreen = true;

	gl::Texture2dRef mTexture;
	vector<vec2> mTrail;

	Sender mSender;

private:
	Kinect2::BodyFrame mBodyFrame;
	ci::Channel8uRef mChannelBodyIndex;
	ci::Channel16uRef mChannelDepth;
	Kinect2::DeviceRef mDevice;

	int width;
	int height;
};

RSGApp::RSGApp() : mSender(10001, destinationHost, destinationPort) {
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame) {
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame) {
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame) {
		mChannelDepth = frame.getChannel();
	});
}

void RSGApp::setup()
{
	setFullScreen(mFullScreen);
	width = getWindowWidth();
	height = getWindowHeight();
	auto img = loadImage(loadAsset("TOMS_GP_v7.11.png"));
	mTexture = gl::Texture2d::create(img);

	// set up OSC
	mSender.bind();
#if ! USE_UDP
	mSender.connect();
#endif
}

void RSGApp::mouseDown(MouseEvent event) {
	mFullScreen = !mFullScreen;
	setFullScreen(mFullScreen);
}

void RSGApp::keyDown(KeyEvent event) {
	if (event.getChar() == 'a') {
		mTrail.clear();
	}
}

void RSGApp::update()
{
}

void RSGApp::draw()
{
	gl::clear(Color::white());
	gl::color(Color::white());
	gl::draw(mTexture, Rectf(0, 0, getWindowWidth(), getWindowHeight()));
	for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
		if (body.isTracked()) {
			for (const auto& joint : body.getJointMap()) {
				// if it is the mid spine
				if (joint.first == 1) {
					vec3 pos = joint.second.getPosition();
					// console() << pos.x << endl;
					// distance between start of the grid and the camera (in pixels)
					float offset1 = (width / 19.0) * .45;
					// distance between camera and downstage edge & distance between camera threshold in the x direction and upstage edge
					float offset2 = (height / 15.0) * 4.15;
					// distance between edge of downstage and threshold for camera tracking in x direction
					float offset3 = (height / 15.0) * 1.15;
					std::wstringstream ss;
					ss << "x pos: " << pos.x << endl;
					OutputDebugString(ss.str().c_str());
					float xPos = mapIt(pos.z, 0.7, 4.28, width*.82, width * .56);
					float yPos = mapIt(pos.x, -2.2, 2.2, height*.1, height*.8);
					gl::color(Color(1, 0, 0));
					gl::drawSolidCircle(vec2(xPos, yPos), 25);
					mTrail.push_back(vec2(xPos, yPos));
					osc::Message msg("/pos/2");
					msg.append(xPos/width);
					msg.append(yPos/height);
					mSender.send(msg);
				}
			}
		}
	}
	for (vec2& point : mTrail) {
		gl::drawSolidCircle(point, 5);
	}
}

float RSGApp::mapIt(float s, float a1, float a2, float b1, float b2) {
	return b1 + (s - a1)*(b2 - b1) / (a2 - a1);
}

void RSGApp::shutdown() {
	mDevice->stop();
}

CINDER_APP(RSGApp, RendererGl)