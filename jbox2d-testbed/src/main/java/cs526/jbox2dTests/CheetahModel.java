package cs526.jbox2dTests;

import org.jbox2d.common.Vec2;

public class CheetahModel extends AutoLoadedTest {
	@Override
	public void initTest(boolean argDeserialized) {
		// The friction motor
		frictionMotorTorque = 0.3f;
		super.initTest(argDeserialized);
		
		// zoom the camera
		super.setCamera(new Vec2(0f, 0f), 50f);
	}
}