/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 7:50:04 AM Jan 20, 2011
 */
package cs526.jbox2dTests;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.MassData;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.FrictionJointDef;
import org.jbox2d.dynamics.joints.JointType;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

import cs526.animation.PdController;
import cs526.jbox2dTests.StickTest.PIDController;

/**
 * @author Daniel Murphy
 */
public class LampModel extends TestbedTest {

	float timestep = 1.0f / 60;
	int velocityIterations = 8;
	int positionIterations = 3;

	@Override
	public boolean isSaveLoadEnabled() {
		return true;
	}

	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest(boolean)
	 */
	@Override
	public void initTest(boolean argDeserialized) {
		if (argDeserialized) {
			return;
		}
		{
			BodyDef bd = new BodyDef();
			Body ground = getWorld().createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
			getWorld().setGravity(new Vec2(0, 0f));
		}

		createCharater2();
		
		String currentDri = new File(".").getAbsolutePath();
		System.out.println(currentDri);
	}

	private class BodyInfo {
		float height;
		float width;
		float mass;
	}

	private class JointInfo {
		float upperAngleDegree;
		float lowerAngleDegree;
		String a;
		String b;
		JointPosition aJointPosition;
		JointPosition bJointPosition;
	}

	public enum JointPosition {
		Left, Middle, Right
	}

	private Vec2 computeJointPositionVector(JointPosition jointPosition,
			float width, float height) {
		Vec2 vector = null;

		float dist = width / 2 - height / 2;
		if (jointPosition == JointPosition.Middle) {
			vector = new Vec2(0f, 0f);
		} else if (jointPosition == JointPosition.Left) {
			vector = new Vec2(-dist, 0f);
		} else if (jointPosition == JointPosition.Right) {
			vector = new Vec2(dist, 0f);
		}
		return vector;
	}

	private void createCharater2() {
		HashMap<String, BodyInfo> bodyInfoMap = new HashMap<String, LampModel.BodyInfo>();
		BodyInfo b1Info = new BodyInfo();
		b1Info.height = 0.08f * 2;
		b1Info.width = 1.0f * 2;
		b1Info.mass = 0.5f;
		bodyInfoMap.put("b1", b1Info);

		BodyInfo b2Info = new BodyInfo();
		b2Info.height = 0.08f * 2;
		b2Info.width = 0.7f * 2;
		b2Info.mass = 0.33f;
		bodyInfoMap.put("b2", b2Info);

		BodyInfo b3Info = new BodyInfo();
		b3Info.height = 0.08f * 2;
		b3Info.width = 0.7f * 2;
		b3Info.mass = 0.63f;
		bodyInfoMap.put("b3", b3Info);

		HashMap<String, Body> links = new HashMap<String, Body>();
		BodyDef bodyDef = new BodyDef();
		bodyDef.type = BodyType.DYNAMIC;
		bodyDef.position.set(0.0f, 1.0f);

		for (String key : bodyInfoMap.keySet()) {
			BodyInfo bodyInfo = bodyInfoMap.get(key);
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(bodyInfo.width / 2, bodyInfo.height / 2);

			FixtureDef fixture = new FixtureDef();
			fixture.shape = shape;
			fixture.density = bodyInfo.mass
					/ (bodyInfo.width * bodyInfo.height);
			fixture.friction = 0.8f;
			fixture.restitution = 0.2f;

			Body link = getWorld().createBody(bodyDef);
			link.createFixture(fixture);
			
			links.put(key, link);
		}

		link1 = links.get("b1");
		link2 = links.get("b2");
		Body link3 = links.get("b3");


		HashMap<String, JointInfo> jointInfoMap = new HashMap<String, LampModel.JointInfo>();
		JointInfo jointInfo1 = new JointInfo();
		jointInfo1.a = "b1";
		jointInfo1.b = "b2";
		jointInfo1.aJointPosition = JointPosition.Middle;
		jointInfo1.bJointPosition = JointPosition.Left;
		jointInfo1.upperAngleDegree = 130;
		jointInfo1.lowerAngleDegree = 100;
		jointInfoMap.put("a1", jointInfo1);

		JointInfo jointInfo2 = new JointInfo();
		jointInfo2.a = "b2";
		jointInfo2.b = "b3";
		jointInfo2.aJointPosition = JointPosition.Right;
		jointInfo2.bJointPosition = JointPosition.Left;
		jointInfo2.upperAngleDegree = -30;
		jointInfo2.lowerAngleDegree = -120;
		jointInfoMap.put("a2", jointInfo2);

		HashMap<String, RevoluteJoint> joints = new HashMap<String, RevoluteJoint>();
		for (String key : jointInfoMap.keySet()) {
			JointInfo jointInfo = jointInfoMap.get(key);
			RevoluteJointDef jointDef = new RevoluteJointDef();
			Body link1 = links.get(jointInfo.a);
			Body link2 = links.get(jointInfo.b);
			BodyInfo link1Info = bodyInfoMap.get(jointInfo.a);
			BodyInfo link2Info = bodyInfoMap.get(jointInfo.b);
			jointDef.bodyA = link1;
			jointDef.bodyB = link2;
			jointDef.localAnchorA = computeJointPositionVector(jointInfo.aJointPosition, link1Info.width, link1Info.height);
			jointDef.localAnchorB = computeJointPositionVector(jointInfo.bJointPosition, link2Info.width, link2Info.height);
			
			jointDef.upperAngle = (float) Math.toRadians(jointInfo.upperAngleDegree);
			jointDef.lowerAngle = (float) Math.toRadians(jointInfo.lowerAngleDegree);
			
			jointDef.enableLimit = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.enableMotor = true;
			
			joints.put(key, (RevoluteJoint) getWorld().createJoint(jointDef));
		}
		
//		RevoluteJointDef jointP1 = new RevoluteJointDef();
//		jointP1.bodyA = link1;
//		jointP1.bodyB = link2;
//		jointP1.localAnchorA = new Vec2(0f, 0f);
//		jointP1.localAnchorB = new Vec2(0.65f, 0f);
//		jointP1.enableLimit = true;
//		jointP1.upperAngle = (float) Math.PI / 180 * 130;
//		jointP1.lowerAngle = (float) Math.PI / 180 * 100;
//		jointP1.maxMotorTorque = 10.0f;
//		jointP1.motorSpeed = 0.0f;
//		jointP1.enableMotor = true;
		joint1 = (RevoluteJoint) joints.get("a1");

		// // P2
//		RevoluteJointDef jointP2 = new RevoluteJointDef();
//		jointP2.bodyA = link2;
//		jointP2.bodyB = link3;
//		jointP2.localAnchorA = new Vec2(-0.65f, 0f);
//		jointP2.localAnchorB = new Vec2(0.65f, 0f);
//		jointP2.enableLimit = true;
//		jointP2.upperAngle = -(float) Math.PI / 180 * 30;
//		jointP2.lowerAngle = -(float) Math.PI / 180 * 120; // -120 ~ -30 degrees
//		jointP2.maxMotorTorque = 10.0f;
//		jointP2.motorSpeed = 0.0f;
//		jointP2.enableMotor = true;
		joint2 = (RevoluteJoint) joints.get("a2");
		//
		
		HashMap<String, ControllerInfo> controllerInfoMap = new HashMap<String, ControllerInfo>();
		
		ControllerInfo controllerInfo = new ControllerInfo();
		
		controllerInfo.linkA = "b1";
		controllerInfo.ks = 80f;
		controllerInfo.kd = 2f;
		
		controllerInfoMap.put("a1", controllerInfo);
		
		controllerInfo = new ControllerInfo();
		controllerInfo.linkA = "b2";
		controllerInfo.ks = 40f;
		controllerInfo.kd = 2;
		
		controllerInfoMap.put("a2", controllerInfo);
		
		 controllers = new HashMap<String, PdController>();
		for (String key : controllerInfoMap.keySet())
		{
			
			ControllerInfo cInfo = controllerInfoMap.get(key);
			Body link = links.get(cInfo.linkA);
			PdController controller = new PdController(link, joints.get(key), cInfo.ks, cInfo.kd);
			controllers.put(key, controller);
		}
						
		controller1 = controllers.get("a1");
		controller2 = controllers.get("a2");
		
		DesiredState state1 = new DesiredState();
		state1.put("a1", 130f);
		state1.put("a2", -120f);
				
		DesiredState state2 = new DesiredState();
		state2.put("a1", 100f);
		state2.put("a2", -30f);
				
		DesiredState state3 = new DesiredState();
		state3.put("a1", 100f);
		state3.put("a2", -120f);
		
		states.add(state1);
		states.add(state2);
		states.add(state3);
		
	}
	
	private class ControllerInfo{
		String linkA; 
		float ks;
		float kd;
	}
	
	private class DesiredState
	{
		HashMap<String, Float> desiredAngles = new HashMap<String, Float>();
		
		public void put(String controllerName, Float angleInDegree)
		{
			desiredAngles.put(controllerName, (float)Math.toRadians(angleInDegree));
		}
	}
	
	
	private ArrayList<DesiredState> states = new ArrayList<LampModel.DesiredState>();
//	HashMap<String, >

	private void createCharacter() {
		{

			// L1
			BodyDef bodyDef = new BodyDef();
			bodyDef.type = BodyType.DYNAMIC;
			bodyDef.position.set(0.0f, 1.0f);

			PolygonShape shape = new PolygonShape();
			shape.setAsBox(1.0f, 0.08f); // 2m * 0.16m

			FixtureDef fixture = new FixtureDef();
			fixture.shape = shape;
			fixture.density = 1.5f;
			fixture.friction = 0.8f;
			fixture.restitution = 0.2f;

			link1 = getWorld().createBody(bodyDef);

			link1.createFixture(fixture);

			shape.setAsBox(0.7f, 0.08f); // 1.4m * 0.16m
			fixture.density = 1.5f;
			fixture.shape = shape;
			link2 = getWorld().createBody(bodyDef);
			link2.createFixture(fixture);

			fixture.density = 3.0f;
			Body link3 = getWorld().createBody(bodyDef);
			link3.createFixture(fixture);

			// // create joints
			//
			// // P1
			RevoluteJointDef jointP1 = new RevoluteJointDef();
			jointP1.bodyA = link1;
			jointP1.bodyB = link2;
			jointP1.localAnchorA = new Vec2(0f, 0f);
			jointP1.localAnchorB = new Vec2(0.65f, 0f);
			jointP1.enableLimit = true;
			jointP1.upperAngle = (float) Math.PI / 180 * 130;
			jointP1.lowerAngle = (float) Math.PI / 180 * 100;
			jointP1.maxMotorTorque = 10.0f;
			jointP1.motorSpeed = 0.0f;
			jointP1.enableMotor = true;
			// jointP1.referenceAngle = (float)-Math.PI / 9;
			joint1 = (RevoluteJoint) getWorld().createJoint(jointP1);
			// //
			//
			// // P2
			RevoluteJointDef jointP2 = new RevoluteJointDef();
			jointP2.bodyA = link2;
			jointP2.bodyB = link3;
			jointP2.localAnchorA = new Vec2(-0.65f, 0f);
			jointP2.localAnchorB = new Vec2(0.65f, 0f);
			jointP2.enableLimit = true;
			jointP2.upperAngle = -(float) Math.PI / 180 * 30;
			jointP2.lowerAngle = -(float) Math.PI / 180 * 120; // -120 ~ -30
																// degrees
			jointP2.maxMotorTorque = 10.0f;
			jointP2.motorSpeed = 0.0f;
			jointP2.enableMotor = true;
			joint2 = (RevoluteJoint) getWorld().createJoint(jointP2);
			//

			StickTest st = new StickTest();
			controller1 = new PdController(link1, joint1, 80f, 2f);
			controller2 = new PdController(link2, joint2, 40f, 1f);
		}
	}

	HashMap<String, PdController> controllers = new HashMap<String, PdController>();
	PdController controller1, controller2;
	float gravity = 0.0f;
	static float GRAVITY = 10.0f;
	RevoluteJoint joint1, joint2;

	int jumpStateNumber = 0;

	Body link1, link2;

	@Override
	public synchronized void step(TestbedSettings settings) {
		super.step(settings);
		addTextLine(String.format("joint angle is %.2f degrees.\n ",
				joint1.getJointAngle() * 180 / (Math.PI)));
		addTextLine(String.format("kick is %d", jumpStateNumber));
		addTextLine(String.format("mass of link 1 %f", link1.getMass()));
		addTextLine(String.format("mass of link 2 %fkg", link2.getMass()));
		addTextLine(String.format("target angle is %.2f degrees",
				Math.toDegrees(controller1.targetAngle)));

		addTextLine(String.format("current angle is %.2f degrees",
				Math.toDegrees(controller1.currentAngle)));
		addTextLine(String.format("torque is %.2f N", controller1.torque));
		addTextLine(String.format("degree velocity is %.2f degrees/s",
				Math.toDegrees(joint1.getJointSpeed())));

		addTextLine(String.format("target angle of joint2 is %.2f degrees",
				Math.toDegrees(controller2.targetAngle)));

		addTextLine(String.format("current angle of joint 2 is %.2f degrees",
				Math.toDegrees(controller2.currentAngle)));
		addTextLine(String.format("torque of joint2 is %.2f Nm",
				controller2.torque));
		addTextLine(String.format(
				"degree velocity of joint 2 is %.2f degrees/s",
				Math.toDegrees(joint2.getJointSpeed())));

		DesiredState state = states.get(jumpStateNumber);
		for (String key : state.desiredAngles.keySet())
		{
			PdController controller = controllers.get(key);
			float desiredAngle = state.desiredAngles.get(key);
			controller.moveTo(desiredAngle);
		}
		
	}

	@Override
	public void keyPressed(char key, int argKeyCode) {
		switch (key) {
		case 'g':
			gravity = GRAVITY - gravity;
			getWorld().setGravity(new Vec2(0, -gravity));
			System.out.println("gravity is " + gravity);
			break;

		case 'j':
			getModel().getKeys()['j'] = false;
			jumpStateNumber++;
			jumpStateNumber %= states.size();
			break;
		}
	}
	
	

	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Lamp Model";
	}

}
