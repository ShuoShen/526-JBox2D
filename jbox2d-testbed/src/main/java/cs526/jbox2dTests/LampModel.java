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
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;
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

import cs526.controls.PdController;
import cs526.jbox2dTests.StickTest.PIDController;
import cs526.models.CharacterInfo;
import cs526.utilities.ControllerInfo;
import cs526.utilities.DesiredState;
import cs526.utilities.JointInfo;
import cs526.utilities.LinkPosition;
import cs526.utilities.LinkInfo;

/**
 * @author Shuo Shen
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

	private Vec2 computeJointPositionVector(LinkPosition jointPosition,
			float width, float height) {
		Vec2 vector = null;

		float dist = width / 2 - height / 2;
		if (jointPosition == LinkPosition.Middle) {
			vector = new Vec2(0f, 0f);
		} else if (jointPosition == LinkPosition.Left) {
			vector = new Vec2(-dist, 0f);
		} else if (jointPosition == LinkPosition.Right) {
			vector = new Vec2(dist, 0f);
		}
		return vector;
	}
//	HashMap<String, LinkInfo> linkInfoMap = new HashMap<String, LinkInfo>();

	CharacterInfo model;
	private void createCharater2() {
		
		model = new CharacterInfo(this);
		
		HashMap<String, Body> links = new HashMap<String, Body>();
		BodyDef bodyDef = new BodyDef();
		bodyDef.type = BodyType.DYNAMIC;
		bodyDef.position.set(0.0f, 1.0f);

		for (String key : model.getLinkNames()){
			LinkInfo bodyInfo = model.getLinkInfoByName(key);
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

		HashMap<String, RevoluteJoint> joints = new HashMap<String, RevoluteJoint>();
		for (String jointName : model.getJointNames()){
			JointInfo jointInfo = model.getJointInfoByName(jointName);
			RevoluteJointDef jointDef = new RevoluteJointDef();
			Body link1 = links.get(jointInfo.linkA);
			Body link2 = links.get(jointInfo.linkB);
			LinkInfo link1Info = model.getLinkInfoByName(jointInfo.linkA);
			LinkInfo link2Info = model.getLinkInfoByName(jointInfo.linkB);
			jointDef.bodyA = link1;
			jointDef.bodyB = link2;
			jointDef.localAnchorA = computeJointPositionVector(jointInfo.linkAPosition, link1Info.width, link1Info.height);
			jointDef.localAnchorB = computeJointPositionVector(jointInfo.linkBPosition, link2Info.width, link2Info.height);
			
			jointDef.upperAngle = (float) Math.toRadians(jointInfo.upperAngleDegree);
			jointDef.lowerAngle = (float) Math.toRadians(jointInfo.lowerAngleDegree);
			
			jointDef.enableLimit = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.enableMotor = true;
			
			joints.put(jointName, (RevoluteJoint) getWorld().createJoint(jointDef));
		}
		
		joint1 = (RevoluteJoint) joints.get("a1");

		joint2 = (RevoluteJoint) joints.get("a2");
		//
		
		
		 controllers = new HashMap<String, PdController>();
		for (String key : model.getControllerNames())
		{
			
			ControllerInfo cInfo = model.getControllerInfoByJointName(key);
			PdController controller = new PdController(joints.get(key), cInfo.ks, cInfo.kd);
			controllers.put(key, controller);
		}
						
		controller1 = controllers.get("a1");
		controller2 = controllers.get("a2");

		
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

		DesiredState state = model.getState(jumpStateNumber);
		for (String key : state.getJointNames())
		{
			
			PdController controller = controllers.get(key);
			float desiredAngle = state.getAngleByJointName(key);
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
