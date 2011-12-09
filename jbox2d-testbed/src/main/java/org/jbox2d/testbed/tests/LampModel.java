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
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
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
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

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
	  if(argDeserialized){
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
		
		
		{
			
			// L1
			BodyDef bodyDef = new BodyDef();
			bodyDef.type = BodyType.DYNAMIC;
			bodyDef.position.set(0.0f, 1.0f);

			PolygonShape shape = new PolygonShape();
			shape.setAsBox(1.0f, 0.08f);		// 2m  *  0.16m 
			
			FixtureDef fixture = new FixtureDef();
			fixture.shape = shape;
			fixture.density = 1.5f;
			fixture.friction = 0.8f; 
			fixture.restitution = 0.2f;
			
			 link1 = getWorld().createBody(bodyDef);
			
			link1.createFixture(fixture);
			
			shape.setAsBox(0.7f, 0.08f);		// 1.4m * 0.16m
			fixture.density = 1.5f;
			fixture.shape = shape;
			 link2 = getWorld().createBody(bodyDef);
			link2.createFixture(fixture);
			
			fixture.density = 3.0f;
			Body link3 = getWorld().createBody(bodyDef);
			link3.createFixture(fixture);
			
//			// create joints
//			
//			// P1
			RevoluteJointDef jointP1 = new RevoluteJointDef();
			jointP1.bodyA = link1;
			jointP1.bodyB = link2;
			jointP1.localAnchorA = new Vec2(0f, 0f);
			jointP1.localAnchorB = new Vec2(0.65f, 0f);
			jointP1.enableLimit = true;
			jointP1.upperAngle = (float)Math.PI / 180 * 130;
			jointP1.lowerAngle = (float)Math.PI / 180 * 100;
			jointP1.maxMotorTorque = 10.0f;
			jointP1.motorSpeed = 0.0f; 
			jointP1.enableMotor = true;
//			jointP1.referenceAngle = (float)-Math.PI / 9;
		 joint1 = (RevoluteJoint) getWorld().createJoint(jointP1);
////						
//			
//			// P2
			RevoluteJointDef jointP2 = new RevoluteJointDef();
			jointP2.bodyA = link2;
			jointP2.bodyB = link3;
			jointP2.localAnchorA = new Vec2(-0.65f, 0f);
			jointP2.localAnchorB = new Vec2(0.65f, 0f);
			jointP2.enableLimit = true;
			jointP2.upperAngle = -(float)Math.PI / 180 * 30;
			jointP2.lowerAngle = -(float)Math.PI / 180 * 120;    // -120 ~ -30 degrees
			jointP2.maxMotorTorque = 10.0f;
			jointP2.motorSpeed = 0.0f; 
			jointP2.enableMotor = true;
			joint2 = (RevoluteJoint) getWorld().createJoint(jointP2);
//			
		 
		 
		 	StickTest st = new StickTest();
		 	controller1 =  st.new PIDController(link1, joint1, 80f, 2f );
		 	controller2 =  st.new PIDController(link2, joint2, 40f, 1f );
		}
		

	}
	StickTest.PIDController controller1, controller2;
	float gravity = 0.0f;
	static float GRAVITY = 10.0f;
	RevoluteJoint joint1, joint2;
	
	int jumpState = 2;
	
	Body link1, link2;
 
	
	@Override
	public synchronized void step(TestbedSettings settings) {
		super.step(settings);
		addTextLine(String.format("joint angle is %.2f degrees.\n ", joint1.getJointAngle() * 180 / (Math.PI)));
		addTextLine(String.format("kick is %d", jumpState));
		addTextLine(String.format("mass of link 1 %f", link1.getMass()));
		addTextLine(String.format("mass of link 2 %fkg", link2.getMass()));
		addTextLine(String.format("target angle is %.2f degrees", Math.toDegrees(controller1.targetAngle)));
		
		addTextLine(String.format("current angle is %.2f degrees", Math.toDegrees(controller1.currentAngle)));
		addTextLine(String.format("torque is %.2f N", controller1.torque));
		addTextLine(String.format("degree velocity is %.2f degrees/s", Math.toDegrees(joint1.getJointSpeed())));
		
		addTextLine(String.format("target angle of joint2 is %.2f degrees", Math.toDegrees(controller2.targetAngle)));
		
		addTextLine(String.format("current angle of joint 2 is %.2f degrees", Math.toDegrees(controller2.currentAngle)));
		addTextLine(String.format("torque of joint2 is %.2f Nm", controller2.torque));
		addTextLine(String.format("degree velocity of joint 2 is %.2f degrees/s", Math.toDegrees(joint2.getJointSpeed())));
		
		
		if (jumpState == 0)
		{
			controller1.moveTo((float)Math.PI / 180 * 130);
			controller2.moveTo((float)Math.toRadians(-120));
		}
		else if (jumpState == 1)
		{
			controller1.moveTo((float)Math.PI / 180 * 100);
			controller2.moveTo((float)Math.toRadians(-30));
		}
		else
		{
			controller1.moveTo((float)Math.PI / 180 * 100 );   
			controller2.moveTo((float)Math.toRadians(-120));
		}
	}
	@Override
	public void keyPressed(char key, int argKeyCode) {
		switch (key) {
			case 'g' :
				gravity = GRAVITY - gravity;
				getWorld().setGravity(new Vec2(0, -gravity));
				System.out.println("gravity is " + gravity);
				break;
				
			case 'j' :
				getModel().getKeys()['j'] = false;
				jumpState++;
				jumpState %= 3;
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
