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
			shape.setAsBox(1.0f, 0.08f);
			
			FixtureDef fixture = new FixtureDef();
			fixture.shape = shape;
			fixture.density = 100.0f;
			fixture.friction = 0.5f; 
			fixture.restitution = 0.3f;
			
			Body link1 = getWorld().createBody(bodyDef);
			link1.createFixture(fixture);
			
			shape.setAsBox(0.7f, 0.08f);
			fixture.density = 400.0f;
			fixture.shape = shape;
			Body link2 = getWorld().createBody(bodyDef);
			link2.createFixture(fixture);
			
			fixture.density = 900.f;
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
			jointP1.lowerAngle = (float)Math.PI / 180 * 110;
			jointP1.maxMotorTorque = 10.0f;
			jointP1.motorSpeed = 0.0f; 
			jointP1.enableMotor = true;
//			jointP1.referenceAngle = (float)-Math.PI / 9;
			getWorld().createJoint(jointP1);
////						
			
//			// P1
			RevoluteJointDef jointP2 = new RevoluteJointDef();
			jointP2.bodyA = link2;
			jointP2.bodyB = link3;
			jointP2.localAnchorA = new Vec2(-0.65f, 0f);
			jointP2.localAnchorB = new Vec2(0.65f, 0f);
			jointP2.enableLimit = true;
			jointP2.upperAngle = -(float)Math.PI / 180 * 30;
			jointP2.lowerAngle = -(float)Math.PI / 180 * 120;
			jointP2.maxMotorTorque = 10.0f;
			jointP2.motorSpeed = 0.0f; 
			jointP2.enableMotor = true;
//			jointP1.referenceAngle = (float)-Math.PI / 9;
			getWorld().createJoint(jointP2);
			
		}
	}
	
	float gravity = 0.0f;
	static float GRAVITY = 10.0f;
	
	@Override
	public void keyPressed(char key, int argKeyCode) {
		switch (key) {
			case 'g' :
				gravity = GRAVITY - gravity;
				getWorld().setGravity(new Vec2(0, -gravity));
				System.out.println("gravity is " + gravity);
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
