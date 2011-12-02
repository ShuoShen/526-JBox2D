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
public class CheetahModel extends TestbedTest {

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
		float linkLength = .2f;    // length of each link 
		float linkWidth = linkLength / 5;   // width set to be one fifth of length
		
		
		{
			// Create body for the links
			BodyDef bodyDef = new BodyDef();
			bodyDef.type = BodyType.DYNAMIC;
			bodyDef.position.set(0.0f, 1.5f);	
						
			// create shape for the links
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(linkLength / 2, linkWidth / 2);
			
			// Create fixture for the links
			FixtureDef fixture = new FixtureDef();
			fixture.shape = shape;
			fixture.density = 100.0f;
			fixture.friction = 0.5f; 
			fixture.restitution = 0.3f;
			
			
			// create seven links
			Body[] links = new Body[7];
			for (int i = 0; i < links.length; i++){
				links[i] = getWorld().createBody(bodyDef);
				links[i].createFixture(fixture);
			}
			
			// create joints
			
			// P1
			RevoluteJointDef jointP1 = new RevoluteJointDef();
			jointP1.bodyA = links[0];
			jointP1.bodyB = links[1];
			jointP1.localAnchorA = new Vec2(linkLength / 2 * 0.9f, 0f);
			jointP1.localAnchorB = new Vec2(-linkLength / 2 * 0.9f, 0f);
			jointP1.enableLimit = true;
			jointP1.upperAngle = (float)Math.PI / 4;
			jointP1.lowerAngle = (float) -Math.PI / 4;
//			joint12.maxMotorTorque = 10.0f;
//			joint12.motorSpeed = 0.0f; 
//			joint12.enableMotor = true;
//			jointP1.referenceAngle = (float)-Math.PI / 9;
			getWorld().createJoint(jointP1);
//						
			
			// P2 
			RevoluteJointDef jointP2 = new RevoluteJointDef();
			jointP2.bodyA = links[4];
			jointP2.bodyB = links[0];
			jointP2.localAnchorA = new Vec2(linkLength / 2 * 0.9f, 0f);
			jointP2.localAnchorB = new Vec2(-linkLength / 2 * 0.9f, 0f);
			jointP2.enableLimit = true;
			jointP2.upperAngle = 0f;
			jointP2.lowerAngle = (float) -Math.PI / 4;
//			joint12.maxMotorTorque = 10.0f;
//			joint12.motorSpeed = 0.0f; 
//			joint12.enableMotor = true;
//			jointP2.referenceAngle = (float)-Math.PI / 9;
			getWorld().createJoint(jointP2);
			
			// A1 
			RevoluteJointDef jointA1 = new RevoluteJointDef();
			jointA1.bodyA = links[1];
			jointA1.bodyB = links[2];
			jointA1.localAnchorA = new Vec2(linkLength / 2 * 0.9f, 0f);
			jointA1.localAnchorB = new Vec2(-linkLength / 2 * 0.9f, 0f);
			jointA1.enableLimit = true;
			jointA1.upperAngle = (float)Math.PI / 4;
			jointA1.lowerAngle = (float) -Math.PI / 4;
//			joint12.maxMotorTorque = 10.0f;
//			joint12.motorSpeed = 0.0f; 
//			joint12.enableMotor = true;
//			jointA1.referenceAngle = (float)-Math.PI / 4;
			getWorld().createJoint(jointA1);
			
			
			// A2 
			RevoluteJointDef jointA2 = new RevoluteJointDef();
			jointA2.bodyA = links[2];
			jointA2.bodyB = links[3];
			jointA2.localAnchorA = new Vec2(linkLength / 2 * 0.9f, 0f);
			jointA2.localAnchorB = new Vec2(-linkLength / 2 * 0.9f, 0f);
			jointA2.enableLimit = true;
			jointA2.upperAngle = (float)Math.PI / 3 * 2;
			jointA2.lowerAngle = (float)Math.PI/6f;
//			joint12.maxMotorTorque = 10.0f;
//			joint12.motorSpeed = 0.0f; 
//			joint12.enableMotor = true;
//			jointA2.referenceAngle = (float)Math.PI / 4;
			getWorld().createJoint(jointA2);
			
			
			
			// A3 
			RevoluteJointDef jointA3 = new RevoluteJointDef();
			jointA3.bodyA = links[5];
			jointA3.bodyB = links[4];
			jointA3.localAnchorA = new Vec2(linkLength / 2 * 0.9f, 0f);
			jointA3.localAnchorB = new Vec2(-linkLength / 2 * 0.9f, 0f);
			jointA3.enableLimit = true;
			jointA3.upperAngle = -(float)Math.PI / 18;
			jointA3.lowerAngle = -(float)Math.PI/ 2.5f;
//			joint12.maxMotorTorque = 10.0f;
//			joint12.motorSpeed = 0.0f; 
//			joint12.enableMotor = true;
//			jointA3.referenceAngle = -(float)Math.PI / 17;
			getWorld().createJoint(jointA3);
			
			
			// A4 
			RevoluteJointDef jointA4 = new RevoluteJointDef();
			jointA4.bodyA = links[6];
			jointA4.bodyB = links[5];
			jointA4.localAnchorA = new Vec2(linkLength / 2 * 0.9f, 0f);
			jointA4.localAnchorB = new Vec2(-linkLength / 2 * 0.9f, 0f);
			jointA4.enableLimit = true;
			jointA4.upperAngle = -(float)Math.PI / 6;
			jointA4.lowerAngle = -(float)Math.PI/ 3 * 2f;
//			joint12.maxMotorTorque = 10.0f;
//			joint12.motorSpeed = 0.0f; 
//			joint12.enableMotor = true;
//			jointA4.referenceAngle = -(float)Math.PI / 2;
			getWorld().createJoint(jointA4);
			
////			bodyDef.position.set(1f, 15f);
//			Body link2 = getWorld().createBody(bodyDef);
//			link2.createFixture(fixture);
//			
//			RevoluteJointDef jointDef = new RevoluteJointDef();
//			jointDef.bodyA = link1;
//			jointDef.bodyB = link2;
////			jointDef.initialize(link1, link2, new Vec2(.5f, 15f));
//			jointDef.localAnchorA = new Vec2(.55f, 0f);
//			jointDef.localAnchorB = new Vec2(-.55f, 0f);
			
						
			
//			RevoluteJoint joint = (RevoluteJoint)getWorld().createJoint(jointDef);
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
		return "Cheetah Model";
	}
	
}
