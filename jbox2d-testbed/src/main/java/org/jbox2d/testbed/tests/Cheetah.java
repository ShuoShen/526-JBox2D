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
 * Created at 7:59:38 PM Jan 12, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Cheetah extends TestbedTest {
	int numOfjoints = 1, numOfLinks = 2;
	private RevoluteJoint m_joint[] = new RevoluteJoint[numOfjoints];
	private boolean isLeft = false;
	float leg_upper = 0.5f , leg_bottom = 0.5f, leg_width = 0.1f;
	float body_length = 0.4f, body_width = 0.2f;
	Body upBody;
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest(boolean)
	 */
	@Override
	public void initTest(boolean argDeserialized) {
		Body ground = null;
		{
			BodyDef bd = new BodyDef();
			ground = getWorld().createBody(bd);
			
			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
		}
		
		{
			float L = 2.0f, W = 0.5f , H = 30.0f;
			
			RevoluteJointDef rjd = new RevoluteJointDef();
			
			Body body[] = new Body[numOfLinks];
			BodyDef bd = new BodyDef();
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(L, W);
			
			bd.type = BodyType.DYNAMIC;
			for(int i = 0; i < numOfLinks; i++){
				
				bd.position.set(0.0f + i * 2 * L, H);
				body[i] = getWorld().createBody(bd);
				body[i].createFixture(shape, 5.0f);
			}
			
			float flag = - 1.0f;
			for(int i = 0; i < numOfjoints; i++){
				
				rjd.initialize(body[i], body[i+1], new Vec2(L + i * 2 * L, H));
				rjd.motorSpeed = 1.0f * MathUtils.PI;
				rjd.maxMotorTorque = 10000.0f;
				//rjd.enableMotor = true;
				
				rjd.lowerAngle = -0.25f * MathUtils.PI;
				rjd.upperAngle = 0f;
				rjd.enableLimit = true;
				
				//flag = flag * (-1.0f);
				m_joint[i] = (RevoluteJoint) getWorld().createJoint(rjd);
				//m_joint[i].setMotorSpeed(2.0f * flag);
				//System.out.print(flag);
			}
			
			shape.setAsBox(body_width/2f, body_length/2f);
			bd.position.set(-10f, 30f);
			bd.linearDamping = 0.0f;
			//bd.angularDamping = 0.01f;
			upBody = getWorld().createBody(bd);
			upBody.createFixture(shape, 1f);
			upBody.m_torque = 25f;
			
			createLegs(new Vec2(-10f, 30f - body_length/2f), upBody);
			System.out.println(getWorld().getGravity());
			System.out.println(upBody.getMass());
		}
	}
	
	public void createLegs(Vec2 p_Pelvis, Body upBody){
		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		Body body[] = new Body[4];
		PolygonShape shape = new PolygonShape();
		RevoluteJointDef rjd = new RevoluteJointDef();
		RevoluteJoint joint[] = new RevoluteJoint[4];
		FixtureDef fd1 = new FixtureDef();
		fd1.filter.groupIndex = -1;
		for(int i = 0; i < 2; i++){
			bd.position.set(p_Pelvis.x, p_Pelvis.y - leg_upper/2f);
			shape.setAsBox(leg_width/2f, leg_upper/2f);
			body[i * 2] = getWorld().createBody(bd);
			fd1.shape = shape;
			fd1.density = 0.5f;
			body[i * 2].createFixture(fd1);
			rjd.initialize(body[i * 2], upBody, p_Pelvis);
			rjd.lowerAngle = -0.5f * MathUtils.PI;
			rjd.upperAngle = 0f;
			rjd.enableLimit = true;
			rjd.collideConnected = false;
			joint[i] = (RevoluteJoint) getWorld().createJoint(rjd);
			
			bd.position.set(p_Pelvis.x, p_Pelvis.y - leg_upper - leg_bottom/2f);
			shape.setAsBox(leg_width/2f, leg_bottom/2f);
			body[i * 2 + 1] = getWorld().createBody(bd);
			fd1.shape = shape;
			fd1.density = 0.5f;
			body[i * 2 + 1].createFixture(fd1);
			rjd.initialize(body[i * 2], body[i * 2 + 1], new Vec2(p_Pelvis.x, p_Pelvis.y - leg_upper));
			rjd.lowerAngle = -0.5f * MathUtils.PI;
			rjd.upperAngle = 0f;
			rjd.enableLimit = true;
			rjd.collideConnected = false;
			joint[i*2 + 1] = (RevoluteJoint) getWorld().createJoint(rjd);
		}
		
	}
	

	
	
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		//addTextLine("Limits " + (m_joint.isLimitEnabled() ? "on" : "off") + ", Motor "
		//		+ (m_joint.isMotorEnabled() ? "on " : "off ") + (isLeft ? "left" : "right"));
		//addTextLine("Keys: (l) limits, (m) motor, (a) left, (d) right");
		//System.out.println(getWorld().getGravity());
		//System.out.println(upBody.getMass());
		//upBody.applyForce( getWorld().getGravity().mul((-1) * upBody.getMass()), upBody.getLocalCenter());
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#keyPressed(char, int)
	 */
	@Override
	public void keyPressed(char argKeyChar, int argKeyCode) {
		/*
		switch (argKeyChar) {
			case 'l' :
				m_joint.enableLimit(!m_joint.isLimitEnabled());
				getModel().getKeys()['l'] = false;
				break;
			case 'm' :
				m_joint.enableMotor(!m_joint.isMotorEnabled());
				getModel().getKeys()['m'] = false;
				break;
			case 'a' :
				m_joint.setMotorSpeed(1.0f * MathUtils.PI);
				getModel().getKeys()['a'] = false;
				isLeft = true;
				break;
			case 'd' :
				m_joint.setMotorSpeed(-1.0f * MathUtils.PI);
				getModel().getKeys()['d'] = false;
				isLeft = false;
				break;
		}
		*/
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Cheetah";
	}
	
}
