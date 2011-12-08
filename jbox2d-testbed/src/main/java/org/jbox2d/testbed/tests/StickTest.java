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

import org.jbox2d.collision.shapes.CircleShape;
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
public class StickTest extends TestbedTest {
	
	private RevoluteJoint m_joint, m_joint2;
	private boolean isLeft = false;
	Body body1;
	Body body2;
	Body body[] = new Body[7];
	RevoluteJoint joint[] = new RevoluteJoint[6];
	PIDController con1, con2;
	PIDController con[] = new PIDController[6];
	int kick = 0;
	float time = 0;
	float targetAngle = MathUtils.PI * 7/6;
	float timeStep = 2f;
	float phase = 0;
	float L = 2.0f, W = 0.5f , H = 0.1f;
	float leg_upper = 0.5f , leg_bottom = 0.5f, leg_width = 0.1f;
	float body_length = 0.4f, body_width = 0.2f;
	float foot_length = 0.2f, foot_width = 0.1f;
	boolean gSwitch = true;
	int flag = 1;
	float ball_r = 0.1f;
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
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(body_width/2f, body_length/2f);
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.position.set(0.0f, H + body_length/2 + leg_upper + leg_bottom);
			body[0] = getWorld().createBody(bd);
			FixtureDef fd1 = new FixtureDef();
			fd1.filter.groupIndex = -1;
			fd1.shape = shape;
			fd1.density = 0.5f;
			body[0].createFixture(fd1);
			createLegs(new Vec2(0, body[0].getWorldCenter().y - body_length/2f), body[0]);
			
			
			CircleShape shape2 = new CircleShape();
			shape2.m_radius = ball_r;

			BodyDef bd2 = new BodyDef();
			bd2.type = BodyType.DYNAMIC;
			bd2.position.set(0.5f, 0.5f);

			Body body = getWorld().createBody(bd2);
			body.createFixture(shape2, 1.0f);
		}
	}
	
	public void createLegs(Vec2 p_Pelvis, Body upBody){
		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		PolygonShape shape = new PolygonShape();
		RevoluteJointDef rjd = new RevoluteJointDef();
		FixtureDef fd1 = new FixtureDef();
		fd1.filter.groupIndex = -1;
		for(int i = 0; i < 2; i++){
			bd.type = BodyType.DYNAMIC;
			//up leg
			bd.position.set(p_Pelvis.x, p_Pelvis.y - leg_upper/2f);
			shape.setAsBox(leg_width/2f, leg_upper/2f);
			body[i * 3 + 1] = getWorld().createBody(bd);
			fd1.shape = shape;
			fd1.density = 0.5f;
			body[i * 3 + 1].createFixture(fd1);
			rjd.initialize(upBody, body[i * 3 + 1],  p_Pelvis);
			if(i == 0){
				rjd.lowerAngle = 0;
				rjd.upperAngle = 0;
			}else{
				rjd.lowerAngle = -0.5f * MathUtils.PI;
				rjd.upperAngle = 0.5f * MathUtils.PI;
			}
			
			rjd.enableLimit = true;
			rjd.collideConnected = false;
			joint[i * 3] = (RevoluteJoint) getWorld().createJoint(rjd);
			con[i * 3] = new PIDController(body[i * 3 + 1], joint[i * 3]);
			//bottom leg
			bd.position.set(p_Pelvis.x, p_Pelvis.y - leg_upper - leg_bottom/2f);
			shape.setAsBox(leg_width/2f, leg_bottom/2f);
			body[i * 3 + 2] = getWorld().createBody(bd);
			fd1.shape = shape;
			fd1.density = 0.5f;
			body[i * 3 + 2].createFixture(fd1);
			rjd.initialize(body[i * 3 + 1], body[i * 3 + 2], new Vec2(p_Pelvis.x, p_Pelvis.y - leg_upper));
			rjd.lowerAngle = 0;//-0.5f * MathUtils.PI;
			rjd.upperAngle = 0;
			rjd.enableLimit = true;
			rjd.collideConnected = false;
			joint[i * 3 + 1] = (RevoluteJoint) getWorld().createJoint(rjd);
			con[i * 3 + 1] = new PIDController(body[i * 3 + 2], joint[i * 3 + 1]);
			//foot
			if (i == 1){
				bd.type = BodyType.STATIC;
			}
			bd.position.set(p_Pelvis.x, p_Pelvis.y - leg_upper - leg_bottom);
			shape.setAsBox(foot_length/2f, foot_width/2f);
			body[i * 3 + 3] = getWorld().createBody(bd);
			fd1.shape = shape;
			fd1.density = 0.5f;
			body[i * 3 + 3].createFixture(fd1);
			rjd.initialize(body[i * 3 + 2], body[i * 3 + 3], new Vec2(p_Pelvis.x, p_Pelvis.y - leg_upper - leg_bottom));
			rjd.lowerAngle = 0;//-0.5f * MathUtils.PI;
			rjd.upperAngle = 0;//0.1f * MathUtils.PI;
			rjd.enableLimit = true;
			rjd.collideConnected = false;
			joint[i * 3 + 2] = (RevoluteJoint) getWorld().createJoint(rjd);
			con[i * 3 + 2] = new PIDController(body[i * 3 + 3], joint[i * 3 + 2], 0.3f, 0.001f);
		}
		
	}
	
	public class PIDController{
		float prevDiffAngle = 0f;
		Body body; 
		float targetAngle, currentAngle;
		RevoluteJoint myJoint;
		float Kp = 0;
		float Kd = 0;
		float Ki = 0;
		public PIDController(Body body, RevoluteJoint myJoint, float Kp, float Kd){
			this.body = body;
			this.myJoint = myJoint;
			this.currentAngle = myJoint.getJointAngle();
			this.Kp = Kp;
			this.Kd = Kd;
		}
		public PIDController(Body body, RevoluteJoint myJoint){
			this.body = body;
			this.myJoint = myJoint;
			this.currentAngle = myJoint.getJointAngle();
			this.Kp = 0.3f;
			this.Kd = 0.001f;
		}
		public void moveTo(float targetAngle){
			
			this.targetAngle = targetAngle;
			float angMomentum, P, I, D, diffAngle, derivDiffAngle, dt = 1/60f;
			float integDiffAngle = 0.0f;
			
			currentAngle = myJoint.getJointAngle() + MathUtils.PI;
			diffAngle =  targetAngle - currentAngle;
			integDiffAngle = integDiffAngle + diffAngle * dt;
			derivDiffAngle = (diffAngle - prevDiffAngle) / dt;

			P = Kp * diffAngle;
			I = Ki * integDiffAngle;
			D = Kd * derivDiffAngle;
			

			//System.out.println(diffAngle);
			
			//gravity compensation
			
			//System.out.println(diffAngle);
			Body bodyB;
			if(myJoint.m_bodyA.equals(body)){
				bodyB = myJoint.m_bodyB;
			} else {
				bodyB = myJoint.m_bodyA;
			}
			//Vec2 jointPos = new Vec2(0f, 0f);
			//jointPos.x = MathUtils.sin(rootJoint.m_bodyA.getAngle()) * MathUtils.abs(rootJoint.m_localAnchor1.y) + rootJoint.m_bodyA.getWorldCenter().x;
			//jointPos.y = MathUtils.cos(rootJoint.m_bodyA.getAngle()) * MathUtils.abs(rootJoint.m_localAnchor1.y) + rootJoint.m_bodyA.getWorldCenter().y;
			
			
			//Vec2 r = body.getWorldCenter().sub(jointPos);
			//System.out.println(rootJoint.m_bodyA.getAngle() * 180 / 3.14 + " " + rootJoint.m_localAnchor1.y);
			//float G = Vec2.cross(r, getWorld().getGravity().mul((-1) * body.getMass())) ;
			angMomentum = P + I + D;
			
			body.applyTorque(angMomentum);
			bodyB.applyTorque(-angMomentum);
			
			
			
			this.prevDiffAngle = diffAngle;
			//System.out.println(prevDiffAngle);
		}
	}
	
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		addTextLine("Click 'k' to enable/disable PID controller, click 'g' to enable/disable gravity" );
		//addTextLine("Keys: (l) limits, (m) motor, (a) left, (d) right");
		
		float dAngle = 0;
		//phase = time / timeStep;
		//dAngle = phase * targetAngle;
		
		if(MathUtils.abs( con[3].currentAngle - targetAngle) < 0.1f && phase <= 0){
			flag = 1;
			phase = 0.5f;
			targetAngle = 2 * MathUtils.PI -targetAngle;
			System.out.println(targetAngle);
		}
		if(phase > 0){
			phase -= 1/60f;
		}
		
		if(kick != 0) {
			con[0].moveTo(MathUtils.PI);
			con[1].moveTo(MathUtils.PI);
			con[2].moveTo(MathUtils.PI);
			con[3].moveTo(targetAngle);
			con[4].moveTo(MathUtils.PI);
			con[5].moveTo(MathUtils.PI);
		}
			//System.out.println(body2.m_angularVelocity);	
		time += 1/60f;
		if(time > timeStep){
		//	targetAngle *= (-1);
		//	time = 0;
		//	System.out.println(targetAngle);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#keyPressed(char, int)
	 */
	@Override
	public void keyPressed(char argKeyChar, int argKeyCode) {
		
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
			case 'k' :
				getModel().getKeys()['k'] = false;
				if(kick == 0){
					kick = 1;
				} else {
					kick = 0;
				}
				break;
			case 'g':
				getModel().getKeys()['g'] = false;
				if (gSwitch){
					getWorld().setGravity(new Vec2(0f,0f));
					gSwitch = false;
				} else {
					getWorld().setGravity(new Vec2(0f, -10f));
					gSwitch = true;
				}
				break;
		}
	}
	
	public void reset() {
		super.reset();
		kick = 0;
		targetAngle = MathUtils.PI * 7/6;
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Sticks";
	}
	
}
