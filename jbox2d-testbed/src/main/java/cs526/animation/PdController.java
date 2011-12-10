package cs526.animation;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.joints.RevoluteJoint;

/**
 * Represents a PD Controller. <br/> 
 * <br/>
 * The standard equation to control the torque of a joint using a PD controller is: <br/>
 *	<blockquote> torque = k_s * (angle_d - angle) - k_d * anglar_velocity, </blockquote>
 *	where <em>k_s</em> is the spring coefficient, <em>k_d</em> is the damper coefficient, 
 *	<em>angle_d </em> is the desired joint angle, <em>angle </em> is the current joint angle, 
 *	and <em>angluar_velocity</em> is the current angular velocity of the joint.<br/> 
 *	
 *	<p><b>Note:</b> all the angles (angular velocity) are represent in radian.</p>
 */
public class PdController{
	float prevDiffAngle = 0f;
	Body body; 
	public float targetAngle, currentAngle;
	RevoluteJoint myJoint;
	float Kp = 0;
	float Kd = 0;
	float Ki = 0;
	public float torque; 
	public PdController(Body body, RevoluteJoint myJoint, float Kp, float Kd){
		this.body = body;
		this.myJoint = myJoint;
		this.currentAngle = myJoint.getJointAngle();
		this.Kp = Kp;
		this.Kd = Kd;
	}
	public PdController(Body body, RevoluteJoint myJoint){
		this(body, myJoint, 0.3f, 0.001f);
	}
	public void moveTo(float targetAngle){
		
		this.targetAngle = targetAngle;
		float angMomentum, P, I, D, diffAngle, derivDiffAngle, dt = 1/60f;
		float integDiffAngle = 0.0f;
		
		currentAngle = myJoint.getJointAngle();
		diffAngle =  targetAngle - currentAngle;
		integDiffAngle = integDiffAngle + diffAngle * dt;
		derivDiffAngle = myJoint.getJointSpeed();
		
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
		torque = P -  D;
		
		body.applyTorque(-torque);
		bodyB.applyTorque(torque);
		
		
		this.prevDiffAngle = diffAngle;
		//System.out.println(prevDiffAngle);
	}
}
