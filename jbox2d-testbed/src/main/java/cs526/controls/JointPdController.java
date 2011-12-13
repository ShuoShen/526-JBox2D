package cs526.controls;

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
public class JointPdController extends PdController{
	RevoluteJoint myJoint;
	
	
	public JointPdController(RevoluteJoint myJoint, float Ks, float Kd){
		this.body = myJoint.m_bodyA;
		this.myJoint = myJoint;
		this.currentAngle = myJoint.getJointAngle();
		this.Ks = Ks;
		this.Kd = Kd;
	}
	
	public JointPdController(RevoluteJoint myJoint){
		this(myJoint, 0.3f, 0.001f);
	}
	
				
	@Override
	protected void enforceTorque()
	{
		Body bodyB;
		
		if(myJoint.m_bodyA.equals(body)){
			bodyB = myJoint.m_bodyB;
		} else {
			bodyB = myJoint.m_bodyA;
		}
		bodyB.applyTorque(torque);
		body.applyTorque(-torque);
	}
	
	@Override
	protected float getAngularVelocity() {
		return myJoint.getJointSpeed();
	}
	@Override
	protected float getCurrentAngle() {
		return myJoint.getJointAngle();
	}
}
