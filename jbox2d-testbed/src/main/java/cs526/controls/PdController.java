package cs526.controls;

import org.jbox2d.dynamics.Body;


public abstract class PdController {
	float prevDiffAngle = 0f;
	Body body; 
	public float targetAngle, currentAngle;

	float Ks = 0;
	float Kd = 0;
	float Ki = 0;
	public float torque; 
	
	public float moveTo(double targetAngle){
		
		this.targetAngle = (float)targetAngle;
		float P,  D, diffAngle, derivDiffAngle;
				
		currentAngle = getCurrentAngle();
		diffAngle =  (float)targetAngle - currentAngle;
		
		derivDiffAngle = getAngularVelocity();
		
		P = Ks * diffAngle;
		D = Kd * derivDiffAngle;
		torque = P -  D;

		enforceTorque();
		
		return torque;	
	}
	
	protected abstract void enforceTorque();
	
	protected abstract float getAngularVelocity();
		
	protected abstract float getCurrentAngle();
	
}
