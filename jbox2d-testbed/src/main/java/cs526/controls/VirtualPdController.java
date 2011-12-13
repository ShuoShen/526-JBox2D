package cs526.controls;

import org.jbox2d.dynamics.Body;

public class VirtualPdController extends PdController {

	public VirtualPdController(Body body, float ks, float kd) {
		this.body = body;
		this.Ks = ks;
		this.Kd = kd;
	}

	@Override
	protected void enforceTorque() {
		body.applyTorque(torque);

	}

	@Override
	protected float getAngularVelocity() {
		return body.getAngularVelocity();

	}

	@Override
	protected float getCurrentAngle() {
		return body.getAngle();
	}

}
