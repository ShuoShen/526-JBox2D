package cs526.jbox2dTests;

import java.util.HashMap;

import org.jbox2d.collision.Manifold;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.testbed.framework.TestbedSettings;

import cs526.controls.PdController;
import cs526.controls.VirtualPdController;

public class Kicker extends AutoLoadedTest {

	public void initTest(boolean argDeserialized) {

		frictionMotorTorque = 0.0f;
		DEFAULT_GRAVITY = 10.0f;
		// TODO Auto-generated method stub
		super.initTest(argDeserialized);

		super.setCamera(new Vec2(0f, 0f), 20f);

		virtualControls.put("torso",
				new VirtualPdController(model.getLinkByName("torso"),300f,
						30f));
		virtualControls.put("l_up_leg",
				new VirtualPdController(model.getLinkByName("l_up_leg"),
						300f, 30f));
		virtualControls.put("r_up_leg",
				new VirtualPdController(model.getLinkByName("r_up_leg"),
						300f, 30f));
	}

	HashMap<String, PdController> virtualControls = new HashMap<String, PdController>();

	@Override
	public synchronized void step(TestbedSettings settings) {
		super.step(settings);

		float virtualTorque = 0.0f;
		int stateId = model.getCurrentStateId();

		float angle = (float) Math.toRadians(35);
		float m_angle = (float) Math.toRadians(0);
		float zero = (float) Math.toRadians(0);

		
		float com = model.getLinkByName("torso").getWorldCenter().x;
		float stanceKnee = com;
		if (!leftSwing)
			stanceKnee = model.getLinkByName("l_up_leg").getWorldCenter().x;

		addTextLine(String.format("%f", stanceKnee - com));

		switch (stateId) {
		case 0:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("l_up_leg").moveTo(angle);
			model.getLinkByName("r_up_leg").applyTorque(-virtualTorque);
			break;
		case 1:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("l_up_leg").moveTo(m_angle);

			model.getLinkByName("r_up_leg").applyTorque(-virtualTorque);
			if (rightSwing)
				model.nextState();
			break;
		case 2:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("r_up_leg").moveTo(angle);
			model.getLinkByName("l_up_leg").applyTorque(-virtualTorque);
			break;
		case 3:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("r_up_leg").moveTo(m_angle);

			model.getLinkByName("l_up_leg").applyTorque(-virtualTorque);
			if (leftSwing)
				model.nextState();
			break;

		}

	}

	boolean leftSwing = true;
	boolean rightSwing = true;

	@Override
	public void preSolve(Contact contact, Manifold oldManifold) {
//		// TODO Auto-generated method stub
//		super.preSolve(contact, oldManifold);
//
//		// System.out.println("contact");
//		Body b = contact.m_fixtureB.getBody();
//		Body leftFoot = model.getLinkByName("l_foot");
//		Body rightFoot = model.getLinkByName("r_foot");
//
//		if (b == leftFoot && leftSwing
//				&& leftFoot.getWorldCenter().x > rightFoot.getWorldCenter().x) {
//			leftSwing = false;
//			rightSwing = true;
//			System.out.println("left foot contact");
//		}
//
//		else if (b == rightFoot && rightSwing
//				&& leftFoot.getWorldCenter().x < rightFoot.getWorldCenter().x) {
//			rightSwing = false;
//			leftSwing = true;
//			System.out.println("right foot contact");
//		}

	}

}
