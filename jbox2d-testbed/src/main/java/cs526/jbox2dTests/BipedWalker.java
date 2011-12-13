package cs526.jbox2dTests;

import java.util.HashMap;

import org.jbox2d.common.Vec2;
import org.jbox2d.testbed.framework.TestbedSettings;

import cs526.controls.PdController;
import cs526.controls.VirtualPdController;

public class BipedWalker extends AutoLoadedTest {

	@Override
	public void initTest(boolean argDeserialized) {

		frictionMotorTorque = 0.0f;
		DEFAULT_GRAVITY = 1.0f;
		// TODO Auto-generated method stub
		super.initTest(argDeserialized);

		super.setCamera(new Vec2(0f, 0f), 20f);

		virtualControls.put("torso",
				new VirtualPdController(model.getLinkByName("torso"), 0.125f,
						0.025f));
		virtualControls.put("l_up_leg",
				new VirtualPdController(model.getLinkByName("l_up_leg"),
						0.125f, 0.025f));
		virtualControls.put("r_up_leg",
				new VirtualPdController(model.getLinkByName("r_up_leg"),
						0.125f, 0.025f));
		
	}

	HashMap<String, PdController> virtualControls = new HashMap<String, PdController>();

	@Override
	public synchronized void step(TestbedSettings settings) {
		super.step(settings);
		
		float virtualTorque = 0.0f;
		int stateId = model.getCurrentStateId();
		
		float angle = (float) Math.toRadians(45-90);
		float zero = (float) Math.toRadians(-90);
		
		switch(stateId)
		{
		case 0:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("l_up_leg").moveTo(angle);
			model.getLinkByName("r_up_leg").applyTorque(-virtualTorque);
			break;
		case 1:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("l_up_leg").moveTo(zero);
			model.getLinkByName("r_up_leg").applyTorque(-virtualTorque);
			break;
		case 2:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("r_up_leg").moveTo(angle);
			model.getLinkByName("l_up_leg").applyTorque(-virtualTorque);
			break;
		case 3:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("r_up_leg").moveTo(zero);
			model.getLinkByName("l_up_leg").applyTorque(-virtualTorque);
			break;
			
			
		}
		
	}

}
