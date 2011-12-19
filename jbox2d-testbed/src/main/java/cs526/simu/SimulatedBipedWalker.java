package cs526.simu;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintStream;
import java.util.HashMap;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.testbed.framework.TestbedSettings;

import cs526.controls.PdController;
import cs526.controls.VirtualPdController;

public class SimulatedBipedWalker extends SimulatedAutoLoadedTest {

	public SimulatedBipedWalker(World world, float[] dths)
	{
		super(world);
		this.dths = dths.clone();
	}
	
	// 
	/**
	 * compensate angles
	 */
	float[] dths ;
	
	@Override
	public void initTest() {

		for (int i=0; i < dths.length ; i++)
		{
			dths[i] = (float) Math.toRadians(dths[i]);
		}
		
		frictionMotorTorque = 0.0f;
		DEFAULT_GRAVITY = 1.0f;
		gravity = 1.0f;

		//
		Vec2[] vertices = new Vec2[3];
		vertices[0] = new Vec2(0.0f, 0.0f);
		vertices[1] = new Vec2(0.1f, 0.0f);
		vertices[2] = new Vec2(0.03f, 0.1f);
		int count = 3;
		CircleShape circle = new CircleShape();
		float radius = 0.08f;
		circle.m_radius = radius;

		BodyDef bd = new BodyDef();
		bd.position = new Vec2(1.0f, radius);
		roadBlock = getWorld().createBody(bd);
		bd.type = BodyType.DYNAMIC;
//		roadBlock.createFixture(circle, 2.0f);

		// TODO Auto-generated method stub
		super.initTest();


		virtualControls.put("torso",
				new VirtualPdController(model.getLinkByName("torso"), 0.125f,
						0.025f));
		virtualControls.put("l_up_leg",
				new VirtualPdController(model.getLinkByName("l_up_leg"),
						0.125f, 0.025f));
		virtualControls.put("r_up_leg",
				new VirtualPdController(model.getLinkByName("r_up_leg"),
						0.125f, 0.025f));

		Body torso = getTorso();
		torso.setActive(true);
			
//		File file = new File("data.csv");
//		
//		try {
//			writer = new PrintStream(file);
//		}catch (Exception e) {
//			// TODO: handle exception
//		}
//		writer.println("stance_anckle, swing_anckle, com, com-stance, com-swing, velocity, angle_compensation");
				
	}
	PrintStream writer;
		
	private Body roadBlock;

	HashMap<String, PdController> virtualControls = new HashMap<String, PdController>();

	private float compensateAngle(float d, float v) {
		float cd = 0.3f;
		float cv = 0.0f;
		float result = cd * d + cv * v;
		return 0.0f;
	}

	@Override
	public synchronized void step(int hz) {
		super.step(hz);

		float virtualTorque = 0.0f;
		int stateId = model.getCurrentStateId();

		float angle = (float) Math.toRadians(35);
		float m_angle = (float) Math.toRadians(0);
		float zero = (float) Math.toRadians(0);

		Body torso = getTorso();
		// torso.setActive(true);
		float velocity = torso.m_linearVelocity.x;

		com = getComX();
		if (leftSwing && !rightSwing) {
			stanceAnkleX = getRightAnklePosX();
			swingAnkleX = getLeftAnklePosX();
		} else if (rightSwing && !leftSwing) {
			stanceAnkleX = getLeftAnklePosX();
			swingAnkleX = getRightAnklePosX();
		}

		

		float sumOfXDiffer = 2 * com - stanceAnkleX - swingAnkleX;
		float differStanceSwing = stanceAnkleX - swingAnkleX;
		
		float compensateAngle = compensateAngle(2*com - stanceAnkleX - swingAnkleX, velocity);
		float timeStepScale = compensateAngle / angle + 1;

		angle += compensateAngle;
		m_angle += compensateAngle;

		model.scaleStepTime(timeStepScale);
		

		// addTextLine(String.format("timestep scale is: %2.2f",
		// (float)timeStepScale));
		
		
//		if (getStepCount() % 6 == 0)
////		writer.println("stance_anckle, swing_anckle, com, com-stance, com-swing, velocity, angle_compensation");
//			writer.printf("%2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f\n", stanceAnkleX, swingAnkleX, com, com-stanceAnkleX, com-swingAnkleX, velocity, compensateAngle);
		
		
		switch (stateId) {
		case 0:
			
			virtualTorque = virtualControls.get("torso").moveTo(zero );
			virtualTorque += virtualControls.get("l_up_leg").moveTo(angle + dths[0]);
			model.getLinkByName("r_up_leg").applyTorque(-virtualTorque);
			model.setNewTargetAngleCompensate("l_knee", dths[1]);
			
			break;
		case 1:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("l_up_leg").moveTo(m_angle + dths[2]);
			model.getLinkByName("r_up_leg").applyTorque(-virtualTorque);
			if (rightSwing && !leftSwing)
				model.nextState();
			
			break;
		case 2:
			virtualTorque = virtualControls.get("torso").moveTo(zero );
			virtualTorque += virtualControls.get("r_up_leg").moveTo(angle + dths[0]);
			model.getLinkByName("l_up_leg").applyTorque(-virtualTorque);
			model.setNewTargetAngleCompensate("r_knee", dths[1]);
			
			break;
		case 3:
			virtualTorque = virtualControls.get("torso").moveTo(zero);
			virtualTorque += virtualControls.get("r_up_leg").moveTo(m_angle + dths[2]);

			model.getLinkByName("l_up_leg").applyTorque(-virtualTorque);
			if (leftSwing && !rightSwing)
				model.nextState();
			
			break;

		}

	}

	
	boolean leftSwing = true;
	boolean rightSwing = true;
	float com = 0.0f;
	float stanceAnkleX = 0.0f;
	float swingAnkleX = 0.0f;

	@Override
	public void preSolve(Contact contact, Manifold oldManifold) {
		// TODO Auto-generated method stub
		super.preSolve(contact, oldManifold);

		Body a = contact.m_fixtureA.getBody();
		// System.out.println("contact");
		Body b = contact.m_fixtureB.getBody();
		Body leftFoot = model.getLinkByName("l_foot");
		Body rightFoot = model.getLinkByName("r_foot");

		if (a == roadBlock)
			System.out.println("road block");
		if (!model.isActivated())
			model.activateMotion();
		
		if (b == leftFoot && leftSwing
				&& leftFoot.getWorldCenter().x > rightFoot.getWorldCenter().x) {
			leftSwing = false;
			rightSwing = true;
			// com = torso.getWorldPoint(new Vec2(0, -0.2f)).x;
			// stanceKnee = ;
//			System.out.println("left foot contact");

		}

		else if (b == rightFoot && rightSwing
				&& leftFoot.getWorldCenter().x < rightFoot.getWorldCenter().x) {
			rightSwing = false;
			leftSwing = true;
			// com = torso.getWorldPoint(new Vec2(0, -0.2f)).x;
			// stanceKnee = ;
//			System.out.println("right foot contact");
		}

	}

	private Body getTorso() {
		return model.getLinkByName("torso");
	}

	public float getComX() {
		Body torso = model.getLinkByName("torso");
		return torso.getWorldPoint(new Vec2(0, -0.2f)).x;
	}
	
	public float getComY() {
		Body torso = model.getLinkByName("torso");
		return torso.getWorldPoint(new Vec2(0, -0.2f)).y;
	}

	public float getLeftAnklePosX() {
		return model.getLinkByName("l_bottom_leg").getWorldPoint(
				new Vec2(0, -0.25f)).x;
	}

	public float getRightAnklePosX() {
		return model.getLinkByName("r_bottom_leg").getWorldPoint(
				new Vec2(0, -0.25f)).x;
	}

}
