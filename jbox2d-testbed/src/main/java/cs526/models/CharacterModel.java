package cs526.models;

import java.util.HashMap;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

import cs526.controls.PdController;
import cs526.utilities.ControllerInfo;
import cs526.utilities.DesiredState;
import cs526.utilities.JointInfo;
import cs526.utilities.LinkInfo;
import cs526.utilities.LinkPosition;

/**
 * 
 * Represents a character in a physical world. It contains all the information 
 * of characters for animation such as links, joints, controllers, desired states, etc.
 *  
 * @author shuo
 *
 */
public class CharacterModel {
	
	private static final float MOTOR_TORQUE = 10.0f;
	private HashMap<String, Body> links;
	HashMap<String, RevoluteJoint> joints;
	HashMap<String, PdController> controllers;
	private World world;
	private CharacterInfo characterInfo;
	private int currentStateId = -1;
	
	public CharacterModel(World world, CharacterInfo info)
	{
		this.world = world;
		this.characterInfo = info;
		links = createLinks(this.world, characterInfo);
		joints = createJoints(this.world, characterInfo, links);
		controllers = createControllers(this.world, characterInfo, joints);
	}
	
	public Body getLinkByName(String linkName)
	{
		return links.get(linkName);
	}
	
	public RevoluteJoint getJointByName(String jointName)
	{
		return joints.get(jointName);
	}
	
	public PdController getControllerByName(String controllerName)
	{
		return controllers.get(controllerName);
	}
	public void nextState()
	{
		currentStateId = (currentStateId + 1) % characterInfo.nStates();
	}
	public void driveToDesiredState()
	{
		
		DesiredState state = getCurrentDesiredState();
		if (state == null)
			return;
		for (String key : state.getJointNames())
		{
			PdController controller = getControllerByName(key);
			float desiredAngle = state.getAngleByJointName(key);
			controller.moveTo(desiredAngle);
		}
	}
	
	public DesiredState getCurrentDesiredState()
	{
		if (currentStateId < 0)
			return null;
		return characterInfo.getState(currentStateId);
	}
	
	private HashMap<String, PdController> createControllers(World world, CharacterInfo modelInfo, HashMap<String, RevoluteJoint> joints)
	{
		HashMap<String, PdController> controllers = new HashMap<String, PdController>();
		
		for (String key : modelInfo.getControllerNames())
		{
			ControllerInfo cInfo = modelInfo.getControllerInfoByJointName(key);
			PdController controller = new PdController(joints.get(key), cInfo.ks, cInfo.kd);
			controllers.put(key, controller);
		}
		return controllers;
	}
	
	private HashMap<String, RevoluteJoint> createJoints(World world, CharacterInfo modelInfo, HashMap<String, Body> links)
	{
		HashMap<String, RevoluteJoint> joints = new HashMap<String, RevoluteJoint>();
		
		for (String jointName : modelInfo.getJointNames()){
			JointInfo jointInfo = modelInfo.getJointInfoByName(jointName);
			RevoluteJointDef jointDef = new RevoluteJointDef();
			Body link1 = getLinkByName(jointInfo.linkA);
			Body link2 = getLinkByName(jointInfo.linkB);
			LinkInfo link1Info = modelInfo.getLinkInfoByName(jointInfo.linkA);
			LinkInfo link2Info = modelInfo.getLinkInfoByName(jointInfo.linkB);
			jointDef.bodyA = link1;
			jointDef.bodyB = link2;
			jointDef.localAnchorA = computeLinkPositionVector(jointInfo.linkAPosition, link1Info.width, link1Info.height);
			jointDef.localAnchorB = computeLinkPositionVector(jointInfo.linkBPosition, link2Info.width, link2Info.height);
			
			jointDef.upperAngle = (float) Math.toRadians(jointInfo.upperAngleDegree);
			jointDef.lowerAngle = (float) Math.toRadians(jointInfo.lowerAngleDegree);
			
			jointDef.enableLimit = true;
			jointDef.maxMotorTorque = MOTOR_TORQUE;
			jointDef.enableMotor = true;
			
			joints.put(jointName, (RevoluteJoint) world.createJoint(jointDef));
		}
		
		return joints;
	}
	
	private HashMap<String, Body> createLinks(World world, CharacterInfo info)
	{
		HashMap<String, Body> links = new HashMap<String, Body>();
		BodyDef bodyDef = new BodyDef();
		bodyDef.type = BodyType.DYNAMIC;
		bodyDef.position.set(0.0f, 1.0f);
		

		for (String linkName : info.getLinkNames()){
			LinkInfo bodyInfo = info.getLinkInfoByName(linkName);
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(bodyInfo.width / 2, bodyInfo.height / 2);

			FixtureDef fixture = new FixtureDef();
			fixture.shape = shape;
			fixture.density = bodyInfo.mass
					/ (bodyInfo.width * bodyInfo.height);
			fixture.friction = 0.8f;
			fixture.restitution = 0.2f;

			Body link = world.createBody(bodyDef);
			link.createFixture(fixture);
			
			links.put(linkName, link);
		}
		return links;
	}
	
	private Vec2 computeLinkPositionVector(LinkPosition jointPosition,
			float width, float height) {
		Vec2 vector = null;

		float dist = width / 2 - height / 2;
		if (jointPosition == LinkPosition.Middle) {
			vector = new Vec2(0f, 0f);
		} else if (jointPosition == LinkPosition.Left) {
			vector = new Vec2(-dist, 0f);
		} else if (jointPosition == LinkPosition.Right) {
			vector = new Vec2(dist, 0f);
		}
		return vector;
	}
}
