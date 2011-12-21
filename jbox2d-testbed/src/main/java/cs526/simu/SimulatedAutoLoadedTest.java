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
 * Created at 7:50:04 AM Jan 20, 2011
 */
package cs526.simu;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

import cs526.models.CharacterInfo;
import cs526.models.CharacterModel;

/**
 * Simply extend this class and create a input file with the same name as the
 * subclass. The subclass will automatically load character from the input file,
 * and create it in the physics world. <br/>
 * 
 * <br/>
 * See the sample file LampModel
 * 
 * @author Shuo Shen
 */
public abstract class SimulatedAutoLoadedTest implements ContactListener {


	float gravity = 0.0f;
	float DEFAULT_GRAVITY = 10.0f;
	float frictionMotorTorque = 10.0f;
	CharacterInfo modelInfo;
	CharacterModel model;
World world;

	public World getWorld()
	{
		return world;
	}

	public SimulatedAutoLoadedTest(World world)
	{
		this.world = world;
	}
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest(boolean)
	 */
	
	public void initTest() {
		getWorld().setContactListener(this);
		createGround();
		createCharater();
	}

	private void createGround() {
		{
			BodyDef bd = new BodyDef();
			Body ground = getWorld().createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(4000000.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
			getWorld().setGravity(new Vec2(0, -gravity));
		}
	}

	private void createCharater() {
		modelInfo = new CharacterInfo(this);
		model = new CharacterModel(getWorld(), modelInfo, frictionMotorTorque);
	}

	int stepCount = 0;
	public int getStepCount()
	{
		return stepCount;
	}
	
	public void step(int hz) {
		float timeStep = 1f/ hz; 
		
		getWorld().step(timeStep,
				8,
				3);
		
		stepCount++;
		
		model.driveToDesiredState((int)hz);
		
		
	}

	@Override
	public void beginContact(Contact contact) {
				
	}

	@Override
	public void endContact(Contact contact) {
				
	}

	@Override
	public void preSolve(Contact contact, Manifold oldManifold) {
				
	}

	@Override
	public void postSolve(Contact contact, ContactImpulse impulse) {
		
	}

	

}
