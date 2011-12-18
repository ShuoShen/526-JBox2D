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
package cs526.jbox2dTests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
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
public abstract class AutoLoadedTest extends TestbedTest {

	@Override
	public boolean isSaveLoadEnabled() {
		return true;
	}

	float gravity = 0.0f;
	float DEFAULT_GRAVITY = 10.0f;
	float frictionMotorTorque = 10.0f;
	CharacterInfo modelInfo;
	CharacterModel model;

	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest(boolean)
	 */
	@Override
	public void initTest(boolean argDeserialized) {
		if (argDeserialized) {
			return;
		}
		createGround();
		createCharater();
	}

	private void createGround() {
		{
			BodyDef bd = new BodyDef();
			Body ground = getWorld().createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
			getWorld().setGravity(new Vec2(0, 0f));
		}
	}

	private void createCharater() {
		modelInfo = new CharacterInfo(this);
		model = new CharacterModel(getWorld(), modelInfo, frictionMotorTorque);
	}

	@Override
	public synchronized void step(TestbedSettings settings) {
		super.step(settings);
		addTextLine("Press 'g' to toggle gravity");
		addTextLine("Press 'k' to activate move.");
//		addTextLine("Press 'd' to deactivate move.");
		addTextLine("Press 'n' to next state.");
		addTextLine("Current state is :" + model.getCurrentStateId());
		addTextLine("elapsed time is :" + model.getElapsedCurrentTime());

		model.driveToDesiredState();
	}

	@Override
	public void keyPressed(char key, int argKeyCode) {
		switch (key) {
		case 'g':
			gravity = DEFAULT_GRAVITY - gravity;
			getWorld().setGravity(new Vec2(0, -gravity));
			break;

		case 'k':
			getModel().getKeys()['k'] = false;
			// model.nextState();
			if (model.isActivated())
				model.deactivateMotion();
			else
				model.activateMotion();
			break;


			
		case 'n':
			getModel().getKeys()['n'] = false;
			System.out.println("n");
			model.nextState();
			break;
		}
		
	}

	@Override
	public String getTestName() {
		return this.getClass().getSimpleName();
	}

}
