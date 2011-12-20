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

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.framework.TestbedSettings;



/**
 * @author Shuo Shen
 */
public class SimulatedLampModel extends SimulatedAutoLoadedTest {
	
	float[] stepChange = {0.185806f, 0.267656f, 0.092896f};
	
	public SimulatedLampModel(World world, float[] stepChange)
	{
		super(world);
		this.stepChange = stepChange.clone();  
	}
	
	@Override
	public void initTest() {
		gravity = DEFAULT_GRAVITY - gravity;
		// TODO Auto-generated method stub
		super.initTest();
		model.activateMotion();
	}
	
	public float getComX()
	{
		Body torso = model.getLinkByName("b1");
		return torso.getWorldPoint(new Vec2(0, -0.2f)).x;
	}
		
	
	@Override
	public synchronized void step(int hz) {
		// TODO Auto-generated method stub
		super.step(hz);
		
		int stateId = model.getCurrentStateId();
		model.changeStepTime(stepChange[stateId]);
	}
	


}
