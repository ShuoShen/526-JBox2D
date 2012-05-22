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

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.testbed.framework.TestbedSettings;

import cs526.utilities.LampStatesConverter;



/**
 * @author Shuo Shen
 */
public class LampModel extends AutoLoadedTest {
	
	float[] stepChange = {0f, 0f, 0f};
	
	static double[] stateParams = {
		
//		0.3, 2.268928, -2.094395, 0.3, 1.745329, -0.523599, 0.3, 1.745329, -2.094395};  // original
		//-2.706996, 6.022684, -2.461829, -1.708927, -2.789764, 6.248480, -4.451129, -2.648830, -4.306936}; // 180 garbage


//		0.300275, 2.261925, -2.084589, 0.297431, 1.701422, -0.310756, 0.298053, 1.802196, -2.118483 } ;//  32.97 gradient
		// 21.600599, -62.135024, -5.084821, 46.146903, 24.402739, 28.434325, -10.389204, 1.504404, -1.334109};
//		0.311284, 2.284139, -2.578416, 0.194784, 0.850898, 1.027319, 0.220233, 1.870379, -1.458678};
//		-34.364154, -65.970284, -9.959347, 323.522160, -66955.999269, -72404.677423, 245.342801, -78.825845, -119.484074 }; // gradient descent 
	
		0.339816, 1.851503, -3.209133, 0.553538, 2.039530, 0.427434, 0.592207, 1.125476, -1.559522 }; // 110
		
//		0.998946, 4.531613, -7.407768, 1.526571, -7.687272, 4.688313, -2.400771, 3.260281, -1.656138}; // -147
	
	//	0.300000, 2.268928, -2.094395, 0.300000, 1.745329, -0.424447, 0.215957, 1.745329, -2.159873 }; // 33-hill
		
	
	
//		0.300000, 1.958411, -2.054179, 0.300000, 1.651179, -0.523599, 0.300000, 1.745329, -2.155991}; // 47-hill
		
		// 0.300000, 2.085769, -2.094395, 0.300000, 1.607433, -0.457843, 0.300000, 1.699918, -2.150697}; // 41
		
//		 0.300000, 2.176279, -2.075273, 0.216626, 1.745329, -0.485321, 0.300000, 1.723036, -2.052137 }; // 34
		
//		0.689683, 2.054897, -2.391576, 0.492408, 1.596347, -0.120926, -0.058541, 2.013426, -1.919496};
	
	//	0.446386, 2.205529, -1.892719, 0.958075, 1.392581, 1.054715, 0.231314, 2.052316, -2.953275};  // 56 
		
		 //0.949520 + 0.185806, 2.243547, -2.045300, -0.016442 + 0.267656, 1.495176, 1.153491, 0.315811 + 0.092896, 1.626707, -2.988626 }; // 78  
		// 1.207595, 3.711536, -3.743792, 0.639098, -1.050229, 1.652925, -0.978264, 3.466270, -3.525436 }; // 117  
		
	//	-0.372368, 0.716507, -1.505375, 0.306718, -0.063691, -0.450312, 1.422759, 2.792528, -2.466536} ; // 100 +
		
		//0.740296, 2.449755, -2.606241, -0.054901, 1.000824, 0.002220, 0.348619, 1.428438, -3.047047};  // 70 + 
		
		//0.300000, 2.064956, -2.094395, 0.203720, 1.629473, -0.142652, 0.113540, 1.745329, -2.094395};  // 50+
//		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; 
	
	public LampModel()
	{
		
		super(new LampStatesConverter(), stateParams);
	}
	
	@Override
	public void initTest(boolean argDeserialized) {
		gravity = DEFAULT_GRAVITY - gravity;
		// TODO Auto-generated method stub
		super.initTest(argDeserialized);
		model.activateMotion();
		
	}
	
	public float getComX()
	{
		Body torso = model.getLinkByName("b1");
		return torso.getWorldPoint(new Vec2(0, 0.0f)).x;
	}
	
	
	@Override
	public synchronized void step(TestbedSettings settings) {
		// TODO Auto-generated method stub
		super.step(settings);
		
		int stateId = model.getCurrentStateId();
		model.changeStepTime(stepChange[stateId]);
		
		if (getStepCount() == 60 * 30)
			System.out.println("Distance is " + getComX());
	}

	@Override
	public String getTestName() {
		return "Lamp Model";
	}

}
