package cs526.simu;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.World;



public class LampObjectiveFunction implements CountedObjectiveFunction{

	SimulatedLampModel lamp;
	
	final static int HZ = 60;
	final static int SECS = 30;
	public LampObjectiveFunction()
	{
		
	}
		
	int count = 0;
	
	@Override
	public int evalCount() {
		// TODO Auto-generated method stub
		return count;
	}


	@Override
	public boolean isFeasible(double[] arg0) {
		return true;
	}

	@Override
	public double valueOf(double[] params) {
		count++;
		lamp = getLamp(params);
		while(lamp.getStepCount() <= HZ * SECS)
		{
			lamp.step(HZ);
		}
		return -lamp.getComX();
	}
	
	private static SimulatedLampModel getLamp(double[] params) {
		Vec2  gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		World world = new World(gravity, doSleep);
				
		SimulatedLampModel lamp = new SimulatedLampModel(world, params);
		lamp.initTest();
		return lamp;
	}

}
