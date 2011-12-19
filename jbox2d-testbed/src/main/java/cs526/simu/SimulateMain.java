package cs526.simu;

import java.awt.List;
import java.util.ArrayList;
import java.util.Random;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;

public class SimulateMain {
	public static void main(String[] args)
	{
//		SimulatedBipedWalker walker = getWalker();
		int hz = 60;
		int seconds = 30;
		
		SimulatedBipedWalker walker = getWalker(new float[] { -5f, 0.000000f, 0});
		System.out.println(walkForSeconds(walker, hz, seconds));
		
		hillClimbing(new float[] {-5f, 0f, 0.0f}, 2f, 100, hz, seconds);
		
		
		System.out.println("done");
		
//		
//		System.out.println("done");
//		for (float[] neighbour : neighbours(dths))
//		{
//			for (int i = 0; i < neighbour.length; i++)
//			{
//				System.out.printf("%.2f, ", neighbour[i]);
//			}
//			System.out.println();
//		}
	}
	private static float walkForSeconds(SimulatedBipedWalker walker, int hz, int seconds)
	{
		boolean flew = false;
		while(walker.getStepCount() <= hz * seconds)
		{
			walker.step(hz);
			
			if (walker.getComY() > 1.5f)
			{
				flew = true;
				break;
			}
			
//			if (walker.getStepCount() % hz == 0)
//			{
////				System.out.printf("com x %2.2f\n", walker.getComX());
//			}
		}
		if (flew)
			return Float.MIN_VALUE;
		else
			return walker.getComX();
	}
	
	private static SimulatedBipedWalker getWalker(float[] dths) {
		Vec2  gravity = new Vec2(0.0f, -1.0f);
		boolean doSleep = true;
		World world = new World(gravity, doSleep);
				
//		float[] dths = new float[] {20, 1, 1};
		SimulatedBipedWalker walker = new SimulatedBipedWalker(world, dths);
		walker.initTest();
		return walker;
	}
	
	public static void hillClimbing(float[] start, float stepLimit, int attempts, int hz, int seconds)
	{
	
		int iterations = 100;
		
		float[] current = start.clone();
		SimulatedBipedWalker walker = getWalker(current);
		float currentEval = walkForSeconds(walker, hz, seconds);
		for (int i = 0; i < iterations; i++)
		{
			float[] maxNode = current;
			float maxEval = currentEval;
			for (float[] next : neighbours(current, stepLimit, attempts))
			{
				walker = getWalker(next);
				float nextEval = walkForSeconds(walker, hz, seconds);
				if (nextEval > maxEval)
				{
					maxNode = next;
					maxEval = nextEval;
				}
			}
			if (maxNode == current)
			{
				break;
			}
			else
			{
				System.out.printf("iteration %d, maxEval %.2f ", i, maxEval);
				for (float f : maxNode)
				{
					System.out.printf("%f, ", f);
				}
				System.out.println();
				current = maxNode;
				currentEval = maxEval;
			}
		}
	}
	
	public static ArrayList<float[]> neighbours(float[] current, float stepLimit, int attempts)
	{
		
		
		ArrayList<float[]> results = new ArrayList<float[]>();
//		float[] steps = new float[]{-.5f, .5f};
		
		for (int i = 0; i < current.length; i++)
		{
			for (int j = 0; j < attempts; j++)
			{
				Random random = new Random();
				
				float[] next = current.clone();
				next[i] = current[i] + stepLimit * random.nextFloat(); 
				results.add(next);
			}
		}
		return results;
	}
}
