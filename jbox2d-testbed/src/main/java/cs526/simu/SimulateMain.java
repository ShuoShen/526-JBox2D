package cs526.simu;

import java.awt.List;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;

import fr.inria.optimization.cmaes.fitness.IObjectiveFunction;

public class SimulateMain {
	public static void main(String[] args)
	{
//		SimulatedBipedWalker walker = getWalker();
		int hz = 60;
		int seconds = 30;
		
		double[] start = new double[] {
				0.3f, 
				(float) Math.toRadians(130),
				(float) Math.toRadians(-120),
				0.3f,
				(float) Math.toRadians(100),
				(float) Math.toRadians(-30),
				0.3f,
				(float) Math.toRadians(100),
				(float) Math.toRadians(-120)
		};
		System.out.println("initial point: \n");
		for (int i = 0; i < start.length; i++)
		{
			System.out.printf("%f, ", start[i]);
		}
		
		
		System.out.println();
		
		
		PrintStream print = null; 
		try {
			print = new PrintStream(new File("result.csv"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
				
		int iter = 1;
		ArrayList<Double> results = new ArrayList<Double>();
		ArrayList<Integer> evalCounts = new ArrayList<Integer>();
		
		for (int it = 0; it < iter; it++)
		{
			LampObjectiveFunction func = new LampObjectiveFunction();
			Optimizer optimizer = new GradientDecentOptimizer();
			
			
			double[] best = optimizer.optimize(func, start, 100);
			
			for (int i = 0; i < best.length; i++)
			{
				System.out.printf("%f, ", best[i]);
			}
			
			double eval = func.valueOf(best);
			
			
			System.out.println();
			results.add(eval);
			evalCounts.add(func.evalCount());
		}
		
		for (double d : results)
		{
			print.printf("%f , ", d);
			
		}
		print.println();
		
		for (int i : evalCounts)
		{
			print.printf("%d , ", i);
		}
		
		try{
			print.close();
		}
		catch (Exception ex)
		{
			
		}
		
//		SimulatedLampModel lamp = getLamp(start);
//		
//		System.out.println(jumpForSeconds(lamp, hz, seconds));
//		
////		SimulatedBipedWalker walker = getWalker(new float[] { -5f, 0f, 0f});
////		System.out.println(walkForSeconds(walker, hz, seconds));
//		
////		hillClimbing(new float[] {-5f, 0f, 0.0f}, 1f, 30, hz, seconds);
//		hillClimbingForLamp(start, 0.1f, 30, hz, seconds);
		
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
	
	private static SimulatedBipedWalker getWalker(double[] dths) {
		Vec2  gravity = new Vec2(0.0f, -1.0f);
		boolean doSleep = true;
		World world = new World(gravity, doSleep);
				
//		float[] dths = new float[] {20, 1, 1};
		SimulatedBipedWalker walker = new SimulatedBipedWalker(world, dths);
		walker.initTest();
		return walker;
	}
	
	private static float jumpForSeconds(SimulatedLampModel lamp, int hz, int seconds)
	{
		while(lamp.getStepCount() <= hz * seconds)
		{
			
			lamp.step(hz);
		}
		return lamp.getComX();
	}
	
	
	/**
	 * @param params
	 * 	 * @param params
	 * s1, s2, s3 (seconds)
	 * j11, j12, j21, j22, j31, j32 (in rad)
	 * @return
	 */
	private static SimulatedLampModel getLamp(double[] params) {
		Vec2  gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		World world = new World(gravity, doSleep);
				
		SimulatedLampModel lamp = new SimulatedLampModel(world, params);
		lamp.initTest();
		return lamp;
	}
	
	public static void hillClimbingForLamp(double[] start, float stepLimit, int attempts, int hz, int seconds)
	{
		IObjectiveFunction func = new LampObjectiveFunction();
		
		int iterations = 100;
		
		double[] current = start.clone();
//		SimulatedLampModel lamp = getLamp(current);
		double currentEval = func.valueOf(current); //jumpForSeconds(lamp, hz, seconds);
		for (int i = 0; i < iterations; i++)
		{
			double[] maxNode = current;
			double maxEval = currentEval;
			for (double[] next : neighbours(current, stepLimit, attempts))
			{
//				lamp = getLamp(next);
				double nextEval = func.valueOf(next); //jumpForSeconds(lamp, hz, seconds);
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
				for (double f : maxNode)
				{
					System.out.printf("%f, ", f);
				}
				System.out.println();
				current = maxNode;
				currentEval = maxEval;
			}
		}
	}
	
	public static void hillClimbing(double[] start, double stepLimit, int attempts, int hz, int seconds)
	{
	
		int iterations = 100;
		
		double[] current = start.clone();
		SimulatedBipedWalker walker = getWalker(current);
		double currentEval = walkForSeconds(walker, hz, seconds);
		for (int i = 0; i < iterations; i++)
		{
			double[] maxNode = current;
			double maxEval = currentEval;
			for (double[] next : neighbours(current, stepLimit, attempts))
			{
				walker = getWalker(next);
				double nextEval = walkForSeconds(walker, hz, seconds);
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
				for (double f : maxNode)
				{
					System.out.printf("%f, ", f);
				}
				System.out.println();
				current = maxNode;
				currentEval = maxEval;
			}
		}
	}
	
	public static ArrayList<double[]> neighbours(double[] current, double stepLimit, int attempts)
	{
		
		
		ArrayList<double[]> results = new ArrayList<double[]>();
//		float[] steps = new float[]{-.5f, .5f};
		
		for (int i = 0; i < current.length; i++)
		{
			for (int j = 0; j < attempts; j++)
			{
				Random random = new Random();
				
				double[] next = current.clone();
				next[i] = current[i] + stepLimit * 2  * (random.nextFloat() - 0.5f); 
				results.add(next);
			}
		}
		return results;
	}
}
