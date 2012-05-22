package cs526.simu;

import java.util.ArrayList;
import java.util.Date;
import java.util.Random;

import fr.inria.optimization.cmaes.fitness.IObjectiveFunction;

public class HillClimbingOptimizer implements Optimizer {

		static final int attempts = 30;
	static final double stepLimit = 0.1;
	
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
	
	@Override
	public double[] optimize(IObjectiveFunction func, double[] startPt, int maxIt) {
		
		System.out.println("---------------------------------------------------");
		
		System.out.printf("attempts : %d, neighbour: %f\n", attempts, stepLimit);
		Date date = new Date();
		int count = 0;	
		double[] current = startPt.clone();
//		SimulatedLampModel lamp = getLamp(current);
		double currentEval = func.valueOf(current); //jumpForSeconds(lamp, hz, seconds);
		
		System.out.printf("first eval: %f\n", currentEval);
		double[] minNode = current;
		for (int i = 0; i < maxIt; i++)
		{
		
			double minEval = currentEval;
			for (double[] next : neighbours(current, stepLimit, attempts))
			{
//				lamp = getLamp(next);
				double nextEval = func.valueOf(next); //jumpForSeconds(lamp, hz, seconds);
				count ++;
				if (nextEval < minEval)
				{
					minNode = next;
					minEval = nextEval;
				}
			}
			if (minNode == current)
			{
				break;
			}
			else
			{
				System.out.printf("iteration %d, maxEval %.2f ", i, minEval);
				for (double f : minNode)
				{
					System.out.printf("%f, ", f);
				}
				System.out.println();
				current = minNode;
				currentEval = minEval;
			}
		}
		
		System.out.printf("%d evaluations in %d milsec", count, new Date().getTime() - date.getTime());
		// TODO Auto-generated method stub
		return minNode;
	}
	

}
