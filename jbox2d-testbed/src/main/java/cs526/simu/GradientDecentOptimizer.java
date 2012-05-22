package cs526.simu;

import fr.inria.optimization.cmaes.fitness.IObjectiveFunction;

public class GradientDecentOptimizer implements Optimizer {
	final double tol = 1e-4; 
	final double mu = 0.001;
	final double s = 0.5;
	final double delta = 1e-5;
	final double startStep = 0.8;
	
	@Override
	public double[] optimize(IObjectiveFunction fun, double[] startPt, int maxIt) {
		
		
		double[] pt = startPt;
		System.out.printf("init eval: %f\n\n\n", fun.valueOf(pt));
		
		int i = 0; 
		while (true)
		{
			i++;
			double[] grad = gradient(fun, pt, delta);
			
			if (norm(grad) < tol || i >maxIt)
				break;
			double[] d = grad.clone();
			
			for (int j = 0; j < d.length; j++)
			{
				d[j] *= -1;
			}
			

			System.out.printf("iteration %d\n", i);
			System.out.print("gradient: ");
			for (double g : grad)
			{
				System.out.printf("%f, ", g);
			}
			System.out.println();
			
			double alpha = stepSize(fun, pt, d);
			
			System.out.printf("alpha: %f\n", alpha);
			
			pt = step(pt, d, alpha);
			
			System.out.print("point: ");
			for (double g : pt)
			{
				System.out.printf("%f, ", g);
			}
			
			
			System.out.println();
			
			System.out.printf("eval: %f\n\n\n", fun.valueOf(pt));
		}
		
		System.out.printf("final eval = %f after %d iterations.\n", fun.valueOf(pt), i);
		return pt;
	}
	
	private double[] step(double[] pt, double[] dir, double stepSize)
	{
		double[] ret = pt.clone();
		
		for (int i = 0; i < ret.length; i++)
		{
			ret[i] += dir[i] * stepSize; 
		}
		
		return ret;
	}
	
	private double stepSize(IObjectiveFunction fun,  double[] x, double[] dir)
	{
		double fx = fun.valueOf(x);
	
		double alpha = startStep;
		double gtdn = -Math.pow(norm(dir), 2); 
		
		while (true)
		{
			double[] xnn = x.clone();
			for (int i = 0; i < xnn.length; i++)
			{
				xnn[i] = x[i] + alpha * dir[i]; 
			}
			
			if (fun.valueOf(xnn) - fx <= mu * alpha * gtdn)
			{
				break;
			}
			
			alpha *= s;
		}
		return alpha;
	}
	
	private double norm(double[] vec)
	{
		double val = 0;
		for (double v : vec)
		{
			val += v*v;
		}
		
		return Math.sqrt(val);
	}
	private double[] gradient(IObjectiveFunction fun, double[] pt, double delta)
	{
		double[] grad = new double[pt.length];
		for (int i = 0; i < pt.length; i++)
		{
			double d = delta * Math.abs(pt[i]);
			double[] fw = pt.clone();
			fw[i] += d;
			double[] bw = pt.clone();
			bw[i] -= d;
			
			grad[i] = (fun.valueOf(fw) - fun.valueOf(bw)) / (2* d);
		}
		
		return grad;
	}

}
