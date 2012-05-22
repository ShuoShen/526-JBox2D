package cs526.simu;

import fr.inria.optimization.cmaes.fitness.IObjectiveFunction;

public interface Optimizer {
	double[] optimize(IObjectiveFunction fun,  double[] startPt, int maxIt);
	
}
