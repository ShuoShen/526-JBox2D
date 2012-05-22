package cs526.simu;

import fr.inria.optimization.cmaes.fitness.IObjectiveFunction;

public interface CountedObjectiveFunction extends IObjectiveFunction {
	int evalCount();
}
