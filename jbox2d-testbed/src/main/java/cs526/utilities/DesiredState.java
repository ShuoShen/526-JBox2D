package cs526.utilities;

import java.util.HashMap;
import java.util.Set;

public class DesiredState
{
	private float stepTime;
	
	// TODO: change the desired angle back to degree
	private HashMap<String, Float> desiredAngles = new HashMap<String, Float>();
		
	public DesiredState(float stepTime)
	{
		this.stepTime = stepTime;
	}
	
	public void putDesiredAngle(String jointName, Float angleInDegree)
	{
		desiredAngles.put(jointName, (float)Math.toRadians(angleInDegree));
	}
	
	public Set<String> getJointNames()
	{
		return desiredAngles.keySet();
	}
	
	public float getAngleByJointName(String jointName)
	{
		return desiredAngles.get(jointName);
		
	}
	public float getStepTime()
	{
		return stepTime;
				
	}
	
}