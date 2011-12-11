package cs526.utilities;

import java.util.HashMap;
import java.util.Set;

public class DesiredState
{
	// TODO: change the desired angle back to degree
	private HashMap<String, Float> desiredAngles = new HashMap<String, Float>();
	
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
}