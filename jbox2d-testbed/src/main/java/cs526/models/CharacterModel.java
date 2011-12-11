package cs526.models;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;
import java.util.Set;

import cs526.utilities.ControllerInfo;
import cs526.utilities.ControllersLoader;
import cs526.utilities.DesiredState;
import cs526.utilities.JointInfo;
import cs526.utilities.JointsLoader;
import cs526.utilities.LinkInfo;
import cs526.utilities.LinksLoader;
import cs526.utilities.StatesLoader;

/**
 * Represents a character model. <br/>
 * <br/>
 * It contains information such as: links, joints, controllers and control states.
 * @author shuo
 *
 */
public class CharacterModel {
	
	/**
	 * text that marks the beginning of links section in the input file
	 */
	private static final String LINKS = "links";
	
	/**
	 * text that marks the beginning of joints section in the input file
	 */
	private static final String JOINTS = "joints";
	
	private static final String CONTROLLERS = "controllers";
	
	private static final String STATES = "states";
	
	
	private HashMap<String, LinkInfo> linkInfoMap = new HashMap<String, LinkInfo>();
	private HashMap<String, JointInfo> jointInfoMap = new HashMap<String, JointInfo>();
	private HashMap<String, ControllerInfo> controllerInfoMap = new HashMap<String, ControllerInfo>();
	private ArrayList<DesiredState> states = new ArrayList<DesiredState>();
	
	/**
	 * The constructor is used to load a character model from a file
	 * @param obj a file with the same name as the name of the obj's class will be loaded.
	 */
	public CharacterModel(Object obj)
	{
		Scanner scanner = null;
		try {
			scanner = new Scanner(new File(obj.getClass().getSimpleName()));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		
		while (scanner.hasNext())
		{
			String line = scanner.nextLine();
			if (line.trim().startsWith("#")) // it's a comment
			{
				continue;
			}
			else if (LINKS.compareToIgnoreCase(line.trim())==0)
			{
				LinksLoader.loadLinks(scanner, linkInfoMap);
			}
			else if (JOINTS.compareToIgnoreCase(line.trim())==0)
			{
				JointsLoader.loadJoints(scanner, jointInfoMap);
			}
			else if (CONTROLLERS.compareToIgnoreCase(line.trim())== 0)
			{
				ControllersLoader.loadControllers(scanner, controllerInfoMap);
			}
			else if (STATES.compareToIgnoreCase(line.trim()) == 0)
			{
				StatesLoader.loadStates(scanner, states);
			}
		}
				
	}
	
	public Set<String> getLinkNames()
	{
		return linkInfoMap.keySet();
	}
	
	public LinkInfo getLinkInfoByName(String linkName)
	{
		return linkInfoMap.get(linkName);
	}
	
	public Set<String> getJointNames()
	{
		return jointInfoMap.keySet();
	}
	
	public JointInfo getJointInfoByName(String jointName)
	{
		return jointInfoMap.get(jointName);
	}

	public Set<String> getControllerNames() {
		return controllerInfoMap.keySet();
	}

	public ControllerInfo getControllerInfoByJointName(String jointName) {
		return controllerInfoMap.get(jointName);
	}

	public DesiredState getState(int stateId){
		if (stateId < 0 || states.size() == 0)
			return null;
		else
			return states.get(stateId % states.size());
	}
}
