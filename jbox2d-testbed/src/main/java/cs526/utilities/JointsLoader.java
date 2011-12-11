package cs526.utilities;

import java.util.HashMap;
import java.util.Scanner;

public class JointsLoader {

	public static void loadJoints(Scanner scanner,
			HashMap<String, JointInfo> jointInfoMap) {
		while (scanner.hasNext()) {
			String start = scanner.next();
			if (start.trim().startsWith("#")) // don't read comment lines.
			{
				scanner.nextLine();
				continue;
			}
			else if (start.trim().equals("00")) // quit reading links when new line
			{
				break;
			}
			JointInfo jointInfo = new JointInfo();
			String jointName = start;
			jointInfo.linkA = scanner.next();
			
			jointInfo.linkAPosition = LinkPosition.valueOf(scanner.next());
			jointInfo.linkB = scanner.next();
			jointInfo.linkBPosition = LinkPosition.valueOf(scanner.next());
			
			jointInfo.upperAngleDegree = scanner.nextFloat();
			jointInfo.lowerAngleDegree = scanner.nextFloat();
			
			jointInfoMap.put(jointName, jointInfo);
			
		}
		
	}

}
