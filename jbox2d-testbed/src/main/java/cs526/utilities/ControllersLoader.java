package cs526.utilities;

import java.util.HashMap;
import java.util.Scanner;

public class ControllersLoader {

	public static void loadControllers(Scanner scanner,
			HashMap<String, ControllerInfo> controllerInfoMap) {
		
//		StickTest st = new StickTest();
//		controller1 = new PdController(joint1, 80f, 2f);
//		controller2 = new PdController(joint2, 40f, 1f);

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
			
			ControllerInfo controllerInfo = new ControllerInfo();
			
			String jointName = start;
			controllerInfo.ks = scanner.nextFloat();
			controllerInfo.kd = scanner.nextFloat();
			
			controllerInfoMap.put(jointName, controllerInfo);
		}
		
		
	}

}
