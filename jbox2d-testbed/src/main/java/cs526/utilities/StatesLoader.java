package cs526.utilities;

import java.util.ArrayList;
import java.util.Scanner;

public class StatesLoader {

	public static void loadStates(Scanner scanner,
			ArrayList<DesiredState> states) {
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
			int nJoints = Integer.parseInt(start);
			
			DesiredState state = new DesiredState();
			while (nJoints-- > 0)
			{
				state.putDesiredAngle(scanner.next(), scanner.nextFloat());
			}
			states.add(state);
						
		}
		
	}

}