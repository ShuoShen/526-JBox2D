package cs526.utilities;

import java.util.HashMap;
import java.util.Scanner;

/**
 * a utility class that load body links into a hashmap.
 * 
 * @author shuo
 * 
 */
public class LinksLoader {
	public static void loadLinks(Scanner scanner,
			HashMap<String, LinkInfo> linkInfoMap) {
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
			LinkInfo linkInfo = new LinkInfo();
			String linkName = start;
			linkInfo.width = scanner.nextFloat();
			linkInfo.height = scanner.nextFloat();
			linkInfo.mass = scanner.nextFloat();

			linkInfoMap.put(linkName, linkInfo);
		}
	}
}
