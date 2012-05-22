package cs526.utilities;

import java.util.ArrayList;
import java.util.Scanner;

public class LampStatesConverter implements ParamToStateConverter {

	
	/* (non-Javadoc)
	 *  * s1, s2, s3 (seconds)
	 * j11, j12, j21, j22, j31, j32 (in rad)
	 */
	@Override
	public Scanner convert(double[] stateParams) {
		if (stateParams == null)
			return null;
		StringBuilder sb = new StringBuilder();
		
		int n = 9;
		assert(stateParams.length == n);
		
		for (int i = 0; i < n; i++ )
		{
			if (i % 3 == 0)
			{
				sb.append(stateParams[i]);
				sb.append(" 2 ");
			}
			else if (i % 3 == 1)
			{
				sb.append(" a1 ");
				sb.append(Math.toDegrees(stateParams[i]));
			}
			else
			{
				sb.append(" a2 ");
				sb.append(Math.toDegrees(stateParams[i]));
				sb.append("\n");
			}
		}
		sb.append("00\n");
		
		return new Scanner(sb.toString());

	}

	

}
