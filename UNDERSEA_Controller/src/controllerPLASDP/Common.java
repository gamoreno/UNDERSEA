package controllerPLASDP;

import auxiliary.Utility;

public class Common {
	
	static protected Common instance = new Common();
	
	public String[] sensors;
	public double[] speeds;
	public double TIME_WINDOW;
	

	
	protected Common() {
		TIME_WINDOW = Double.parseDouble(Utility.getProperty("TIME_WINDOW"));
		
		sensors = Utility.getProperty("SENSORS").split(",");
		
		String[] speedRange = Utility.getProperty("SPEED").split(",");
		double min = Double.parseDouble(speedRange[0]);
		double max = Double.parseDouble(speedRange[1]);
		int numOfSpeeds = Integer.parseInt(speedRange[2]);
		speeds = new double[numOfSpeeds];
		double delta = (max - min) / (numOfSpeeds - 1);
		for (int i = 0; i < numOfSpeeds - 1; i++) {
			speeds[i] = min + delta * i;
		}
		speeds[numOfSpeeds - 1] = max;
		
		// test
		for (int i = 0; i < numOfSpeeds; i++) {
			assert(getSpeedIndex(speeds[i]) == i);
		}
	}
	
	public static Common getInstance() {
		return instance;
	}

	/**
	/* Find the closest discretized speed
	 * 
	 * @param speed
	 * @return index of discrete speed closest to speed
	 */
	public int getSpeedIndex(double speed) {
		double minError = Double.MAX_VALUE;
		int i = 0;
		while (i < speeds.length) {
			double error = Math.abs(speed - speeds[i]);
			if (error > minError) {
				break;
			}
			minError = error;
			i++;
		}
		return i - 1;
	}	
	
}
