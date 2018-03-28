package controllerPLASDP;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import controller.Knowledge;
import controller.uuv.UUVSensor;
import pladapt.GenericConfiguration;
import pladapt.GenericEnvironment;
import pladapt.GenericUtilityFunction;


public class UtilityFunction extends GenericUtilityFunction {

	private static final Map<String, Double> switchOnCost;
	private static final Map<String, Double> switchOffCost;
	private static final Map<String, Double> readingEnergy;
	
	public static final double w1 = 1;
	public static final double w2 = 20;
	
	private double evalPeriod;
	private double[] speeds;
	
    static {
        Map<String, Double> on = new HashMap<>();
        on.put("SENSOR1", 10.0);
        on.put("SENSOR2", 8.0);
        on.put("SENSOR3", 5.0);
        switchOnCost = Collections.unmodifiableMap(on);

        Map<String, Double> off = new HashMap<>();
        off.put("SENSOR1", 2.0);
        off.put("SENSOR2", 1.5);
        off.put("SENSOR3", 1.0);
        switchOffCost = Collections.unmodifiableMap(off);

        Map<String, Double> energy = new HashMap<>();
        energy.put("SENSOR1", 3.0);
        energy.put("SENSOR2", 2.4);
        energy.put("SENSOR3", 2.1);
        readingEnergy = Collections.unmodifiableMap(energy);
    }
    
    public UtilityFunction() {
    	evalPeriod = Common.getInstance().TIME_WINDOW;
    	this.speeds = Common.getInstance().speeds;
    }
    
    private double getSensorProperty(String sensor, Map<String, Double> propertyMap) {
    	Double value = propertyMap.get(sensor);
    	return (value == null) ? 0.0 : value.doubleValue();
    }

	public double getGenAdditiveUtility(GenericConfiguration config, GenericEnvironment env, int time) {
//		if (config.getBool("SENSOR1") && config.getBool("SENSOR2")) {
//			return 10.0;
//		}
//		return 1.0;

		double speed = speeds[config.getInt("speed")];

		double joulesPerMeter = 0.0;
		for (Entry<String, UUVSensor> entry : Knowledge.getInstance().sensorsMap.entrySet()) {
			if (config.getBool(entry.getKey())) {
				UUVSensor sensor = entry.getValue();
				double sensorRate = 2; // TODO get this from env
				joulesPerMeter += sensorRate * getSensorProperty(sensor.getName(), readingEnergy);
			}
		}
		joulesPerMeter /= speed;
		
		double energy = 10 * joulesPerMeter;
		
		double costPer10m = w1 * energy + w2 / speed;
		
		// consider only the distance traveled in the evaluation period
		return -(costPer10m / 10) * speed * evalPeriod;
	}
	
	public double getGenAdaptationReward(GenericConfiguration from, GenericConfiguration to, int time) {
		double switchingCost = 0.0;
		
		double speed = speeds[to.getInt("speed")];

		for (String sensor : Knowledge.getInstance().sensorsMap.keySet()) {
			if (from.getBool(sensor) && !to.getBool(sensor)) {
				switchingCost += getSensorProperty(sensor, switchOffCost);
			} else if (!from.getBool(sensor) && to.getBool(sensor)) {
				switchingCost += getSensorProperty(sensor, switchOnCost);
			} 
		}
		
		// prorate the switching cost to the distance traveled in the evaluation period
		return -(w1 * switchingCost / 10) * speed * evalPeriod; // reward is negative cost
	}
	
	private int requirement1(GenericConfiguration config, GenericEnvironment env) {
		double imgsPerMeter = 0.0;

		double speed = speeds[config.getInt("speed")];

		for (Entry<String, UUVSensor> entry : Knowledge.getInstance().sensorsMap.entrySet()) {
			if (config.getBool(entry.getKey())) {
				UUVSensor sensor = entry.getValue();
				double alpha = 0.5; // TODO get this right
				double probOfAccurateReading = 1 - alpha * speed;
				double sensorRate = 2; // TODO get this from env
				imgsPerMeter +=  probOfAccurateReading * sensorRate;
			}
		}
		
		imgsPerMeter /= speed;
		
		return (imgsPerMeter < 2.0) ? 0 : 1; // at least 20 images per 10m
	}

	@Override
	public double getGenMultiplicativeUtility(GenericConfiguration config, GenericEnvironment env, int time) {
		return requirement1(config, env);
	}
	
}
