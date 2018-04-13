package controllerPLASDP;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import controller.Knowledge;
import controller.uuv.UUVSensor;
import pladapt.Configuration;
import pladapt.Environment;
import pladapt.GenericConfiguration;
import pladapt.GenericEnvironment;
import pladapt.JointEnvironment;
import pladapt.UtilityFunction;


public class UuvUtilityFunction extends UtilityFunction {

	private static final Map<String, Double> switchOnCost;
	private static final Map<String, Double> switchOffCost;
	private static final Map<String, Double> readingEnergy;
	
	public static final double w1 = 1;
	public static final double w2 = 20;
	public static final double REWARD_SHIFT = 10000000;
	
	private double evalPeriod;
	private double[] speeds;
	private GenericConfiguration currentConfig;
	private Map<String, Integer> componentMap;
	
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
    
    public UuvUtilityFunction(GenericConfiguration currentConfig, Map<String, Integer> componentMap) {
    	evalPeriod = Common.getInstance().TIME_WINDOW;
    	this.speeds = Common.getInstance().speeds;
    	this.currentConfig = currentConfig;
    	this.componentMap = componentMap;
    }
    
    private double getSensorProperty(String sensor, Map<String, Double> propertyMap) {
    	Double value = propertyMap.get(sensor);
    	return (value == null) ? 0.0 : value.doubleValue();
    }

    /**
     * Compute the energy used for sensing per second
     * 
     * Since this is for a single configuration, no switching cost is considered
     * 
     * @param config
     * @param jointEnv
     * @return Joules per second
     */
	private double getSensingJoulesPerSec(GenericConfiguration config, JointEnvironment jointEnv) {
		double joulesPerSec = 0.0;
		for (Entry<String, UUVSensor> entry : Knowledge.getInstance().sensorsMap.entrySet()) {
			if (config.getBool(entry.getKey())) {
				double sensorRate = jointEnv.getComponent(componentMap.get(entry.getKey())).asDouble();
				UUVSensor sensor = entry.getValue();
				joulesPerSec += sensorRate * getSensorProperty(sensor.getName(), readingEnergy);
			}
		}
		return joulesPerSec;
	}
	
    @Override
	public double getAdditiveUtility(Configuration config, Environment environment, int time) {
		GenericConfiguration _config = (GenericConfiguration) config;
		JointEnvironment jointEnv = (JointEnvironment) environment;

		double speed = speeds[_config.getInt("speed")];

		double joulesPerSec = getSensingJoulesPerSec(_config, jointEnv);
		
		double joulesPerMeter = joulesPerSec / speed;
		
		double energy = 10 * joulesPerMeter;
		
		double costPer10m = w1 * energy + w2 / speed;
		
		// consider only the distance traveled in the evaluation period
		return REWARD_SHIFT - (costPer10m / 10) * speed * evalPeriod;
	}

    @Override
	public double getAdaptationReward(Configuration from, Configuration to, int time) {
    	GenericConfiguration _from = (GenericConfiguration) from;
    	GenericConfiguration _to = (GenericConfiguration) to;
    	
    	double switchingCost = getSwitchingEnergy(_from, _to);
		
		// prorate the switching cost to the distance traveled in the evaluation period
		// reward is negative cost, we shift it so that * 0 for unsat requirement is the minimum
		double speed = speeds[_to.getInt("speed")];
		return REWARD_SHIFT - (w1 * switchingCost / 10) * speed * evalPeriod; 
	}

	private double getSwitchingEnergy(GenericConfiguration from, GenericConfiguration to) {
		double switchingCost = 0.0;
		for (String sensor : Knowledge.getInstance().sensorsMap.keySet()) {
			if (from.getBool(sensor) && !to.getBool(sensor)) {
				switchingCost += getSensorProperty(sensor, switchOffCost);
			} else if (!from.getBool(sensor) && to.getBool(sensor)) {
				switchingCost += getSensorProperty(sensor, switchOnCost);
			} 
		}
		return switchingCost;
	}
	
	private int requirement1(GenericConfiguration config, JointEnvironment env) {
		double imgsPerSec = 0.0;

		double speed = speeds[config.getInt("speed")];

		for (Entry<String, UUVSensor> entry : Knowledge.getInstance().sensorsMap.entrySet()) {
			if (config.getBool(entry.getKey())) {
//				UUVSensor sensor = entry.getValue();
//				double alpha = 0.15; // TODO get this right
//				double probOfAccurateReading = 1 - alpha * speed;
				double probOfAccurateReading = Common.getInstance().sensorReliability.get(entry.getKey());
				double sensorRate = env.getComponent(componentMap.get(entry.getKey())).asDouble();
				imgsPerSec +=  probOfAccurateReading * sensorRate;
				//System.out.println("speed=" + speed + " probOfAccRead=" + probOfAccurateReading + " sensorRate=" + sensorRate + " imgPerMeter=" + imgsPerMeter);
			}
		}
		
		double imgsPerMeter = imgsPerSec / speed;
		
		//System.out.println("ImgPerMeter = " + imgsPerMeter);
		
		return (imgsPerMeter < 2.0) ? 0 : 1; // at least 20 images per 10m
	}

	/**
	 * Checks satisfaction of requirement 2
	 * 
	 * The switching cost is only considered for the first adaptation
	 * 
	 * @param config
	 * @param env
	 * @return 1 if satisfied, 0 otherwise
	 */
	private int requirement2(GenericConfiguration config, JointEnvironment env, int time) {
		double switchingEnergy = 0;
		if (time == 1) {
			
			// this is the first adaptation from the current configuration
			switchingEnergy = getSwitchingEnergy(currentConfig, config); 
		}
		
		double speed = speeds[config.getInt("speed")];
		double joulesPerSec = getSensingJoulesPerSec(config, env);
		double joulesPerMeter = joulesPerSec / speed;

		return (switchingEnergy + joulesPerMeter * 10 > 120.0) ? 0 : 1; // no more than 120 Joules per 10m
	}
	
	@Override
	public double getMultiplicativeUtility(Configuration config, Environment environment, int time) {
		GenericConfiguration _config = (GenericConfiguration) config;
		JointEnvironment _environment = (JointEnvironment) environment;
		return requirement1(_config, _environment) * requirement2(_config, _environment, time);
	}
	
}
