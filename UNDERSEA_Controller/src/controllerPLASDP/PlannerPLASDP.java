package controllerPLASDP;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

import org.apache.velocity.VelocityContext;
import org.apache.velocity.app.Velocity;
import org.ho.yaml.Yaml;

import auxiliary.Utility;
import controller.Knowledge;
import controller.Planner;
import pladapt.EnvironmentDTMCPartitioned;
import pladapt.GenericConfiguration;
import pladapt.GenericConfigurationManager;
import pladapt.JavaSDPAdaptationManager;
import pladapt.SDPAdaptationManager;
import pladapt.StringVector;
import pladapt.TimeSeriesPredictor;

public class PlannerPLASDP extends Planner {
	
	protected static final String PLASDP_REACH_PATH = "/home/gmoreno/research/code/adaptmgr/reach/reach.sh";
	protected static final String PLASDP_REACH_MODEL = "uuv-plasdp";
	protected static final int PLASDP_ENV_DTMC_BRANCHING_DEPTH = 2;
	protected static final int PLASDP_ENV_DTMC_DEPTH = 2;
	protected JavaSDPAdaptationManager m_adaptMgr;

	private long HORIZON;
	private double MIN_SPEED;
	
    static {
        System.loadLibrary("pladapt_wrap");
      }
	
	public PlannerPLASDP() {
		MIN_SPEED = Common.getInstance().speeds[0];
		HORIZON = Math.max(5, Math.round(10 / (MIN_SPEED * Common.getInstance().TIME_WINDOW)));
	}

	protected void generateAlloyModel() throws IOException {
        Velocity.init();
        VelocityContext context = new VelocityContext();
        context.put("Common.getInstance().sensors", Common.getInstance().sensors);

        //StringWriter w = new StringWriter();
        File alloyModel = new File(PLASDP_REACH_MODEL + "-nl.als");
        System.out.println("Generating Alloy model " + alloyModel.getAbsolutePath());
        BufferedWriter writer = new BufferedWriter(new FileWriter(alloyModel));

        Velocity.mergeTemplate("models/plasdp/" + PLASDP_REACH_MODEL + "-nl.vm", "UTF-8", context, writer );
        writer.close();
	}
	
	public void initialize() throws IOException {
		System.out.println("Starting PLA-SDP Adaptation Manager initialization");
        m_adaptMgr = new JavaSDPAdaptationManager();
        
        generateAlloyModel();

        // define configuration space
        GenericConfigurationManager configMgr = new GenericConfigurationManager();
        GenericConfiguration configTemplate = configMgr.getConfigurationTemplate();
        
        configTemplate.setInt("speed", 0);
        // create one config 0/1 variable for each sensor 
        for (String sensor : Common.getInstance().sensors) {
        	configTemplate.setBool(sensor, false);
        }

        // generate all the configurations
        for (int speed = 0; speed < Common.getInstance().speeds.length; speed++) {
        
	        // iterate over a binary encoding of the sensor configurations
	        int numberOfSensorConfigs = (int) Math.pow(2, Common.getInstance().sensors.length);
	        for (int sensorConfig = 0; sensorConfig < numberOfSensorConfigs; sensorConfig++) {
	        	GenericConfiguration config = configMgr.addNewConfiguration();
	        	config.setInt("speed", speed);
	        	int mask = 1;
	            for (String sensor : Common.getInstance().sensors) {
	            	config.setBool(sensor, (sensorConfig & mask) > 0);
	            	mask = mask << 1;
	            }
	        }
        }
        
    	// create configuration parameters.
    	HashMap<String, Object> params = new HashMap<>();
        params.put(SDPAdaptationManager.getNO_LATENCY(), Boolean.TRUE);
        params.put(SDPAdaptationManager.getREACH_PATH(), PLASDP_REACH_PATH);
        params.put(SDPAdaptationManager.getREACH_MODEL(), PLASDP_REACH_MODEL);
        params.put(SDPAdaptationManager.getREACH_OPTIONS(), "-c ConfigUuv");
		params.put(SDPAdaptationManager.getREACH_SCOPE(), "S=" + Common.getInstance().speeds.length);
//				"S=" + maxServers + " TAP#=" + addServerLatencyPeriods + " D=" + rubisModel.getDimmerLevels());
        String yamlParams = Yaml.dump(params);
        
        m_adaptMgr.initialize(configMgr, yamlParams);
    	System.out.println("PLA-SDP Adaptation Manager Initialized");
	}

	
	protected EnvironmentDTMCPartitioned generateEnvironment() {
		
		// this generates a DTMC for a single prediction
		// the GenericEnvironment state in the DTMC has a property "m" the default, with the value
		// of the state (i.e., what was being fed to the predictor)
		// when using multiple predictors, all the DTMCs will have to be joined
		Iterator<Entry<String, TimeSeriesPredictor> > it = Knowledge.getInstance().predictors.entrySet().iterator();
		EnvironmentDTMCPartitioned envDTMC = it.next().getValue().generateEnvironmentDTMC(PLASDP_ENV_DTMC_BRANCHING_DEPTH,
				PLASDP_ENV_DTMC_DEPTH);
		EnvironmentDTMCPartitioned env2 = null;
		if (it.hasNext()) {
			env2 = EnvironmentDTMCPartitioned.createJointDTMC(envDTMC, it.next().getValue().generateEnvironmentDTMC(PLASDP_ENV_DTMC_BRANCHING_DEPTH,
				PLASDP_ENV_DTMC_DEPTH));
		}
		
		return env2;
	}


	@Override
	public void run() {

        GenericConfiguration currentConfig = new GenericConfiguration();
        currentConfig.setInt("speed", Common.getInstance().getSpeedIndex(Knowledge.getInstance().getUUVspeed()));
        for (String sensor : Common.getInstance().sensors) {
        	int sensorState = (int) Knowledge.getInstance().getSensorState(sensor);
        	if (sensorState == -1) {
        		sensorState = 0; // TODO for now assume that -1 == 0, but is it the same?
        	}
        	currentConfig.setBool(sensor, sensorState != 0);
        }
        System.out.println("current configuration is " + currentConfig);
        
        UtilityFunction utilityFunction = new UtilityFunction();
        EnvironmentDTMCPartitioned env = generateEnvironment();
		m_adaptMgr.setDebug(true);
        StringVector tactics = m_adaptMgr.evaluate(currentConfig, env, utilityFunction, HORIZON );

        if (tactics.isEmpty()) {
        	System.out.println("no adaptation required");
        }
        
        Knowledge.getInstance().tacticsToExecute = tactics; // for the Executor to use
	}	

}
