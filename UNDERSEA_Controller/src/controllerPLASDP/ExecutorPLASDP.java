package controllerPLASDP;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import controller.Executor;
import controller.Knowledge;
import pladapt.StringVector;

public class ExecutorPLASDP extends Executor {
	
	public ExecutorPLASDP() {
	}

	@Override
	public void run () {
		
		/*
		 * For the CSV output to work correctly the speed and the state
		 * of the sensors has to be set in the Knowledge even if they don't
		 * change
		 */
		
		Knowledge knowledge = Knowledge.getInstance();

		// get the state before executing the tactics
		double speed = knowledge.getUUVspeed();
		int speedIndex = Common.getInstance().getSpeedIndex(speed);

		Map<String, Double> sensorState = new HashMap<>();
		for (String sensor : knowledge.sensorsMap.keySet()) {
			sensorState.put(sensor, knowledge.getSensorState(sensor));
		}
		
		
		StringVector tactics = knowledge.tacticsToExecute;
        if (tactics != null && !tactics.isEmpty()) {
	        for (int t = 0; t < tactics.size(); t++) {
	        	String tactic = tactics.get(t);
	        	System.out.println(tactic);
	        	if (tactic.endsWith("On")) {
	        		String sensor = tactic.substring(0, tactic.length() - 2);
	        		sensorState.put(sensor, 1.0);
	        	} else if (tactic.endsWith("Off")) {
	        		String sensor = tactic.substring(0, tactic.length() - 3);
	        		sensorState.put(sensor, 0.0);
	        	} else if (tactic.endsWith("Speed")) {
	        		if (tactic.startsWith("Inc")) {
	        			speedIndex++;
	        		} else if (tactic.startsWith("Dec")) {
	        			speedIndex--;
	        		}
	        	}
	        }
        }
        
        knowledge.tacticsToExecute.clear();

        // update knowledge while we generate execution command
        
		command = "SPEED="   + (knowledge.getUUVspeed()) +",";
		knowledge.setUUVspeed(Common.getInstance().speeds[speedIndex]);

		Iterator<String> it = knowledge.sensorsMap.keySet().iterator();
		while (it.hasNext()){
			String sensorName = it.next();
			double state = sensorState.get(sensorName);
			command += sensorName +"="+ (state);
			
			if (it.hasNext())
				command += ",";
			
			knowledge.setSensorState(sensorName, (int) state);			
		}
	}	
}
