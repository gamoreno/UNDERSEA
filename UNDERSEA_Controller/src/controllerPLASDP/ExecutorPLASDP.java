package controllerPLASDP;

import java.util.Iterator;

import controller.Executor;
import controller.Knowledge;
import pladapt.StringVector;

public class ExecutorPLASDP extends Executor {
	
	public ExecutorPLASDP() {
	}

	@Override
	public void run () {
		Knowledge knowledge = Knowledge.getInstance();
        StringVector tactics = knowledge.tacticsToExecute;
        if (tactics == null || tactics.isEmpty()) {
        	return;
        }
        
        for (int t = 0; t < tactics.size(); t++) {
        	String tactic = tactics.get(t);
        	System.out.println(tactic);
        	if (tactic.endsWith("On")) {
        		String sensor = tactic.substring(0, tactic.length() - 2);
        		knowledge.setSensorState(sensor, 1);
        	} else if (tactic.endsWith("Off")) {
        		String sensor = tactic.substring(0, tactic.length() - 3);
        		knowledge.setSensorState(sensor, 0);
        	} else if (tactic.endsWith("Speed")) {
        		double speed = knowledge.getUUVspeed();
        		int speedIndex = Common.getInstance().getSpeedIndex(speed);
        		if (tactic.startsWith("Inc")) {
        			speedIndex++;
        		} else if (tactic.startsWith("Dec")) {
        			speedIndex--;
        		}
        		knowledge.setUUVspeed(Common.getInstance().speeds[speedIndex]);
        	}
        }
        
        knowledge.tacticsToExecute.clear();
        
		command = "SPEED="   + (knowledge.getUUVspeed()) +",";
		
		Iterator<String> it = knowledge.sensorsMap.keySet().iterator();
		while (it.hasNext()){
			String sensorName = it.next();
			command += sensorName +"="+ (knowledge.getSensorState(sensorName));
			
			if (it.hasNext())
				command += ",";
		}
	}	
}
