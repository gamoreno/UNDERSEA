package controllerPLASDP;

import java.util.Map.Entry;

import controller.Knowledge;
import controller.Monitor;
import controller.uuv.UUVSensor;
import pladapt.TimeSeriesPredictor;

public class MonitorPLASDP extends Monitor {
	
	public static final String TIMESERIES_PREDICTOR_MODEL_ARGS = "LES 0.8 0.15";
	public static final int TIMESERIES_PREDICTOR_TRAINING_LENGTH = 15;
	public static final int TIMESERIES_PREDICTOR_HORIZON = 3;


	public MonitorPLASDP() {
	}

	@Override
	public void run() {
		System.out.println("MonitorPLASDP.run()");
		Knowledge.getInstance().analysisRequired  = true; // this is not used, though
		
		
		for (Entry<String, UUVSensor> entry : Knowledge.getInstance().sensorsMap.entrySet()) {
			UUVSensor sensor = entry.getValue();
			TimeSeriesPredictor predictor = Knowledge.getInstance().predictors.get(entry.getKey());
			if (predictor == null) {
				predictor = TimeSeriesPredictor.getInstance(TIMESERIES_PREDICTOR_MODEL_ARGS,
						TIMESERIES_PREDICTOR_TRAINING_LENGTH, TIMESERIES_PREDICTOR_HORIZON);
				Knowledge.getInstance().predictors.put(entry.getKey(), predictor);
			}
		
			double rate = sensor.getCurrentRate();
			System.out.println("Monitor: " + entry.getKey() + " rate = " + rate);
			predictor.observe(rate);
		}
	}	

}
