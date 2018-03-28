package main;

import controller.Analyser;
import controller.Controller;
import controller.Executor;
import controller.Monitor;
import controllerDefault.AnalyserDefault;
import controllerPLASDP.ExecutorPLASDP;
import controllerPLASDP.MonitorPLASDP;
import controllerPLASDP.PlannerPLASDP;

public class MainController {

	public static String configFile = "resources/config.properties";	

	public static void main(String[] args) {
		//Default controller: does nothing
//	    Monitor monitor  	= new MonitorDefault();
//	    Analyser analyser	= new AnalyserDefault();
//	    Planner planner		= new PlannerDefault();
//	    Executor executor	= new ExecutorDefault();

	    //Random controller
//	    Monitor monitor  	= new MonitorRandom();
//	    Analyser analyser	= new AnalyserRandom();
//	    Planner planner		= new PlannerRandom();
//	    Executor executor	= new ExecutorRandom();


		//PMC-based controller
//	    Monitor monitor  	= new MonitorPMC();
//	    Analyser analyser	= new AnalyserPMC();
//	    Planner planner		= new PlannerPMC();
//	    Executor executor	= new ExecutorPMC();
		
		//CT-based controller
//	    Monitor monitor  	= new MonitorCT();
//	    Analyser analyser	= new AnalyserCT();
//	    Planner planner		= new PlannerCT();
//	    Executor executor	= new ExecutorCT();
		
		//PLA-SDP controller
	    Monitor monitor  	= new MonitorPLASDP();
	    Analyser analyser	= new AnalyserDefault();
	    PlannerPLASDP planner		= new PlannerPLASDP();
	    try {
	    	planner.initialize();
	    }
	    catch (Exception e) {
	    	System.out.println("Failed to initialize planner:");
	    	e.printStackTrace();
	    	return;
	    }
	    Executor executor	= new ExecutorPLASDP();

		//create new controller
		Controller controller = new Controller(monitor, analyser, planner, executor);
					
		//start engine
		ControllerEngine.start(controller);
	}

}
