/************************************************************/
/*    NAME: Simos Gerasimou                                              */
/*    ORGN: MIT                                             */
/*    FILE: UUV.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "UUV.h"

using namespace std;

//---------------------------------------------------------
// Constructor
//---------------------------------------------------------
UUV::UUV()
{
	m_iterations = 0;
	m_timewarp   = 1;
}


//---------------------------------------------------------
// Destructor
//---------------------------------------------------------
UUV::~UUV()
{
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open
//---------------------------------------------------------
bool UUV::OnStartUp()
{
	AppCastingMOOSApp::OnStartUp();               // Add this line

	list<string> sParams;
	m_MissionReader.EnableVerbatimQuoting(false);
	if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
		list<string>::iterator p;
		for(p=sParams.begin(); p!=sParams.end(); p++) {
		  string original_line = *p;
		  string param = stripBlankEnds(toupper(biteString(*p, '=')));
		  string value = stripBlankEnds(*p);

		  if(param == "NAME") { // get uuv name
			m_uuv_name = value;
		  }
		  else if (param == "SENSORS"){
			  bool handled = handleSensorsNames(value);
		  }
		  else //throw a configuration warning
			  reportUnhandledConfigWarning(original_line);
		}
	}
	else{
		reportConfigWarning("No configuration block found for " + GetAppName());
	}

	m_timewarp = GetMOOSTimeWarp();

	RegisterVariables();
	return(true);
}


//---------------------------------------------------------
// Procedure: OnConnectToServer
//---------------------------------------------------------
bool UUV::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);

   RegisterVariables();
   return(true);
}


//---------------------------------------------------------
// Procedure: RegisterVariables
//---------------------------------------------------------
void UUV::RegisterVariables()
{
	AppCastingMOOSApp::RegisterVariables();      // Add this line

	for (vector<string>::iterator it = m_uuv_sensors.begin();  it != m_uuv_sensors.end(); it++){
		 Register(*it, 0);
	}
}


//---------------------------------------------------------
// Procedure: OnNewMail
//---------------------------------------------------------
bool UUV::OnNewMail(MOOSMSG_LIST &NewMail)
{
	AppCastingMOOSApp::OnNewMail(NewMail);        // Add this line

	MOOSMSG_LIST::iterator p;

	for(p=NewMail.begin(); p!=NewMail.end(); p++) {
		CMOOSMsg &msg = *p;

		#if 0 // Keep these around just for template
			string key   = msg.GetKey();
			string comm  = msg.GetCommunity();
			double dval  = msg.GetDouble();
			string sval  = msg.GetString();
			string msrc  = msg.GetSource();
			double mtime = msg.GetTime();
			bool   mdbl  = msg.IsDouble();
			bool   mstr  = msg.IsString();
		#endif

		string key   = msg.GetKey();
//		if (find(m_uuv_sensors.begin(), m_uuv_sensors.end(), key) != m_uuv_sensors.end()){
//
//		}

	}

	return(true);
}


//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second
//---------------------------------------------------------
bool UUV::Iterate()
{
	AppCastingMOOSApp::Iterate();                  // Add this line

	//do app stuff here
	m_iterations++;

	AppCastingMOOSApp::PostReport();               // Add this line
	return(true);
}




//---------------------------------------------------------
// Procedure: buildReport
//---------------------------------------------------------
bool UUV::buildReport()
{
	m_msgs << "UUV name:\t" << m_uuv_name <<  endl << endl;


	m_msgs << "UUV Sensors (" << m_uuv_sensors.size() << ")" << endl;
	m_msgs << "------------------------------------------------"   << endl;
	for (unsigned i=0; i<m_uuv_sensors.size(); i++){
		m_msgs << m_uuv_sensors.at(i) << endl;
	}
//	m_msgs << endl << endl;
//	for (unsigned i=0; i<m_uuv_sensors.size(); i++){
//		m_msgs << m_uuv_sensors.at(i) << endl;
//	}

	return true;
}


//---------------------------------------------------------
// Procedure: handleSensorsNames
// check if the provided string is in the format SENSOR_1SEart:end:degradationPercentage
// e.g. 50:100:50
//---------------------------------------------------------
bool UUV::handleSensorsNames(string value)
{
	vector<string> v = parseString(removeWhite(value),",");

	//check if all tokens are alphanumerics
	for (vector<string>::iterator it = v.begin();  it != v.end(); it++){
		if (!isAlphaNum(*it))
			reportConfigWarning("Problem with configuring 'SENSORS ="+ value +"': expected alphanumeric but received " + *it);
		else
			//if everything is OK, create a sensor element and add it to the vector
			m_uuv_sensors.push_back(*it);
	}

	return true;
}


//---------------------------------------------------------
// Procedure: initSensorsMap
//---------------------------------------------------------
void UUV::initSensorsMap()
{
	for (vector<string>::iterator it = m_uuv_sensors.begin();  it != m_uuv_sensors.end(); it++){
		Sensor newSensor;
		newSensor.name		 	= *it;
		newSensor.averageRate 	= 0;
		newSensor.numOfReadings = 0;
		newSensor.state			= -1;
		m_sensors_map[*it] = newSensor;
	}
}