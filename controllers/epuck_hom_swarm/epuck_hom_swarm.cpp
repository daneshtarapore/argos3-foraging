

/* Include the controller definition */
#include "epuck_hom_swarm.h"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

//#define DATA_BYTE_BOUND 240.0f
//UInt8 CEPuckHomSwarm::BEACON_SIGNAL = BEACON_SIGNAL;
//UInt8 CEPuckHomSwarm::NEST_BEACON_SIGNAL = NEST_BEACON_SIGNAL;


//#define SELF_INFO_PACKET 243 /* used to encompass info of self, be that the proprioceptively computed FVs, the bearings at which neighbours are observed, or proprioceptively computed angular acceleration.*/
//#define SELF_INFO_PACKET_FOOTER 244

//#define RELAY_FVS_PACKET 245
//#define RELAY_FVS_PACKET_FOOTER 246

//#define VOTER_PACKET 247
//#define ATTACK_VOTE 248
//#define TOLERATE_VOTE 249
//#define ATTACK_CONSENSUS 250
//#define TOLERATE_CONSENSUS 251
//#define VOTER_PACKET_FOOTER 252


//#define PROPRIOCEPT_MODE 0
//#define OBSERVATION_MODE 1
//#define COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE 2
//#define BAYESIANINFERENCE_MODE 3
//#define FV_MODE BAYESIANINFERENCE_MODE



//#define PROPRIOCEPT_MODE 0
//#define OBSERVATION_MODE 1
//#define COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE 2
//#define BAYESIANINFERENCE_MODE 3
//#define FV_MODE BAYESIANINFERENCE_MODE
///****************************************/
///****************************************/

///*
// * Probability to forget FV in distribution
// */
//#define PROBABILITY_FORGET_FV 1.0f //0.001f // We don't need a large history because FVs are being relayed every time-step. That combined with the BI of FVs (small observation window) means that robots will have sufficient FVs of neighbours to make a decision. Also it is difficult to assume that robot behavior has not changed in the last 100s.Will have a CRM_RESULTS_VALIDFOR_SECONDS history instead.

///*
// * Consensus threshold on FVs.
// */
//#define CONSENSUS_THRESHOLD 5u /* odd number so that we don't have a tie in attackers and tolerators - but this is just a threshold. number of voters may be more than threshold */

///*
// * The results of the CRM are valid for atmost 10s in the absence of any FVs to run the CRM
// */
//#define CRM_RESULTS_VALIDFOR_SECONDS 10.0f // in seconds // assume the robot behaviors have not changed in the last 10s // can we remove this.

///*
// * The vote counts and consensus are valid for atmost 10s before being refreshed
// */
//#define VOTCON_RESULTS_VALIDFOR_SECONDS 10.0f // in seconds //10.0f

/****************************************/
/****************************************/

CBehavior::SensoryData CBehavior::m_sSensoryData;
CBehavior::RobotData CBehavior::m_sRobotData;

CProprioceptiveFeatureVector::RobotData CProprioceptiveFeatureVector::m_sRobotData;

CObservedFeatureVector::RobotData CObservedFeatureVector::m_sRobotData;

CBayesianInferenceFeatureVector::RobotData CBayesianInferenceFeatureVector::m_sRobotData;

/****************************************/
/****************************************/

CEPuckHomSwarm::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_AGGREGATION),
    SBehavior_Trans(SWARM_NONE),
    FBehavior(FAULT_NONE),
    id_FaultyRobotInSwarm("-1"),
    time_between_robots_trans_behav(-1.0f)
{
    robot_ids_behav1.clear(); robot_ids_behav2.clear();    
}


void CEPuckHomSwarm::ExperimentToRun::Init(TConfigurationNode& t_node)
{
    std::string errorbehav;
    std::string swarmbehav_trans, time_between_robots_trans_behav__str;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
        GetNodeAttribute(t_node, "fault_behavior", errorbehav);
        GetNodeAttribute(t_node, "id_faulty_robot", id_FaultyRobotInSwarm);

        GetNodeAttribute(t_node, "swarm_behavior_trans", swarmbehav_trans);
        GetNodeAttribute(t_node, "time_between_robots_trans_behav", time_between_robots_trans_behav__str);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);

    if (swarmbehav.compare("SWARM_AGGREGATION") == 0)
        SBehavior = SWARM_AGGREGATION;
    else if (swarmbehav.compare("SWARM_DISPERSION") == 0)
        SBehavior = SWARM_DISPERSION;
    else if (swarmbehav.compare("SWARM_FLOCKING") == 0)
        SBehavior = SWARM_FLOCKING;
    else if (swarmbehav.compare("SWARM_HOMING") == 0)
        SBehavior = SWARM_HOMING;
    else if (swarmbehav.compare("SWARM_STOP") == 0)
        SBehavior = SWARM_STOP;
    else
    {
        std::cerr << "invalid swarm behavior";
        assert(-1);
    }

    if (errorbehav.compare("FAULT_NONE") == 0)
        FBehavior = FAULT_NONE;
    else if  (errorbehav.compare("FAULT_STRAIGHTLINE") == 0)
        FBehavior = FAULT_STRAIGHTLINE;
    else if  (errorbehav.compare("FAULT_RANDOMWALK") == 0)
        FBehavior = FAULT_RANDOMWALK;
    else if  (errorbehav.compare("FAULT_CIRCLE") == 0)
        FBehavior = FAULT_CIRCLE;
    else if  (errorbehav.compare("FAULT_STOP") == 0)
        FBehavior = FAULT_STOP;


    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMIN") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMIN;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMAX") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMAX;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETRANDOM") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETRANDOM;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETOFFSET") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETOFFSET;


    else if  (errorbehav.compare("FAULT_RABSENSOR_SETOFFSET") == 0)
        FBehavior = FAULT_RABSENSOR_SETOFFSET;
    else if  (errorbehav.compare("FAULT_RABSENSOR_MISSINGRECEIVERS") == 0)
        FBehavior = FAULT_RABSENSOR_MISSINGRECEIVERS;


    else if  (errorbehav.compare("FAULT_ACTUATOR_LWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_LWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_RWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_RWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_BWHEELS_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_BWHEELS_SETZERO;

    else if  (errorbehav.compare("FAULT_SOFTWARE") == 0)
        FBehavior = FAULT_SOFTWARE;

    else if  (errorbehav.compare("FAULT_POWER_FAILURE") == 0)
        FBehavior = FAULT_POWER_FAILURE;

    else
    {
        std::cerr << "invalid fault behavior";
        assert(-1);
    }


    /* Transition to behavior */
    if (swarmbehav_trans.compare("SWARM_AGGREGATION") == 0)
        SBehavior_Trans = SWARM_AGGREGATION;
    else if (swarmbehav_trans.compare("SWARM_DISPERSION") == 0)
        SBehavior_Trans = SWARM_DISPERSION;
    else if (swarmbehav_trans.compare("SWARM_FLOCKING") == 0)
        SBehavior_Trans = SWARM_FLOCKING;
    else if (swarmbehav_trans.compare("SWARM_HOMING") == 0)
        SBehavior_Trans = SWARM_HOMING;
    else if (swarmbehav_trans.compare("SWARM_STOP") == 0)
        SBehavior_Trans = SWARM_STOP;
    else if (swarmbehav_trans.compare("") == 0)
        SBehavior_Trans = SWARM_NONE;
    else
    {
        std::cerr << "invalid swarm transition behavior";
        assert(-1);
    }

    time_between_robots_trans_behav = strtold(time_between_robots_trans_behav__str.c_str(),NULL);
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::SWheelTurningParams::Init(TConfigurationNode& t_node)
{
    try
    {
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);

    //std::cout << " MaxSpeed " << MaxSpeed << std::endl;
}

/****************************************/
/****************************************/

CEPuckHomSwarm::CEPuckHomSwarm() :
    m_pcWheels(NULL),
    m_pcLEDs(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcProximity(NULL),
    m_pcRNG(CRandom::CreateRNG("argos")),
    m_pcRNG_FVs(CRandom::CreateRNG("argos")),
    b_damagedrobot(false),
    u_num_consequtivecollisions(0)
{
#ifndef DESYNC_ROB_CLOCK
    m_fRobotTimerAtStart = 0.0f;
#else
    // desync clocks by +/-5 s - Gaussian dist
    m_fRobotTimerAtStart = m_pcRNG->Gaussian(25.0f, 50.0f); // mean 50 ticsk, std dev. 10 ticks (50 ticks = 5 sec)
    if(m_fRobotTimerAtStart < 0.0f)
        m_fRobotTimerAtStart = 0.0f;
    else if(m_fRobotTimerAtStart > 100.0f)
        m_fRobotTimerAtStart = 100.0f;
    else
        m_fRobotTimerAtStart = (unsigned) m_fRobotTimerAtStart;
#endif

    m_fInternalRobotTimer = m_fRobotTimerAtStart;
    listFVsSensed.clear();
    listMapFVsToRobotIds.clear();
    listMapFVsToRobotIds_relay.clear();
    listConsensusInfoOnRobotIds.clear();
    listVoteInformationRobots.clear();

    b_CRM_Run = false;
    m_fCRM_RUN_TIMESTAMP = 0.0f;
    m_uRobotFV = 9999; // for debugging urposes
}

/****************************************/
/****************************************/

CEPuckHomSwarm::~CEPuckHomSwarm()
{
    // delete all behaviors

    // delete list of feature-vectors

    // delete crm model's data containers
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Init(TConfigurationNode& t_node)
{
    try
    {
        /*
       * Initialize sensors/actuators
       */
        m_pcWheels        = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcLEDs          = GetActuator<CCI_LEDsActuator                >("leds"                 );
        m_pcRABA          = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
        m_pcRABS          = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
        m_pcProximity     = GetSensor  <CCI_EPuckProximitySensor        >("epuck_proximity"    );
        m_pcWheelsEncoder = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");

        /*
       * Parse XML parameters
       */
        /* Experiment to run */
        m_sExpRun.Init(GetNode(t_node, "experiment_run"));
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the e-puck hom_swarm controller for robot \"" << GetId() << "\"", ex);

    /*
    * Initialize other stuff
    */
    /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
    // Now initialised at CEPuckHomSwarm() constructor
    /*m_pcRNG     = CRandom::CreateRNG("argos");
    m_pcRNG_FVs = CRandom::CreateRNG("argos");*/

    Reset();

    m_sRobotDetails.SetKinematicDetails(m_sWheelTurningParams.MaxSpeed, m_sWheelTurningParams.MaxSpeed);

    CopyRobotDetails(m_sRobotDetails);


    m_pFlockingBehavior = new CFlockingBehavior(m_sRobotDetails.iterations_per_second * 1.0f); // 5.0f

    if(this->GetId().compare("ep"+m_sExpRun.id_FaultyRobotInSwarm) == 0)
        b_damagedrobot = true;

    // robotid set to 0 for now
    crminAgent = new CRMinRobotAgentOptimised(RobotIdStrToInt(), CProprioceptiveFeatureVector::NUMBER_OF_FEATURES);
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::CopyRobotDetails(RobotDetails& robdetails)
{
    //std::cout << " robdetails.MaxLinearSpeed " << robdetails.MaxLinearSpeed << std::endl;
    //std::cout << " robdetails.iterations_per_second " << robdetails.iterations_per_second << std::endl;

    CBehavior::m_sRobotData.MaxSpeed                    = robdetails.MaxLinearSpeed * robdetails.iterations_per_second; // max speed in cm/s to control behavior
    CBehavior::m_sRobotData.iterations_per_second       = robdetails.iterations_per_second;
    CBehavior::m_sRobotData.seconds_per_iterations      = 1.0f / robdetails.iterations_per_second;
    CBehavior::m_sRobotData.HALF_INTERWHEEL_DISTANCE    = robdetails.HALF_INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.INTERWHEEL_DISTANCE         = robdetails.INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.WHEEL_RADIUS                = robdetails.WHEEL_RADIUS;

    CBehavior::m_sRobotData.m_cNoTurnOnAngleThreshold   = robdetails.m_cNoTurnOnAngleThreshold;
    CBehavior::m_sRobotData.m_cSoftTurnOnAngleThreshold = robdetails.m_cSoftTurnOnAngleThreshold;

    CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CBehavior::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CBehavior::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CBehavior::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CBehavior::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CBehavior::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CBehavior::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CBehavior::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;




    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CProprioceptiveFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;



    CObservedFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CObservedFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CObservedFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CObservedFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CObservedFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CObservedFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CObservedFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CObservedFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CObservedFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;



    CObservedFeatureVector::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CObservedFeatureVector::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CObservedFeatureVector::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CObservedFeatureVector::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CObservedFeatureVector::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CObservedFeatureVector::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CObservedFeatureVector::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;




    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CBayesianInferenceFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CBayesianInferenceFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;

    CBayesianInferenceFeatureVector::m_sRobotData.SetLengthOdometryTimeWindows();

    CBayesianInferenceFeatureVector::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CBayesianInferenceFeatureVector::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CBayesianInferenceFeatureVector::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CBayesianInferenceFeatureVector::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;
}

/****************************************/
/****************************************/

unsigned CEPuckHomSwarm::SumFVDist(t_listFVsSensed& FVsSensed)
{
    unsigned robotcount = 0;
    for (t_listFVsSensed::iterator it = FVsSensed.begin(); it != FVsSensed.end(); ++it)
        robotcount += it->fRobots;

    return robotcount;
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ControlStep()
{
  //std::cout << RobotIdStrToInt() << " " << m_fInternalRobotTimer << std::endl;

    if(m_fInternalRobotTimer == m_fRobotTimerAtStart)
    {
        /*init function of loopfunction object (used to count number of e-pucks) is called after calling init of individual robot objects, and not before; so the u_num_epucks is used here*/
        assert(m_sExpRun.u_num_epucks <= 100u);
        for(unsigned id = 0; id < m_sExpRun.u_num_epucks; ++id)
            m_sExpRun.robot_ids_behav1.push_back(id);
    }


    m_pcRABA->ClearData(); // clear the channel at the start of each control cycle

    m_fInternalRobotTimer+=1.0f;

    bool b_RunningGeneralFaults(false);
    if(b_damagedrobot && (m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP))
    {
        b_RunningGeneralFaults = true;
        RunGeneralFaults();
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION  ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING    ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING      ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_STOP)
        RunHomogeneousSwarmExperiment();


    if(!b_damagedrobot || b_RunningGeneralFaults || m_sExpRun.FBehavior == ExperimentToRun::FAULT_NONE)
        CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    else
    {
        //m_pcLEDs->SetAllColors(CColor::RED);

        if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMIN)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMAX)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETRANDOM)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_MISSINGRECEIVERS)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_SOFTWARE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));

        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_POWER_FAILURE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    }

    /*For flocking behavior - to compute relative velocity*/
    CBehavior::m_sSensoryData.SetWheelSpeedsFromEncoders(m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);

    /*The robot has to continually track the velocity of its neighbours - since this is done over a period of time. It can't wait until the flocking behavior is activated to start tracking neighbours*/
    m_pFlockingBehavior->SimulationStep();


    Real leftSpeed = 0.0, rightSpeed = 0.0;
    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        if (!bControlTaken)
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
            {
                /*if(b_damagedrobot)
                    (*i)->PrintBehaviorIdentity();*/
                /*(*i)->PrintBehaviorIdentity();*/
                (*i)->Action(leftSpeed, rightSpeed);
            }
        } else
            (*i)->Suppress();
    }

    /*If robot is contantly colliding against a wall, half the speed at which the wheels rotate - to make the robot movement closer to reality. We use the IR sensors to detect this scenario and we can do this even when there is a sensor fault as in reality the speed would reduce on its own when the robot is stuck to a wall*/
    /*Using the noiseless variant of the IR sensors for this detection*/

    /*if(b_damagedrobot)
    std::cout << "IR0 " << m_pcProximity->GetReadings_Noiseless()[0].Value <<
                 "   IR1 " << m_pcProximity->GetReadings_Noiseless()[1].Value <<
                 "   IR2 " << m_pcProximity->GetReadings_Noiseless()[2].Value <<
                 "   IR3 " << m_pcProximity->GetReadings_Noiseless()[3].Value <<
                 "   IR4 " << m_pcProximity->GetReadings_Noiseless()[4].Value <<
                 "   IR5 " << m_pcProximity->GetReadings_Noiseless()[5].Value <<
                 "   IR6 " << m_pcProximity->GetReadings_Noiseless()[6].Value <<
                 "   IR7 " << m_pcProximity->GetReadings_Noiseless()[7].Value << std::endl;*/

    /*if(!b_damagedrobot)
    std::cerr << "IR0 " << m_pcProximity->GetReadings_Noiseless()[0].Value <<
                 "   IR1 " << m_pcProximity->GetReadings_Noiseless()[1].Value <<
                 "   IR2 " << m_pcProximity->GetReadings_Noiseless()[2].Value <<
                 "   IR3 " << m_pcProximity->GetReadings_Noiseless()[3].Value <<
                 "   IR4 " << m_pcProximity->GetReadings_Noiseless()[4].Value <<
                 "   IR5 " << m_pcProximity->GetReadings_Noiseless()[5].Value <<
                 "   IR6 " << m_pcProximity->GetReadings_Noiseless()[6].Value <<
                 "   IR7 " << m_pcProximity->GetReadings_Noiseless()[7].Value << std::endl;*/

    if(m_pcProximity->GetReadings_Noiseless()[0].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[1].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[2].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[3].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[4].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[5].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[6].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[7].Value > 0.4f)
        u_num_consequtivecollisions++;
    else
        u_num_consequtivecollisions = 0u;


    // if the robot is colliding with the wall other robot for more than 1s, we reduce its speed by half
    /* this will be harder to detect when we add noise on the IR sensors. Be wary of that. So using the noiseless variant of the IR sensors for this detection*/
    if((Real)u_num_consequtivecollisions > (m_sRobotDetails.iterations_per_second * 1.0f))
    {
        leftSpeed = leftSpeed/2.0f;
        rightSpeed = rightSpeed/2.0f;
    }



    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        leftSpeed  = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        rightSpeed = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
    {
        leftSpeed = 0.0f;
        rightSpeed = 0.0f;
    }

    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed); // in cm/s

    //std::cout << "LS:  " << leftSpeed << " RS:  " << rightSpeed << std::endl;

    CCI_RangeAndBearingSensor::TReadings rabsensor_readings = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);

    m_uRobotId = RobotIdStrToInt();
    SenseCommunicateDetect(RobotIdStrToInt(), m_pcRABA, m_pcWheelsEncoder,
                           m_fInternalRobotTimer, rabsensor_readings,
                           listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed, listVoteInformationRobots, listConsensusInfoOnRobotIds,
                           m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                           b_CRM_Run, m_fCRM_RUN_TIMESTAMP, crminAgent, m_pcRNG_FVs, m_uRobotFV, m_sExpRun.swarmbehav, beaconrobots_ids);

//    if(m_uRobotId == 0)
//        m_pcWheels->SetLinearVelocity(0, 0);

//    CCI_RangeAndBearingSensor::TReadings rab_packet = m_pcRABS->GetReadings();
//    if(m_uRobotId == 0)
//        for(size_t i = 0; i < rab_packet.size(); ++i)
//        {
//            std::cout << "Range " << rab_packet[i].Range << " Bearing " << rab_packet[i].HorizontalBearing << std::endl;
//        }


//    /****************************************/
//#if FV_MODE == PROPRIOCEPT_MODE

//    /* Estimate feature-vectors - proprioceptively */

//    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
//    m_cProprioceptiveFeatureVector.SimulationStep();

//    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue(); // to debug
//    m_uRobotId = RobotIdStrToInt();

//    /* Communicate your id and proprioceptively computed FV to whoever is in range, using the RAB sensor*/
//    /* Also relay the id and fvs of neighbours, received by you in the previous control cycle */
//    //if ((unsigned)m_fInternalRobotTimer%2u == 0)

//    //printf(" SendFVsToNeighbours() \n\n\n");
//    SendFVsToNeighbours();

//    /* Listen for robot ids + feature vectors from neighbours and then assimilate them  */
//    //printf(" Sense(PROBABILITY_FORGET_FV); \n\n\n");
//    Sense(PROBABILITY_FORGET_FV);
//#endif
//    /****************************************/



//    /****************************************/
//#if FV_MODE == OBSERVATION_MODE || FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE

//    /* Estimating FVs proprioceptively - to be used for the simplifying fault detection and to compute angular acceleration for the COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE */
//    /*m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 leftSpeed, rightSpeed);*/
//    /*encoders give you the speed at the previous tick not current tick */
//    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);


//    m_cProprioceptiveFeatureVector.SimulationStep();
//    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();

//    /* Estimate feature-vectors - via observation */
//    m_uRobotId = RobotIdStrToInt();


//    /*m_cObservationFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                              leftSpeed, rightSpeed);*/
//    /*encoders give you the speed at the previous tick not current tick */
//    m_cObservationFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                              m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
//    m_cObservationFeatureVector.SimulationStep();

//    Sense(PROBABILITY_FORGET_FV);

//    //if ( ((unsigned)m_fInternalRobotTimer%2u == 0) || (m_fInternalRobotTimer <= MODELSTARTTIME))
//    /*
//     * Send the robot id and the bearing at which it observes its different neighbours. Also relay the observed FVs
//     */

//    SendIdSelfBearingAndObsFVsToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector, RobotIdStrToInt(),
//                                           GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), listMapFVsToRobotIds_relay);
//#endif
//    /****************************************/

//    /****************************************/
//#if FV_MODE == BAYESIANINFERENCE_MODE

//    /* Estimating FVs proprioceptively - to be used for computing angular acceleration*/
//    /*encoders give you the speed at the previous tick not current tick */
//    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);

//    m_cProprioceptiveFeatureVector.SimulationStep();
//    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();

//    /* Estimate feature-vectors - via observation */
//    m_uRobotId = RobotIdStrToInt();


//    /*encoders give you the speed at the previous tick not current tick */
//    m_cBayesianInferredFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                   m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
//    m_cBayesianInferredFeatureVector.SimulationStep();

//    Sense(PROBABILITY_FORGET_FV);

//    SendIdSelfBearingAndObsFVsToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector, RobotIdStrToInt(),
//                                           GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), listMapFVsToRobotIds_relay);
//#endif
//    /****************************************/

//    if(((unsigned)m_fInternalRobotTimer % (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) == 0u)
//        // to avoid consensus already in the medium to establish itself in the next step. when the robot clocks are not in sync, this period would have to be longer than just 2 iterations
//    {
//        listConsensusInfoOnRobotIds.clear();
//        listVoteInformationRobots.clear();
//    }
//    else if(m_fInternalRobotTimer > MODELSTARTTIME)
//        /* else because you don't want to receive consensus already in the medium from before the buffer was cleared*/
//    {
//        /* Listen for voting packets and consensus packets from neighbours*/
//        ReceiveVotesAndConsensus(listVoteInformationRobots, listMapFVsToRobotIds, listConsensusInfoOnRobotIds, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
//        EstablishConsensus(m_fInternalRobotTimer, listVoteInformationRobots, listConsensusInfoOnRobotIds);
//    }


//    Real TimeSinceCRM = (m_fInternalRobotTimer - m_fCRM_RUN_TIMESTAMP) * CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations; // in seconds
//    if (b_CRM_Run && (TimeSinceCRM > CRM_RESULTS_VALIDFOR_SECONDS)) /* the results of the CRM are no longer valid */
//        b_CRM_Run = false;

//    if((m_fInternalRobotTimer > MODELSTARTTIME) && (listFVsSensed.size() > 0))
//        // the robot has atleast had one FV entry in its distribution. if not the CRM will crash.
//    {
//        crminAgent->SimulationStepUpdatePosition(m_fInternalRobotTimer, &listFVsSensed);
//        b_CRM_Run = true;
//        m_fCRM_RUN_TIMESTAMP = m_fInternalRobotTimer;
//    }

//    if(b_CRM_Run) // a failsafe to make sure you don't use outdated CRM results
//    {
//        // the CRM results on FVs in listFVsSensed is not outdated
//        for(t_listFVsSensed::iterator it_fv = listFVsSensed.begin(); it_fv != listFVsSensed.end(); ++it_fv)
//        {
//#ifndef FILTER_BEFORE_VOTE
//            UpdateVoterRegistry(listVoteInformationRobots,
//                                listMapFVsToRobotIds,
//                                listConsensusInfoOnRobotIds,
//                                RobotIdStrToInt(), it_fv->uFV, it_fv->uMostWantedState);
//#else
//            Real m_fVoteCompilationStartTime = (Real)(((unsigned)m_fInternalRobotTimer)/
//                                                 ((unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second))) *
//                                          (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second);


//            if(((m_fInternalRobotTimer - m_fVoteCompilationStartTime)/ (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) > 0.9f)
//            {
//                UpdateVoterRegistry(listVoteInformationRobots,
//                                    listMapFVsToRobotIds,
//                                    listConsensusInfoOnRobotIds,
//                                    RobotIdStrToInt(), it_fv->uFV, it_fv->uMostWantedState, true);
//            }
//            else
//            {
//                IntegrateAttackTolerateDecisions(listMapFVsToRobotIds, it_fv->uFV, it_fv->uMostWantedState);
//            }

//#endif
//        }
//    }

//    if(((unsigned)m_fInternalRobotTimer % (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) != 0u) /* dont send CRM results if buffer is cleared*/
//    {
//        if ((m_fInternalRobotTimer > MODELSTARTTIME)) // && (unsigned)m_fInternalRobotTimer%2u == 1)
//        {
//#ifndef FILTER_BEFORE_VOTE
//            SendCRMResultsAndConsensusToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
//                                                   RobotIdStrToInt(), listMapFVsToRobotIds,
//                                                   listFVsSensed, listConsensusInfoOnRobotIds, b_CRM_Run); // only send CRM results if they are valid
//#else
//            Real m_fVoteCompilationStartTime = (Real)(((unsigned)m_fInternalRobotTimer)/
//                                                 ((unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second))) *
//                                          (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second);


//            if(((m_fInternalRobotTimer - m_fVoteCompilationStartTime)/ (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) > 0.9f)
//            {
//                for (t_listFVsSensed::iterator it_fvdist = listFVsSensed.begin(); it_fvdist != listFVsSensed.end(); ++it_fvdist)
//                {
//                    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
//                    {
//                        if(it_map->uFV == it_fvdist->uFV)
//                        {
//                                if (it_map->f_TimesAttacked / (it_map->f_TimesAttacked + it_map->f_TimesTolerated) > 0.5f)
//                                {
//                                    it_fvdist->uMostWantedState = 1;
//                                }
//                                else
//                                {
//                                    it_fvdist->uMostWantedState = 2;
//                                }
//                                break;
//                        }
//                    }
//                }
//                SendCRMResultsAndConsensusToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
//                                                       RobotIdStrToInt(), listMapFVsToRobotIds,
//                                                       listFVsSensed, listConsensusInfoOnRobotIds, b_CRM_Run); // only send CRM results if they are valid
//            }
//#endif
//        }
//    }


    /*if((RobotIdStrToInt() == 6 || RobotIdStrToInt() == 13) && ((m_fInternalRobotTimer == 1701.0f || m_fInternalRobotTimer == 1702.0f)  || (m_fInternalRobotTimer == 1801.0f || m_fInternalRobotTimer == 1802.0f)  || (m_fInternalRobotTimer == 1901.0f || m_fInternalRobotTimer == 1902.0f)))
    {
        PrintVoterRegistry(RobotIdStrToInt(), listVoteInformationRobots, 9);
        PrintConsensusRegistry(RobotIdStrToInt(), listConsensusInfoOnRobotIds, 9);
    }

    if((RobotIdStrToInt() == 0) && ((m_fInternalRobotTimer == 1701.0f || m_fInternalRobotTimer == 1702.0f)  || (m_fInternalRobotTimer == 1801.0f || m_fInternalRobotTimer == 1802.0f)  || (m_fInternalRobotTimer == 1901.0f || m_fInternalRobotTimer == 1902.0f)))
    {
        crminAgent->PrintFeatureVectorDistribution(0);
    }*/

}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::SendIdSelfBearingAndObsFVsToNeighbours(const CCI_RangeAndBearingSensor::TReadings& tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay)
//{
//    /*Communicate your id to neighbours, so they know who they are observing*/
//    /*Also communicate the bearing at which you observed the neighbours */
//    /*Also communicate the FVs you have observed */

//    WriteToCommunicationChannel(m_pcRABA,  m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
//                                RobotIdStrToInt(), tPackets, IdToFVsMap_torelay);

//    //WriteToCommunicationChannel(RobotIdStrToInt(), tPackets, IdToFVsMap_torelay);

//}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::SendFVsToNeighbours()
//{
//    /*Communicate your id and FV, and relay the id and fvs of neighbours, received by you in the previous control cycle*/
//    WriteToCommunicationChannel(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
//                                RobotIdStrToInt(), m_cProprioceptiveFeatureVector.GetValue(), listMapFVsToRobotIds_relay);
//}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::SendCRMResultsAndConsensusToNeighbours(bool b_CRM_Results_Valid)
//{
//    /* Commmunicate your CRM results to your neighbours
//     * Also broadcast consensus information - add any new consensus info to your local list and send it out again */

//    WriteToCommunicationChannel(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
//                                RobotIdStrToInt(), listMapFVsToRobotIds,
//                                listFVsSensed, listConsensusInfoOnRobotIds, b_CRM_Results_Valid);

//    //printf(" finished WriteToCommunicationChannel \n\n");
//}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::Sense(Real m_fProbForget)
//{
//#if FV_MODE == PROPRIOCEPT_MODE

//#ifdef ConsensusOnMapOfIDtoFV
//    exit(-1);
//#endif

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);

//    /* Listen for feature vectors from neighbours */
//    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
//    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
//    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */
//    bool read_status = ReadFromCommunicationChannel_IdFv(m_fInternalRobotTimer, listMapFVsToRobotIds_relay, listMapFVsToRobotIds, tmp); /* returns true if successfully read id and fvs from at least one neighbour*/


//    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS); /*remove entries older than 10s */

//    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed
//#endif

//#if FV_MODE == OBSERVATION_MODE || FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE

//#ifdef ConsensusOnMapOfIDtoFV
//    exit(-1);
//#endif

//    listMapFVsToRobotIds_relay.clear();
//    for (size_t i = 0; i < m_cObservationFeatureVector.ObservedRobotIDs.size(); ++i)
//    {
//        unsigned robotId = m_cObservationFeatureVector.ObservedRobotIDs[i];
//        unsigned fv      = m_cObservationFeatureVector.ObservedRobotFVs[i];

//        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));

//        //if(robotId == 15)
//        //  std::cerr << "Observer: " << m_uRobotId << " ObservedId " << robotId << " ObservedFV " << fv << std::endl;

//        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
//    }

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
//    bool read_status = ReadFromCommunicationChannel_RelayedFv(m_fInternalRobotTimer, listMapFVsToRobotIds, tmp); /* returns true if successfully read id and fvs from at least one neighbour*/

//    //remove entries older than 10s
//    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);
//    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed
//#endif

//#if FV_MODE == BAYESIANINFERENCE_MODE

//    listMapFVsToRobotIds_relay.clear();
//    for (size_t i = 0; i < m_cBayesianInferredFeatureVector.ObservedRobotIDs.size(); ++i)
//    {
//        unsigned robotId = m_cBayesianInferredFeatureVector.ObservedRobotIDs[i];
//        unsigned fv      = m_cBayesianInferredFeatureVector.ObservedRobotFVs[i];
//        unsigned num_observations = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Min_Number_Featureobservations[i];

//        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));


//#ifndef ConsensusOnMapOfIDtoFV
//        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
//#else
//        UpdateFvToRobotIdMap(listMapFVsToRobotIds, RobotIdStrToInt(), fv, robotId, m_fInternalRobotTimer);
//#endif
//    }

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
//    bool read_status = ReadFromCommunicationChannel_RelayedFv(m_fInternalRobotTimer, listMapFVsToRobotIds, tmp); /* returns true if successfully read id and fvs from at least one neighbour*/

//    //remove entries older than 10s
//    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);

//#ifdef ConsensusOnMapOfIDtoFV
//    SelectBestFVFromAllObservedFVs(listMapFVsToRobotIds, CProprioceptiveFeatureVector::NUMBER_OF_FEATURES, m_pcRNG_FVs);
//#endif


//    /*if((RobotIdStrToInt() == 6 || RobotIdStrToInt() == 13) && ((m_fInternalRobotTimer == 1701.0f || m_fInternalRobotTimer == 1702.0f)  || (m_fInternalRobotTimer == 1801.0f || m_fInternalRobotTimer == 1802.0f)  || (m_fInternalRobotTimer == 1901.0f || m_fInternalRobotTimer == 1902.0f)))
//    //if(RobotIdStrToInt() == 6)
//        PrintFvToRobotIdMap(RobotIdStrToInt(), listMapFVsToRobotIds, 9);*/

//    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed

//#endif
//}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::ReceiveVotesAndConsensus()
//{
//    /* Listen to votes and consensus from neighbours */
//    /* Read the voter id:
//     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
//     * If a vote is received,
//     *                      1. map the fv to the robot id (if none existed - ignore vote???)
//     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
//     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
//     *
//     * If a consensus is received,
//     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
//     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
//     *
//    */

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
//    bool read_status = ReadFromCommunicationChannel_VotCon(listVoteInformationRobots, listMapFVsToRobotIds, listConsensusInfoOnRobotIds, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior)); /* returns true if successfully read votes or consensus from at least one neighbour*/
//}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::EstablishConsensus()
//{
//    /* For each robot id in listVoteInformationRobots that is not in listConsensusInfoOnRobotIds
//     * If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining) (not anymore)
//     * Establish temporary consensus on robot id by adding it to listConsensusInfoOnRobotIds
//     */

//    for (t_listVoteInformationRobots::iterator it_vot = listVoteInformationRobots.begin(); it_vot != listVoteInformationRobots.end(); ++it_vot)
//    {
//        unsigned VotedOnRobotId = it_vot->uRobotId;
//        bool b_ConsensusReachedOnId(false);

//        for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
//        {
//            if (it_cons->uRobotId == VotedOnRobotId)
//            {
//                b_ConsensusReachedOnId = true;
//                break; /* the robot id in consensus list are unique */
//            }

//        }

//        if (b_ConsensusReachedOnId)
//            continue; /* consensus already reached for VotedOnRobotId, lets go to the next robot in the listVoteInformationRobots list */
//        else
//        {
//            /*bool b_OneSecondToVotConReset = (((unsigned)m_fInternalRobotTimer %
//                                              (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) >= 9u);*/
//            bool b_OneSecondToVotConReset(false);


//            /* If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining) after which the consensus and vote vectors will be cleared (not anymore) */
//            if ((it_vot->uVoterIds.size() >= CONSENSUS_THRESHOLD) || b_OneSecondToVotConReset) /* at least one vote will be registered. establish consensus on that */
//            {
//                /*if(RobotIdStrToInt() == 19 && VotedOnRobotId==15 && (m_fInternalRobotTimer >= 700 && m_fInternalRobotTimer <= 705))
//                {
//                    for(std::list<unsigned>::iterator tmp = it_vot->uVoterIds.begin(); tmp != it_vot->uVoterIds.end(); ++tmp)
//                    {
//                        std::cout << "In voter list at m_fInternalRobotTimer " << m_fInternalRobotTimer << " voter ids" <<  (*tmp) << std::endl;
//                    }
//                }*/

//                it_vot->fTimeConsensusReached = m_fInternalRobotTimer;
//                listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(it_vot->uRobotId,
//                                                                                 (it_vot->attackvote_count > it_vot->toleratevote_count)?1u:2u)); /* if equal votes, we tolerate robot*/
//            }
//        }
//    }
//}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunGeneralFaults()
{
    m_pcLEDs->SetAllColors(CColor::RED);

    m_vecBehaviors.clear();
    if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE)
    {
        CRandomWalkBehavior* pcStraightLineBehavior = new CRandomWalkBehavior(0.0f);
        m_vecBehaviors.push_back(pcStraightLineBehavior);
    }
    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);  // 0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE)
    {
        CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
        m_vecBehaviors.push_back(pcCircleBehavior);
    }

    else //m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP
    {}
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunHomogeneousSwarmExperiment()
{
    m_vecBehaviors.clear();

    float start_firsttrans_sec = 500.0f;

    if(m_sExpRun.SBehavior_Trans != ExperimentToRun::SWARM_NONE)// && m_sExpRun.time_between_robots_trans_behav > 0.0f)
    {
        if(m_fInternalRobotTimer / m_sRobotDetails.iterations_per_second < start_firsttrans_sec)
        {
            m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior;
        }
        /*
         * else if((m_fInternalRobotTimer / m_sRobotDetails.iterations_per_second >= 500.0f) && (m_fInternalRobotTimer  / m_sRobotDetails.iterations_per_second < 1000.0f))
        {
            std::list<unsigned>::iterator findIter = std::find(m_sExpRun.robot_ids_behav1.begin(), m_sExpRun.robot_ids_behav1.end(), RobotIdStrToInt());
            if ( m_sExpRun.robot_ids_behav1.end() == findIter ) // not in list1
                m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior_Trans;
            else
                m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior;
        }
        */
        else
        {
            std::list<unsigned>::iterator findIter = std::find(m_sExpRun.robot_ids_behav1.begin(), m_sExpRun.robot_ids_behav1.end(), RobotIdStrToInt());
            if ( m_sExpRun.robot_ids_behav1.end() == findIter ) // not in list1
                m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior_Trans;
            else
                m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior;
        }
    }
    else
         m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior;




    //if(RobotIdStrToInt()>0 || (RobotIdStrToInt()==0 && m_fInternalRobotTimer <= 2500.0f))
    if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_AGGREGATION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(100.0f); //range threshold in cm //60.0
        m_vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        //m_pcLEDs->SetAllColors(CColor::GREEN);
    }

    //else if((RobotIdStrToInt()==0 && m_fInternalRobotTimer > 2500.0f))
    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_DISPERSION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        //m_pcLEDs->SetAllColors(CColor::RED);
    }

    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_FLOCKING)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        m_vecBehaviors.push_back(m_pFlockingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_HOMING)
    {
        if(this->GetId().compare("ep0") == 0)
        {
            // ep0 is the beacon robot
            /* Sends out data 'BEACON_SIGNAL' with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
            // BEACON_SIGNAL is way above the DATA_BYTE_BOUND

            m_pcRABA->SetData(0, BEACON_SIGNAL);
            //m_pcLEDs->SetAllColors(CColor::YELLOW);
        }
        else
        {
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
            m_vecBehaviors.push_back(pcDisperseBehavior);

            Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
            CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
            m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }

        // Homing disabled as the beacon signal data will interfere with the FV data
        //exit(-1);
    }
    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_STOP)
    {
    }
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Reset()
{
    /* Set LED color */
    //m_pcLEDs->SetAllColors(CColor::WHITE);
    m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

unsigned CEPuckHomSwarm::RobotIdStrToInt()
{
    std::string id = GetId();
    id.erase(0, 2); // remove the first two characters

    std::string::size_type sz;   // alias of size_t
    unsigned u_id = std::stoi(id, &sz);
    return u_id;


    if(id.compare("ep0")==0)
        return 0;

    else if(id.compare("ep1")==0)
        return 1;

    else if(id.compare("ep2")==0)
        return 2;

    else if(id.compare("ep3")==0)
        return 3;

    else if(id.compare("ep4")==0)
        return 4;

    else if(id.compare("ep5")==0)
        return 5;

    else if(id.compare("ep6")==0)
        return 6;

    else if(id.compare("ep7")==0)
        return 7;

    else if(id.compare("ep8")==0)
        return 8;

    else if(id.compare("ep9")==0)
        return 9;

    else if(id.compare("ep10")==0)
        return 10;

    else if(id.compare("ep11")==0)
        return 11;

    else if(id.compare("ep12")==0)
        return 12;

    else if(id.compare("ep13")==0)
        return 13;

    else if(id.compare("ep14")==0)
        return 14;

    else if(id.compare("ep15")==0)
        return 15;

    else if(id.compare("ep16")==0)
        return 16;

    else if(id.compare("ep17")==0)
        return 17;

    else if(id.compare("ep18")==0)
        return 18;

    else if(id.compare("ep19")==0)
        return 19;

    else
        LOGERR << "We can't be here, there's a bug!" << std::endl;
}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings& tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay)
//{
//    size_t databyte_index;

//    if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//        databyte_index = 0;


//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET);
//    m_pcRABA->SetData(databyte_index++, SelfId);

//#if FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE || FV_MODE == BAYESIANINFERENCE_MODE

//    // angular acceleration [-1, +1] to [0, DATA_BYTE_BOUND]
//    unsigned un_angularacceleration = (unsigned)(((m_cProprioceptiveFeatureVector.m_sSensoryData.GetNormalisedAngularAcceleration() + 1.0) / 2.0) * DATA_BYTE_BOUND);
//    m_pcRABA->SetData(databyte_index++, un_angularacceleration);

//#elif FV_MODE == OBSERVATION_MODE
//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0; unsigned robotId, un_bearing; CRadians bearing;

//        if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
//            byte_index = 1;
//        else
//            byte_index = 0;

//        byte_index++; // the message header type. this is always followed by the robot id.

//        robotId = tPackets[i].Data[byte_index];
//        bearing = tPackets[i].HorizontalBearing;
//        un_bearing = (unsigned)(ToDegrees(bearing).UnsignedNormalize().GetValue() * DATA_BYTE_BOUND / 360.0f);

//        /*if (SelfId == 15 && robotId == 17)
//        {
//            std::cerr << ToDegrees(bearing).UnsignedNormalize() << " " << ToDegrees(bearing).UnsignedNormalize().GetValue() << std::endl;
//            std::cerr << ToDegrees(bearing).UnsignedNormalize().GetValue() * (DATA_BYTE_BOUND / 360.0f) << " " << (un_bearing * 360.0f / DATA_BYTE_BOUND) << std::endl;
//        }*/

//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings& tPackets) ";
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index++, robotId);

//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings& tPackets) ";
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index++, un_bearing);
//    }
//#endif

//    /*if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
//      m_pcRABA->SetData(databyte_index, END_BUFFER);*/

//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET_FOOTER);


//    m_pcRABA->SetData(databyte_index++, RELAY_FVS_PACKET);
//    for (t_listMapFVsToRobotIds::iterator it = IdToFVsMap_torelay.begin(); it != IdToFVsMap_torelay.end(); ++it)
//    {
//        m_pcRABA->SetData(databyte_index++, it->uRobotId);
//        m_pcRABA->SetData(databyte_index++, it->uFV);
//    }
//    m_pcRABA->SetData(databyte_index, RELAY_FVS_PACKET_FOOTER);
//}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay)
//{
//    size_t databyte_index;

//    if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//        databyte_index = 0;


//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET);
//    m_pcRABA->SetData(databyte_index++, SelfId);
//    m_pcRABA->SetData(databyte_index++, SelfFV);
//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET_FOOTER);


//    m_pcRABA->SetData(databyte_index++, RELAY_FVS_PACKET);
//    bool buffer_full(false);
//    for(t_listMapFVsToRobotIds::iterator itd = IdToFVsMap_torelay.begin(); itd != IdToFVsMap_torelay.end(); ++itd)
//    {
//        m_pcRABA->SetData(databyte_index++, itd->uRobotId);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            /*buffer_full = true;
//            m_pcRABA->SetData(databyte_index, END_BUFFER);
//            break;*/
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay) " << std::endl;
//            exit(-1);
//        }

//        m_pcRABA->SetData(databyte_index++, itd->uFV);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            /*buffer_full = true;
//            m_pcRABA->SetData(databyte_index, END_BUFFER);
//            break;*/
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay) " << std::endl;
//            exit(-1);
//        }
//    }

//    /*if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
//      m_pcRABA->SetData(databyte_index, END_BUFFER);*/

//    m_pcRABA->SetData(databyte_index, RELAY_FVS_PACKET_FOOTER);
//}

/****************************************/
/****************************************/

//void CEPuckHomSwarm::WriteToCommunicationChannel(unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds,
//                                                 t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid)
//{
//    //printf("WriteToCommunicationChannel \n\n");

//    size_t databyte_index;

//    // we now put all the different message types in the same packet - to be sent at the same cycle
//    /*if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//    databyte_index = 0;*/

//    bool end_buffer_found(false);
//    for (size_t tmp_index = 0; tmp_index < m_pcRABA->GetSize(); ++tmp_index)
//    {
//        if (m_pcRABA->GetData(tmp_index) == RELAY_FVS_PACKET_FOOTER)
//        {
//            end_buffer_found = true;
//            databyte_index = tmp_index + 1;
//            break;
//        }
//    }

//    if (end_buffer_found == false)
//    {
//        std::cerr << " RELAY_FVS_PACKET_FOOTER not found  " << " WriteToCommunicationChannel(unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid) " << std::endl;
//        exit(-1);
//    }


//    if(databyte_index == (m_pcRABA->GetSize()-1))
//    {
//        std::cerr << " buffer full. no place to write voter packet header type " << std::endl;
//        exit(-1);
//    }
//    m_pcRABA->SetData(databyte_index++, VOTER_PACKET);
//    if(databyte_index == (m_pcRABA->GetSize()-1))
//    {
//        std::cerr << " buffer full. no place to write voter id " << std::endl;
//        exit(-1);
//    }
//    m_pcRABA->SetData(databyte_index++, VoterId);


//    if(CRMResultsOnFVDist.size() == 0 && ConsensusLst.size() == 0) // nothing to be written
//    {
//        if(databyte_index == (m_pcRABA->GetSize()))
//        {
//            std::cerr << " buffer full. no place to write end buffer " << std::endl;
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//        return;
//    }

//    bool buffer_full(false);
//    /*
//     * Write the consensus list to the channel comprising < .... <robot id, its consensus state> ... >
//     */
//    for (t_listConsensusInfoOnRobotIds::iterator it_cons = ConsensusLst.begin(); it_cons != ConsensusLst.end(); ++it_cons)
//    {
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//        m_pcRABA->SetData(databyte_index++, it_cons->uRobotId);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//        m_pcRABA->SetData(databyte_index++, (it_cons->consensus_state==1)?ATTACK_CONSENSUS:TOLERATE_CONSENSUS);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }
//    }

//    if(buffer_full)
//    {
//        std::cerr << " Written consensus. But buffer full now. No longer able to write the vote packet. complain by exiting" << std::endl;
//        std::cerr << " CRMResultsOnFVDist.size() " << CRMResultsOnFVDist.size() << " ConsensusLst.size() " << ConsensusLst.size() << std::endl;
//        exit(-1);
//    }

//    if(!b_CRM_Results_Valid) /* the crm results on the FVs in CRMResultsOnFVDist is no longer valid */
//    {
//        if(databyte_index == (m_pcRABA->GetSize()))
//        {
//            std::cerr << " buffer full. no place to write end buffer " << std::endl;
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//        return;
//    }

//    /*
//     * Write the results of CRM to the channel comprising < .... <fv, attack/tolerate state> ... >
//     * We dont write the results of all the FVs in the listFVsSensed as they may be some very old FVs no longer present in the swarm.
//     * Update: Your FV-ID map may be old too. Other robots may have a better map. Don't curtail information from them.
//     */
//    buffer_full = false;
//    for (t_listFVsSensed::iterator it_fvdist = CRMResultsOnFVDist.begin(); it_fvdist != CRMResultsOnFVDist.end(); ++it_fvdist)
//    {
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//#ifndef VOTESONROBOTID
//        m_pcRABA->SetData(databyte_index++, it_fvdist->uFV);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//        m_pcRABA->SetData(databyte_index++, (it_fvdist->uMostWantedState==1)?ATTACK_VOTE:TOLERATE_VOTE);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }
//#else
//        for (t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
//        {
//            if(it_map->uFV ==  it_fvdist->uFV)
//            {
//                m_pcRABA->SetData(databyte_index++, it_map->uRobotId);
//                if(databyte_index == m_pcRABA->GetSize()-1)
//                {
//                    buffer_full = true;
//                    m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//                    break;
//                }

//                m_pcRABA->SetData(databyte_index++, (it_fvdist->uMostWantedState==1)?ATTACK_VOTE:TOLERATE_VOTE);
//                if(databyte_index == m_pcRABA->GetSize()-1)
//                {
//                    buffer_full = true;
//                    m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//                    break;
//                }
//            }
//        }

//        if(buffer_full)
//            break;
//#endif
//    }

//    if(buffer_full)
//    {
//        std::cerr << " No longer able to write the vote packet. complain by exiting" << std::endl;
//        std::cerr << " CRMResultsOnFVDist.size() or VotedOnRobotIDs " << CRMResultsOnFVDist.size() << " ConsensusLst.size() " << ConsensusLst.size() << std::endl;
//        exit(-1);
//    }

//    if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
//        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//}

///****************************************/
///****************************************/

//bool func_SortPacketsOnRange (const CCI_RangeAndBearingSensor::SPacket i, const CCI_RangeAndBearingSensor::SPacket j) { return (i.Range < j.Range); }

///****************************************/

//bool  CEPuckHomSwarm::ReadFromCommunicationChannel_IdFv(const CCI_RangeAndBearingSensor::TReadings& tPackets)
//{
//    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
//    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
//    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */

//    bool read_successful(false); // successfully read id and fvs from at least one neighbour

//    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations

//    // Adding the most recent observations into listMapFVsToRobotIds and marked for relay in listMapFVsToRobotIds_relay
//    listMapFVsToRobotIds_relay.clear();
//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0; unsigned robotId, fv;

//        if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
//            byte_index = 1;
//        else
//            byte_index = 0;


//        if(tPackets[i].Data[byte_index++] == SELF_INFO_PACKET) // this neighbour is not sending me its FVs
//        {
//            robotId = tPackets[i].Data[byte_index++];
//            fv      = tPackets[i].Data[byte_index++];
//            read_successful = true;
//            byte_index++; // SELF_INFO_PACKET_FOOTER

//            listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));
//            UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
//        }


//        if(tPackets[i].Data[byte_index++] == RELAY_FVS_PACKET) // this neighbour is not sending me its FVs
//            for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
//            {
//                if(tPackets[i].Data[byteindex1] == RELAY_FVS_PACKET_FOOTER)
//                    break;

//                robotId = tPackets[i].Data[byteindex1];

//                if(tPackets[i].Data[byteindex1+1] == RELAY_FVS_PACKET_FOOTER)
//                    break;

//                fv      = tPackets[i].Data[byteindex1+1];

//                UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1); // old information
//            }

//    }

//    return read_successful;
//}

///****************************************/
///****************************************/

//bool  CEPuckHomSwarm::ReadFromCommunicationChannel_RelayedFv(const CCI_RangeAndBearingSensor::TReadings& tPackets)
//{
//    /* Only read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */

//    bool read_successful(false); // successfully read id and fvs from at least one neighbour

//    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations


//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0;  unsigned robotId, fv, observerId = 999u;

//        bool SELF_INFO_PACKET_FOUND(false);
//        for(byte_index = 0; byte_index < tPackets[i].Data.Size(); ++byte_index)
//        {
//            if(tPackets[i].Data[byte_index] == SELF_INFO_PACKET)
//            {
//                SELF_INFO_PACKET_FOUND = true;
//                byte_index++;
//                break;
//            }
//        }

//        if(SELF_INFO_PACKET_FOUND == true)
//            observerId = tPackets[i].Data[byte_index];


//        bool RELAY_FVS_PACKET_FOUND(false);
//        for(byte_index = 0; byte_index < tPackets[i].Data.Size(); ++byte_index)
//        {
//            if(tPackets[i].Data[byte_index] == RELAY_FVS_PACKET)
//            {
//                RELAY_FVS_PACKET_FOUND = true;
//                byte_index++;
//                break;
//            }
//        }

//        if(RELAY_FVS_PACKET_FOUND == false)
//            continue;


//        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
//        {
//            if (tPackets[i].Data[byteindex1] == RELAY_FVS_PACKET_FOOTER)
//                break;

//            robotId = tPackets[i].Data[byteindex1];

//            if (tPackets[i].Data[byteindex1+1] == RELAY_FVS_PACKET_FOOTER)
//                break;

//            fv      = tPackets[i].Data[byteindex1+1];


//            /*if(m_uRobotId == robotId)
//            {
//                std::cerr << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
//            }*/

//            if(robotId == 14)
//            {
//                    //std::cerr << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
//            }
//            else
//            {
//                    //std::cout << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
//            }


//            read_successful = true;


//#ifndef ConsensusOnMapOfIDtoFV
//            UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1); // old information
//#else
//            if(observerId == 999u)
//            {
//                printf("\n observerId was not in packet");
//                exit(-1);
//            }
//            UpdateFvToRobotIdMap(listMapFVsToRobotIds, observerId, fv, robotId, m_fInternalRobotTimer-1);
//#endif
//        }
//    }

//    return read_successful;
//}

///****************************************/
///****************************************/

//bool  CEPuckHomSwarm::ReadFromCommunicationChannel_VotCon(const CCI_RangeAndBearingSensor::TReadings& tPackets)
//{

//    /* Listen to votes and consensus from neighbours */
//    /* Read the voter id:
//     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
//     * If a vote is received,
//     *                      1. map the fv to the robot id (if none existed - ignore vote???)
//     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
//     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
//     *
//     * If a consensus is received,
//     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
//     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
//     *
//    */

//    bool read_successful(false); // successfully read votes / consensus from at least one neighbour


//    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations


//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0;
//        unsigned votertId, fv_or_id, attack_tolerate_vote, ConsensusOnRobotId, ConsensusState; unsigned tmp1, tmp2;

//        /*if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
//            byte_index = 1;
//        else
//        byte_index = 0;*/

//        bool voter_packet_found(false);
//        for (size_t tmp_index = 0; tmp_index < tPackets[i].Data.Size(); ++tmp_index)
//        {
//            if (tPackets[i].Data[tmp_index] == VOTER_PACKET)
//            {
//                voter_packet_found = true;
//                byte_index = tmp_index + 1;
//                break;
//            }
//        }

//        if(voter_packet_found == false) // this neighbour is not sending me any votes or consensus
//            continue;


//        votertId = tPackets[i].Data[byte_index++];

//        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
//        {
//            //printf("\npacket index %d; byteindex1=%d \n",i,byteindex1);

//            if(tPackets[i].Data[byteindex1] == VOTER_PACKET_FOOTER)
//                break;

//            tmp1 = tPackets[i].Data[byteindex1];

//            if(tPackets[i].Data[byteindex1+1] == VOTER_PACKET_FOOTER)
//                break;

//            tmp2 = tPackets[i].Data[byteindex1+1];


//            if(tmp2 == ATTACK_VOTE || tmp2 == TOLERATE_VOTE)
//            {
//                fv_or_id             = tmp1;
//                attack_tolerate_vote = (tmp2==ATTACK_VOTE)?1u:2u;

//#ifndef VOTESONROBOTID
//                UpdateVoterRegistry(listVoteInformationRobots,
//                                    listMapFVsToRobotIds,
//                                    listConsensusInfoOnRobotIds,
//                                    votertId, fv_or_id, attack_tolerate_vote);
//#else
//                UpdateVoterRegistry(listVoteInformationRobots,
//                                    listConsensusInfoOnRobotIds,
//                                    votertId, fv_or_id, attack_tolerate_vote);
//#endif

//            }
//            else
//            {
//                assert(tmp2 == ATTACK_CONSENSUS || tmp2 == TOLERATE_CONSENSUS);
//                ConsensusOnRobotId = tmp1;
//                ConsensusState     = (tmp2==ATTACK_CONSENSUS)?1u:2u;

//                bool b_ConsensusAlreadyEstablishedOnRobot(false);
//                for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
//                {
//                    if(it_cons->uRobotId == ConsensusOnRobotId)
//                    {
//                        b_ConsensusAlreadyEstablishedOnRobot = true;
//                        if (ConsensusState != it_cons->consensus_state)
//                        {
//                            // Difference in consensus state. there is a disparity in our consensus ????
//                            // We can correct state, assuming the lowest id (between my id and voter id) is correct?
//                        }
//                        break; // robot ids are unique in the consensus list
//                    }
//                }

//                if(!b_ConsensusAlreadyEstablishedOnRobot)
//                    listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(ConsensusOnRobotId, ConsensusState));
//            }

//            read_successful = true;
//        }

//    }

//    return read_successful;
//}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckHomSwarm, "epuck_homswarm_controller")
