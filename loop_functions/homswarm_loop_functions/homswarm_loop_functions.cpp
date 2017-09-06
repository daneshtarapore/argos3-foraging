#include "homswarm_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <controllers/epuck_hom_swarm/epuck_hom_swarm.h>



/****************************************/
/****************************************/

CHomSwarmLoopFunctions::CHomSwarmLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(CRandom::CreateRNG("argos"))
{
//    // keeping track of distance travelled by bot in last 100 time-steps
//    m_iDistTravelledTimeWindow = 100;
//    m_unCoordCurrQueueIndex    = 0;
//    m_pvecCoordAtTimeStep = new CVector2[m_iDistTravelledTimeWindow];
//    CurrentStepNumber = 0;
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Init(TConfigurationNode& t_node)
{
   try
    {
      TConfigurationNode& tParams = GetNode(t_node, "params");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();

      /* Get the output file name from XML */
      GetNodeAttribute(tParams, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      //m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;

       GetNodeAttribute(tParams, "arenalength", fArenaLength);

   }
   catch(CARGoSException& ex)
      THROW_ARGOSEXCEPTION_NESTED("Error parsing homswarm loop function.", ex);


    // counting the number of robots in the swarm
    u_num_epucks = 0u;
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        u_num_epucks++;
    }


    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());
        cController.GetExperimentType().SetNumEPuckRobotsInSwarm(u_num_epucks);

        /*if (cController.GetExperimentType().swarmbehav.compare("SWARM_HOMING") == 0)
        {
            if (cController.RobotIdStrToInt() == 0)
            {
                CVector3 beaconposition; CQuaternion beaconorientation;

                // Position the beacon robot to be not close to the wall
                CRange <Real>m_cForagingArenaSideX = CRange<Real>(-fArenaLength/4.0 - 0.3f, fArenaLength/4.0  - 0.3f);

                beaconposition.SetX(m_pcRNG->Uniform(m_cForagingArenaSideX));
                beaconposition.SetY(m_pcRNG->Uniform(m_cForagingArenaSideX));

                cEPuck.GetEmbodiedEntity().MoveTo(beaconposition, beaconorientation);
            }
        }*/
    }
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Reset()
{  /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   //m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::PreStep()
{
    CVector2 cPos1, cPos2;
    CSpace::TMapPerType& m_cEpucks1 = GetSpace().GetEntitiesByType("e-puck");
    CSpace::TMapPerType& m_cEpucks2 = GetSpace().GetEntitiesByType("e-puck");

    for(CSpace::TMapPerType::iterator it = m_cEpucks1.begin();
        it != m_cEpucks1.end();
        ++it)
    {
        /* Get handle to foot-bot entity and controller */
        CEPuckEntity& cEPuck1 = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController1 = dynamic_cast<CEPuckHomSwarm&>(cEPuck1.GetControllableEntity().GetController());

        unsigned rob_id = cController1.RobotIdStrToInt();

        cPos1.Set(cEPuck1.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                  cEPuck1.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        if(rob_id == 15){


            for(CSpace::TMapPerType::iterator it2 = m_cEpucks2.begin();
                it2 != m_cEpucks2.end();
                ++it2)
            {
                CEPuckEntity& cEPuck2 = *any_cast<CEPuckEntity*>(it2->second);
                CEPuckHomSwarm& cController2 = dynamic_cast<CEPuckHomSwarm&>(cEPuck2.GetControllableEntity().GetController());

                unsigned rob_id2 = cController2.RobotIdStrToInt();

                cPos2.Set(cEPuck2.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                          cEPuck2.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

                float dist = (cPos1- cPos2).Length();
                std::cerr << "Robot id: " << rob_id2 << " dist " << dist << std::endl;
            }
        }
    }

//    CVector2 cPos;
//    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");

//    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin();
//        it != m_cEpucks.end();
//        ++it) {
//       /* Get handle to foot-bot entity and controller */
//       CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);

//       cPos.Set(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
//                cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

//       std::cerr << " Pos X " << cPos.GetX() << " Y " << cPos.GetY() << std::endl;
//    }

//    CVector2 vecAgentPos = cPos;

//    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
//    {
//        // distance travelled in last 100 time-steps
//        Real m_fSquaredDistTravelled = (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX()) *
//                                  (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX())  +

//                                  (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY()) *
//                                  (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY());


//        // decision based on distance travelled in the last 100 time-steps
//        std::cout << "True distance travelled for step " << CurrentStepNumber << " is " << sqrt(m_fSquaredDistTravelled)*100.0f << std::endl;
//    }

//    // adding new coordinate values into the queue
//    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
//    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;
//    CurrentStepNumber++;


    /*CSpace::TMapPerType& m_cLights = GetSpace().GetEntitiesByType("light");
    for(CSpace::TMapPerType::iterator it = m_cLights.begin(); it != m_cLights.end(); ++it)
    {
        CLightEntity& cLight = *any_cast<CLightEntity*>(it->second);

        if((unsigned)(GetSpace().GetSimulationClock())%100 == 0)
            cLight.MoveTo(CVector3(), CQuaternion());

        cLight.GetInitPosition();
    }*/


    float start_firsttrans_sec = 150.0f, finish_firsttosecondtrans_sec = 1000.0f;


    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

        if(GetSpace().GetSimulationClock() < start_firsttrans_sec * cController.m_sRobotDetails.iterations_per_second)
        { }

        /*transition robot ids from robot_ids_behav1 to robot_ids_behav2 */
        else if((GetSpace().GetSimulationClock() >= start_firsttrans_sec  * cController.m_sRobotDetails.iterations_per_second) &&
                (GetSpace().GetSimulationClock() < finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second))
        {
            Real time_between_robots_trans_behav = cController.GetExperimentType().time_between_robots_trans_behav * cController.m_sRobotDetails.iterations_per_second;

            Real m_fInternalRobotTimer = GetSpace().GetSimulationClock() - start_firsttrans_sec  * cController.m_sRobotDetails.iterations_per_second;

            if(cController.GetExperimentType().robot_ids_behav1.size() > 0u)
            {
                if(time_between_robots_trans_behav > 0.0f)
                {
                    if(((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0)
                    {
                        cController.GetExperimentType().robot_ids_behav2.push_back(cController.GetExperimentType().robot_ids_behav1.front());
                        cController.GetExperimentType().robot_ids_behav1.pop_front();
                    }
                }
                else if(time_between_robots_trans_behav == 0.0f)
                {
                    while(cController.GetExperimentType().robot_ids_behav1.size() > 0u)
                    {
                        cController.GetExperimentType().robot_ids_behav2.push_back(cController.GetExperimentType().robot_ids_behav1.front());
                        cController.GetExperimentType().robot_ids_behav1.pop_front();
                    }
                }
            }
        }

        /* transition robot ids from robot_ids_behav2 back to robot_ids_behav1 */
        else if (GetSpace().GetSimulationClock() >= finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second)
        {
            Real time_between_robots_trans_behav = cController.GetExperimentType().time_between_robots_trans_behav * cController.m_sRobotDetails.iterations_per_second;

            Real m_fInternalRobotTimer = GetSpace().GetSimulationClock() - finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second;

//            if((((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0) && (cController.GetExperimentType().robot_ids_behav2.size() > 0u))
//            {
//                cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
//                cController.GetExperimentType().robot_ids_behav2.pop_front();
//            }

            if(cController.GetExperimentType().robot_ids_behav2.size() > 0u)
            {
                if(time_between_robots_trans_behav > 0.0f)
                {
                    if(((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0)
                    {
                        cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
                        cController.GetExperimentType().robot_ids_behav2.pop_front();
                    }
                }
                else if(time_between_robots_trans_behav == 0.0f)
                {
                    while(cController.GetExperimentType().robot_ids_behav2.size() > 0u)
                    {
                        cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
                        cController.GetExperimentType().robot_ids_behav2.pop_front();
                    }
                }
            }

        }
    }
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::PostStep()
{
    std::map<size_t, size_t> RobotIndex_ARGoSID_Map;
    RobotIndex_ARGoSID_Map[0] = 201u;
    RobotIndex_ARGoSID_Map[1] = 202u;
    RobotIndex_ARGoSID_Map[2] = 206u;
    RobotIndex_ARGoSID_Map[3] = 207u;
    RobotIndex_ARGoSID_Map[4] = 208u;
    RobotIndex_ARGoSID_Map[5] = 209u;
    RobotIndex_ARGoSID_Map[6] = 210u;

    m_cOutput << GetSpace().GetSimulationClock() << "\t";

    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

        unsigned observed_rob_id = cController.RobotIdStrToInt();
        unsigned observed_rob_fv = cController.GetRobotFeatureVector();

        m_cOutput << RobotIndex_ARGoSID_Map[observed_rob_id] << " " << observed_rob_fv << "\t";
    }
    m_cOutput << std::endl;
    return;
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Destroy()
{
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CHomSwarmLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) // used to paint the floor by the floor entity
{
   return CColor::WHITE;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CHomSwarmLoopFunctions, "homswarm_loop_functions")
