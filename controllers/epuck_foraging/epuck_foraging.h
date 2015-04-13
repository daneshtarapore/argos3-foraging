/*
 * AUTHOR: Danesh Tarapore <daneshtarapore@gmail.com>
 *
 * An example foraging controller for the foot-bot.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/foraging.argos
 */

//! TODO
//! 1. Convert the generic plugins to those used by the e-puck if implementation is different (see the detailed e-puck model of Lorenzo et al.).
//! 2. Introduce noise in the sensors and actuators of the e-puck (see parameters from Lorenzo)


#ifndef EPUCK_FORAGING_H
#define EPUCK_FORAGING_H

/*
 * Include some necessary headers.
 */

#include <iostream>
#include <vector>

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>

/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>

/* Definition of the light sensor */
// seems to be less accurate than the footbot light sensor. but since this sensor is just a placeholder for compass sensor o psi-swarm, we can leave it as it is
#include <argos3/plugins/robots/generic/control_interface/ci_light_updated_sensor.h>


/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>


/* Definitions for behaviors used to control robot */
#include "behavior.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "randomwalkbehavior.h"
#include "phototaxisbehavior.h"
#include "antiphototaxisbehavior.h"
#include "homingtofoodbeaconbehavior.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckForaging : public CCI_Controller
{

public:

    struct ExperimentToRun
    {
        /* The type of experiment to run */
        enum SwarmBehavior
        {
            SWARM_AGGREGATION = 0,
            SWARM_DISPERSION,
            SWARM_HOMING,
            SWARM_FORAGING
        } SBehavior;


        /* The possible faults on robot */
        enum FaultBehavior
        {
            FAULT_NONE = 0,
            /*faults whose effects cause one of the following four general failures */
            FAULT_STRAIGHTLINE,
            FAULT_RANDOMWALK,
            FAULT_CIRCLE,
            FAULT_STOP,

            /*specific faults to sensors */
            FAULT_PROXIMITYSENSOR_SETZERO,
            FAULT_GROUNDSENSOR_SETZERO,
            FAULT_LIGHTSENSOR_SETZERO,

            /*specific faults to communication module */
            FAULT_RABCOMMUNICATION_SETZERO,

            /*specific faults to actuators */
            FAULT_ACTUATOR_LWHEEL_SETZERO,
            FAULT_ACTUATOR_RWHEEL_SETZERO,

            /*fault in controller*/
            FAULT_CONTROLLER_COMPLETEFAILURE
        } FBehavior;


        std::string id_FaultyRobotInSwarm;

        ExperimentToRun();
        void Init(TConfigurationNode& t_node);
    };


    /*
    * This structure holds data about food collecting by the robots
    */
    struct SFoodData
    {
        bool HasFoodItem;      // true when the robot is carrying a food item
        size_t TotalFoodItems; // the total number of food items carried by this robot during the experiment

        SFoodData();
        void Reset();
    };


    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_foraging_controller><parameters><wheel_turning>
    * section.
    */
    struct SWheelTurningParams
    {
        //      /*
        //       * The turning mechanism.
        //       * The robot can be in three different turning states.
        //       */
        //      enum ETurningMechanism
        //      {
        //         NO_TURN = 0, // go straight
        //         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
        //         HARD_TURN    // wheels are turning with opposite speeds
        //      } TurningMechanism;
        //      /*
        //       * Angular thresholds to change turning state.
        //       */
        //      CRadians HardTurnOnAngleThreshold;
        //      CRadians SoftTurnOnAngleThreshold;
        //      CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode& t_tree);
    };

    /*
    * Contains all the state information about the controller.
    */
    struct SStateData
    {
        /* The three possible states in which the controller can be */
        enum EState
        {
            STATE_RESTING = 0,
            STATE_EXPLORING,
            STATE_RETURN_TO_NEST,
            STATE_RESTING_AT_FOOD,
            STATE_BEACON
        } State;

        /* True when the robot is in the nest */
        bool InNest;
        bool OnFood;

        /* Initial probability to switch from resting to exploring */
        Real InitialRestToExploreProb;
        /* Current probability to switch from resting to exploring */
        //Real RestToExploreProb;
        /* Initial probability to switch from exploring to resting */
        //Real InitialExploreToRestProb;
        /* Current probability to switch from exploring to resting */
        //Real ExploreToRestProb;
        /* Used as a range for uniform number generation */
        CRange<Real> ProbRange;
        /* The increase of ExploreToRestProb due to the food rule */
        //Real FoodRuleExploreToRestDeltaProb;
        /* The increase of RestToExploreProb due to the food rule */
        //Real FoodRuleRestToExploreDeltaProb;
        /* The increase of ExploreToRestProb due to the collision rule */
        //Real CollisionRuleExploreToRestDeltaProb;
        /* The increase of RestToExploreProb due to the social rule */
        //Real SocialRuleRestToExploreDeltaProb;
        /* The increase of ExploreToRestProb due to the social rule */
        //Real SocialRuleExploreToRestDeltaProb;
        /* The minimum number of steps in resting state before the robots
         starts thinking that it's time to move */
        size_t MinimumRestingTime;
        /* The number of steps in resting state */
        size_t TimeRested;
        /* The number of exploration steps without finding food after which
         a foot-bot starts thinking about going back to the nest */
        size_t MinimumUnsuccessfulExploreTime;
        /* The number of exploration steps without finding food */
        size_t TimeExploringUnsuccessfully;
        /* If the robots switched to resting as soon as it enters the nest,
         there would be overcrowding of robots in the border between the
         nest and the rest of the arena. To overcome this issue, the robot
         spends some time looking for a place in the nest before finally
         settling. The following variable contains the minimum time the
         robot must spend in state 'return to nest' looking for a place in
         the nest before switching to the resting state. */
        size_t MinimumSearchForPlaceInNestTime;
        /* The time spent searching for a place in the nest */
        size_t TimeSearchingForPlaceInNest;

        SStateData();
        void Init(TConfigurationNode& t_node);
        void Reset();
    };

public:

    /* Class constructor. */
    CEPuckForaging();
    /* Class destructor. */
    virtual ~CEPuckForaging() {}

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_foraging_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    virtual void RunForagingExperiment();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    /*
    * Returns true if the robot is currently exploring.
    */
    inline bool IsExploring() const {
        return m_sStateData.State == SStateData::STATE_EXPLORING;
    }

    /*
    * Returns true if the robot is currently resting.
    */
    inline bool IsResting() const {
        return m_sStateData.State == SStateData::STATE_RESTING;
    }

    /*
    * Returns true if the robot is currently returning to the nest.
    */
    inline bool IsReturningToNest() const {
        return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
    }

    /*
    * Returns the food data
    */
    inline SFoodData& GetFoodData()
    {
        return m_sFoodData;
    }

    /*
    * Returns the experiment type
    */
    inline ExperimentToRun& GetExperimentType()
    {
        return m_sExpRun;
    }

private:

    /*
    * Updates the state information.
    * In pratice, it sets the SStateData::InNest flag.
    * Future, more complex implementations should add their
    * state update code here.
    */
    void UpdateState();

    /*
    * Calculates the vector to the light. Used to perform
    * phototaxis and antiphototaxis.
    */
    //CVector2 CalculateVectorToLight();

    /*
    * Calculates the diffusion vector. If there is a close obstacle,
    * it points away from it; it there is none, it points forwards.
    * The b_collision parameter is used to return true or false whether
    * a collision avoidance just happened or not. It is necessary for the
    * collision rule.
    */
    //CVector2 DiffusionVector(bool& b_collision);

    /*
    * Gets a direction vector as input and transforms it into wheel
    * actuation.
    */
    //void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Executes the resting state at the nest.
    */
    void RestAtNest();

    /*
    * Executes the resting state at the food source - analogy: robot gathering data from site.
    */
    void RestAtFood();

    /*
    * Executes the become a beacon state at the food source.
    */
    void BecomeABeacon();

    /*
    * Executes the exploring state.
    */
    void Explore();

    /*
    * Executes the return to nest state.
    */
    void ReturnToNest();

private:

    TBehaviorVector             m_vecBehaviors;
    bool                        b_damagedrobot;     // true if robot is damaged

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator* m_pcLEDs;
    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;
    /* Pointer to the foot-bot proximity sensor */
    CCI_EPuckProximitySensor* m_pcProximity;
    /* Pointer to the foot-bot light sensor */
    CCI_LightUpdatedSensor* m_pcLight;
    /* Pointer to the foot-bot motor ground sensor */
    CCI_GroundSensor* m_pcGround;

    /* The random number generator */
    CRandom::CRNG* m_pcRNG;

    /* Used in the social rule to communicate the result of the last
    * exploration attempt */
    enum ELastExplorationResult
    {
        LAST_EXPLORATION_NONE = 0,    // nothing to report
        LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a food item found
        LAST_EXPLORATION_UNSUCCESSFUL, // no food found in the last exploration
        BEACON_ESTABLISHED // a beacon has been established at a food source
    } m_eLastExplorationResult;

    /*Information on the experiment to run - the swarm behavior and the faulty behavior*/
    ExperimentToRun m_sExpRun;
    /* The controller state information */
    SStateData m_sStateData;
    /* The turning parameters */
    SWheelTurningParams m_sWheelTurningParams;
    /* The food data */
    SFoodData m_sFoodData;


};

#endif
