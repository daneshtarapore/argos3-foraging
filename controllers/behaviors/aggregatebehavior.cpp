#include "aggregatebehavior.h"


/******************************************************************************/
/******************************************************************************/

CAggregateBehavior::CAggregateBehavior(Real m_fRangeAndBearing_RangeThreshold) :
    m_fRangeAndBearing_RangeThreshold(m_fRangeAndBearing_RangeThreshold)
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CAggregateBehavior::TakeControl()
{
    bool controltaken(false);

    // using range and bearing sensors since proximity sensors have a very small range for both dispersion and aggregation to take place (10cm ideally, but no more than 5 cm according to Lorenzo's epuck model).
    // also we need to aggregate to other robots - and not obstacles
    m_cAggregationVector.Set(0.0f, 0.0f);

    unsigned robotsinrange = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_sSensoryData.m_RABSensorData[i].Range < m_fRangeAndBearing_RangeThreshold)
        {
            // controltaken = true;
            m_cAggregationVector += CVector2(m_sSensoryData.m_RABSensorData[i].Range, m_sSensoryData.m_RABSensorData[i].HorizontalBearing);
            robotsinrange++;
        }
    }

    if(robotsinrange > 3u) // use > 3 instead of > 0 to avoid small aggregates of just 2 robots or just 3 robots.
        controltaken = true;

    /*if(robotsinrange > 0u)
        controltaken = true;*/

    if(controltaken)
    {
        m_cAggregationVector /= robotsinrange;
        //std::cout << "Aggregation behavior taking control " << std::endl;
    }

    return controltaken;



//    /* Get readings from proximity sensor */
//    /* Sum them together */
//    m_cAggregationVector.Set(0.0f, 0.0f);
//    for(size_t i = 0; i <  m_sSensoryData.m_ProximitySensorData.size(); ++i)
//    {
//        m_cAggregationVector += CVector2(m_sSensoryData.m_ProximitySensorData[i].Value, m_sSensoryData.m_ProximitySensorData[i].Angle);
//    }
//    m_cAggregationVector /= m_sSensoryData.m_ProximitySensorData.size();

//    /* If the closest obstacle is far enough, ignore the vector and go straight, otherwise return
//      it */
//    if(m_cAggregationVector.Length() < m_fProximitySensorThreshold)
//        return false;
//    else
//    {
//        //std::cout << " Disperse behavior taking control " << std::endl;
//        return true;
//    }
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CAggregateBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
     CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (1.0f * m_cAggregationVector.Normalize() +
                                                            1.0f * CVector2(1.0f, m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI))));

     // i add a strong random component to break stable aggregates of pair of robots.
     /*CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (0.3f * m_cAggregationVector.Normalize() +
                                                            0.7f * CVector2(1.0f, m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI))));*/

     WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/

void CAggregateBehavior::PrintBehaviorIdentity()
{
    std::cout << "Aggregation taking over";
}

/******************************************************************************/
/******************************************************************************/
