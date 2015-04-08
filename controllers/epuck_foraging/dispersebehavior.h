#ifndef DISPERSEBEHAVIOR_H_
#define DISPERSEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CDisperseBehavior : public CBehavior 
{
public:
    CDisperseBehavior(Real m_fProximitySensorThreshold, CRange<CDegrees>m_cGoStraightAngleRangeDegrees);

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

protected:
    double           m_fProximitySensorThreshold;
    CRange<CDegrees> m_cGoStraightAngleRangeDegrees;
    CVector2         m_cDiffusionVector;

};


/******************************************************************************/
/******************************************************************************/

#endif 
