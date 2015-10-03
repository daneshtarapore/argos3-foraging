#ifndef FEATUREVECTORSINROBOTAGENT_H_
#define FEATUREVECTORSINROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/

#include <list>
#include <algorithm>

/******************************************************************************/
/******************************************************************************/

#include <argos3/core/utility/math/vector2.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/******************************************************************************/
/******************************************************************************/

//#define ConsensusOnMapOfIDtoFV
//#define ConsensusOnMapOfIDtoFV_Threshold 0.75 // threshold which determines the strength of the consensus. a behavior feature value requires more than 75% votes for that value for consensus to be established.


//#define FILTER_BEFORE_VOTE // 90% decision making 10% voting and consensus

#define VOTESONROBOTID // not fv

/******************************************************************************/
/******************************************************************************/

using namespace argos;


/*
 * Stores the distribution of observed Feature vectors - both recent and past (determined by forget probability)
 */
struct StructFVsSensed
{
    unsigned int uFV;
    double fRobots;
    unsigned int uMostWantedState; // 0:  Dont know - no T-cells to make decision or E approx. equal to R (should not occur as T-cells are seeded for APCs where affinity=1)
                                   // 1:  Attack
                                   // 2:  Tolerate

    // proportion of the past time-steps when the FV would have been deemed as abnormal
    // double fSuspicious; //we are now going to use a history of previously sensed feature vectors

    StructFVsSensed(unsigned int fv, double density)
    {
        uFV     = fv;
        fRobots = density;
        uMostWantedState = 999;
    }

    StructFVsSensed(unsigned int fv, double density, unsigned int state)
    {
        uFV     = fv;
        fRobots = density;
        uMostWantedState = state;
    }
};

/*
 * Stores the map of observed FVs to robot ids (one to many function map)
 */
struct DetailedInformationFVsSensed
{
    unsigned int uRobotId;

    unsigned int uFV;
    double       fTimeSensed;

    double       f_TimesAttacked, f_TimesTolerated; //! we count the number of times the fv uFV is attacked / tolerated according to the CRM. During the last X time-steps, we only send out the vote near the end of the vote compilation period.

    DetailedInformationFVsSensed(unsigned int robotid, double timesensed, unsigned int fv)
    {
        uFV         = fv;
        uRobotId    = robotid;
        fTimeSensed = timesensed;

        f_TimesAttacked = 0.0f; f_TimesTolerated = 0.0f;
    }


    /*Additional data structure for establishing consensus on id-fv map entries*/
    std::vector<unsigned> vec_ObserverRobotIds;
    std::vector<unsigned> vec_ObservedRobotFVs;
    std::vector<Real>     vec_TimeObserved;

    DetailedInformationFVsSensed(unsigned int ObserverRobotId, unsigned int ObservedRobotId, double timesensed, unsigned int fv)
    {
        vec_ObserverRobotIds.clear(); vec_ObservedRobotFVs.clear(); vec_TimeObserved.clear();

        uRobotId = ObservedRobotId;
        vec_ObserverRobotIds.push_back(ObserverRobotId);
        vec_TimeObserved.push_back(timesensed);
        vec_ObservedRobotFVs.push_back(fv);
    }

    void AddNewInformationFVsSensed(unsigned int ObserverRobotId, double timesensed, unsigned int fv)
    {
        vec_ObserverRobotIds.push_back(ObserverRobotId);
        vec_TimeObserved.push_back(timesensed);
        vec_ObservedRobotFVs.push_back(fv);
    }

#ifdef ConsensusOnMapOfIDtoFV
    void SelectBestFVFromAllObservedFVs(unsigned u_NumFeatures, CRandom::CRNG* m_pcRNG_FVs, unsigned uSelfRobotId)
    {
        std::vector <Real> vec_countfeatures_one(u_NumFeatures, 0);
        std::vector <Real> vec_countfeatures_zero(u_NumFeatures, 0);
        std::vector <Real> vec_countfeatures_maxvotes(u_NumFeatures, 0);


        for (size_t fv_index = 0; fv_index < vec_ObservedRobotFVs.size(); ++fv_index)
            for(size_t feature_index = 0; feature_index < u_NumFeatures; ++feature_index)
            {
                ((vec_ObservedRobotFVs[fv_index] >> feature_index) & 0x1) ? vec_countfeatures_one[feature_index] += 1.0f : vec_countfeatures_zero[feature_index] += 1.0f;
                vec_countfeatures_maxvotes[feature_index] = std::max(vec_countfeatures_one[feature_index], vec_countfeatures_zero[feature_index]) /
                                                                    (vec_countfeatures_one[feature_index] + vec_countfeatures_zero[feature_index]);
            }

        bool strengthofconsensus(true);
        for(size_t feature_index = 0; feature_index < u_NumFeatures; ++feature_index)
            if(vec_countfeatures_maxvotes[feature_index] < ConsensusOnMapOfIDtoFV_Threshold)
                strengthofconsensus = false;


        if(strengthofconsensus)
        {
            uFV = 0u;
            for(size_t feature_index = 0; feature_index < u_NumFeatures; ++feature_index)
            {
                if((vec_countfeatures_one[feature_index] / (vec_countfeatures_one[feature_index] + vec_countfeatures_zero[feature_index]))  >= 0.5)
                    uFV += (1 << feature_index);
            }
        }
        else
        {
            bool selfobservationmade(false);
            for (size_t fv_index = 0; fv_index < vec_ObservedRobotFVs.size(); ++fv_index)
            {
                 if(vec_ObserverRobotIds[fv_index] == uSelfRobotId)
                 {
                     uFV = vec_ObservedRobotFVs[fv_index];
                     selfobservationmade = true;
                     break;
                 }
            }

            if(selfobservationmade == false)
                uFV = 999u;
        }
    }
#endif
};


struct ConsensusInformationRobots
{
    unsigned uRobotId;
    unsigned consensus_state; //attack or tolerate

    ConsensusInformationRobots(unsigned int id, unsigned state)
    {
        uRobotId        = id;
        consensus_state = state;
    }
};

struct VoteInformationRobots
{
    unsigned uRobotId;

    std::list <unsigned> uVoterIds;
    unsigned attackvote_count;
    unsigned toleratevote_count;
    Real fTimeConsensusReached;


    VoteInformationRobots(unsigned Id, unsigned voterId, unsigned attack_tolerate_vote)
    {
        uRobotId        = Id;

        uVoterIds.clear();
        uVoterIds.push_back(voterId);

        fTimeConsensusReached = 100000.0;


        attackvote_count = 0; toleratevote_count = 0;
        if (attack_tolerate_vote == 1)
            attackvote_count++;
        else
            toleratevote_count++;
    }
};



typedef std::list<StructFVsSensed>              t_listFVsSensed;
typedef std::list<DetailedInformationFVsSensed> t_listMapFVsToRobotIds;
typedef std::list<ConsensusInformationRobots>   t_listConsensusInfoOnRobotIds;
typedef std::list<VoteInformationRobots>        t_listVoteInformationRobots;

/******************************************************************************/
/******************************************************************************/

/*
 *
 *
 */
void UpdaterFvDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                           CRandom::CRNG *m_pcRNG, Real m_fProbForget);


/*
 *
 *
 */
void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                     unsigned int fv, unsigned robotId, double timesensed);


/*
 *
 *
 */
void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                     unsigned int ObserverRobotId, unsigned int fv, unsigned ObservedRobotId, double timesensed);



/*
 *
 *
 */
void IntegrateAttackTolerateDecisions(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                                      unsigned fv,
                                      unsigned attack_tolerate_prevote);


/*
 * voting on decisions communicated to you directly on robot ids.
 *
 */
void UpdateVoterRegistry(t_listVoteInformationRobots   &listVoteInformationRobots,
                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds,
                         unsigned voter_id,
                         unsigned votedon_id,
                         unsigned attack_tolerate_vote);

/*
 * voting on decisions communicated to you on fvs, that you map to robot ids and then vote
 *
 */
void UpdateVoterRegistry(t_listVoteInformationRobots   &listVoteInformationRobots,
                         t_listMapFVsToRobotIds   &listMapFVsToRobotIds,
                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds,
                         unsigned voter_id, unsigned fv,
                         unsigned attack_tolerate_vote, bool filterownersvote = false);


/*
 * Removes old maps from FVs to Robot Ids
 * f_IdToFV_MaintenanceTime: how long we maintain a particular mapping from fv to robot id
   when do we delete old map entries ? if an entry is older than f_FvToId_MaintenanceTime, we delete it.
 */
void TrimFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed, Real f_CurrentRobotTime, Real f_FvToId_MaintenanceTime);


/*
 *
 */
void PrintFvToRobotIdMap(unsigned u_MapOfRobotId, t_listMapFVsToRobotIds &listDetailedInformationFVsSensed, unsigned u_ObservedRobotId=99999u);


/*
 *
 *
 */
void PrintVoterRegistry(unsigned u_VoterRegistryOfRobotId, t_listVoteInformationRobots &listVoteInformationRobots, unsigned u_VotedOnRobotId=99999u);


/*
 *
 */
void PrintConsensusRegistry(unsigned u_ConsensusRegistryOfRobotId, t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds, unsigned u_ConsensusOnRobotId=99999u);


/*
 * Select the best feature value for each feature from all the observered feature-vectors for each map entry.
 */
void SelectBestFVFromAllObservedFVs(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed, unsigned u_NumFeatures, CRandom::CRNG* m_pcRNG_FVs, unsigned uSelfRobotId);

/******************************************************************************/
/******************************************************************************/

#endif
