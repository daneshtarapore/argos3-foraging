#!/bin/bash -e

data="SimulationsOfPhysicalSwarmExpts"
maxnumjobs=7

numreplicates=5

epuck_homswarm_lib="\/home\/danesh\/argos3-epuck\/argos3-foraging\/build\/controllers\/epuck_hom_swarm\/libepuck_hom_swarm_simofphyswarm.so"
epuck_homswarmloopfunc_lib="\/home\/danesh\/argos3-epuck\/argos3-foraging\/build\/loop_functions\/homswarm_loop_functions\/libhomswarm_loop_functions_simofphyswarm.so"


# Create a data diretory
mkdir -p $data

#for NormalBehav in SWARM_AGGREGATION SWARM_DISPERSION SWARM_HOMING_MOVING_BEACON
#do
#    for ErrorBehav in FAULT_PROXIMITYSENSORS_SETMIN FAULT_PROXIMITYSENSORS_SETMAX FAULT_RABSENSOR_SETOFFSET FAULT_ACTUATOR_LWHEEL_SETZERO FAULT_ACTUATOR_RWHEEL_SETZERO FAULT_ACTUATOR_BWHEELS_SETZERO FAULT_NONE
#    do

#    mkdir -p $data/${NormalBehav}/${ErrorBehav}

#    done
#done



for NormalBehav in SWARM_AGGREGATION SWARM_DISPERSION SWARM_HOMING_MOVING_BEACON
do		   
    for ErrorBehav in FAULT_PROXIMITYSENSORS_SETMIN FAULT_PROXIMITYSENSORS_SETMAX FAULT_RABSENSOR_SETOFFSET FAULT_ACTUATOR_LWHEEL_SETZERO FAULT_ACTUATOR_RWHEEL_SETZERO FAULT_ACTUATOR_BWHEELS_SETZERO FAULT_NONE;
    do
        for Replicates in $(seq 1 $numreplicates); do
            # Take template.argos and make an .argos file for this experiment
            SUFFIX=${Replicates}${Replicates}${Replicates}
            sed -e "s/SEED/${Replicates}${Replicates}${Replicates}/"                    \
                -e "s/EPUCKHOMSWARMLIB/${epuck_homswarm_lib}/"                  \
                -e "s/SWARM_BEHAVIOR/${NormalBehav}/"                   \
                -e "s/FAULT_BEHAVIOR/${ErrorBehav}/"                  \
                -e "s/EPUCKHOMSWARMLOOPFUNCLIB/${epuck_homswarmloopfunc_lib}/"                  \
                -e "s|DATAFILE|$data/${NormalBehav}_${ErrorBehav}_${SUFFIX}.fvlog_PropFV|" \
                template_epuck_homswarm_noise.argos                       \
                > $data/${NormalBehav}_${ErrorBehav}_${SUFFIX}.argos
            # Call ARGoS
            parallel --no-notice --semaphore -j${maxnumjobs} argos3 -c $data/${NormalBehav}_${ErrorBehav}_${SUFFIX}.argos &
        done
    done
done

sem --wait
