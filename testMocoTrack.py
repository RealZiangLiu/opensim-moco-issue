# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoTrack.py                                          #
# -------------------------------------------------------------------------- #
# Copyright (c) 2023 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Nicholas Bianco                                                 #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License") you may     #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# This example features two different tracking problems solved using the
# MocoTrack tool. 
#  - The first problem demonstrates the basic usage of the tool interface
#    to solve a torque-driven marker tracking problem. 
#  - The second problem shows how to customize a muscle-driven state tracking 
#    problem using more advanced features of the tool interface.
# 
# See the README.txt next to this file for more information.

import os
import opensim as osim

import random 
import time

random.seed(time.time())


def muscleDrivenStateTracking():

    # Create and name an instance of the MocoTrack tool.
    track = osim.MocoTrack()
    track.setName("muscle_driven_state_tracking")

    # Construct a ModelProcessor and set it on the tool. The default
    # muscles in the model are replaced with optimization-friendly
    # DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    # parameters.

    modelProcessor = osim.ModelProcessor("model.osim")
    # modelProcessor.append(osim.ModOpAddExternalLoads("grf_walk.xml"))
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    # Use a function-based representation for the muscle paths. This is
    # recommended to speed up convergence, but if you would like to use
    # the original GeometryPath muscle wrapping instead, simply comment out
    # this line. To learn how to create a set of function-based paths for
    # your model, see the example 'examplePolynomialPathFitter.py'.
    # modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
    #         "subject_walk_scaled_FunctionBasedPathSet.xml"))
    track.setModel(modelProcessor)

    # Construct a TableProcessor of the coordinate data and pass it to the 
    # tracking tool. TableProcessors can be used in the same way as
    # ModelProcessors by appending TableOperators to modify the base table.
    # A TableProcessor with no operators, as we have here, simply returns the
    # base table.
    # track.setStatesReference(osim.TableProcessor("coordinates.sto"))
    marker_file = "motion_2_modified.trc"

    track.setMarkersReferenceFromTRC(marker_file)

    # Increase the global marker tracking weight, which is the weight
    # associated with the internal MocoMarkerTrackingCost term.
    track.set_markers_global_tracking_weight(10)

    # Increase the tracking weights for individual markers in the data set 
    # placed on bony landmarks compared to markers located on soft tissue.
    markerWeights = osim.MocoWeightSet()
    markerWeights.cloneAndAppend(osim.MocoWeight("CLAV", 20))
    markerWeights.cloneAndAppend(osim.MocoWeight("C7", 20))
    markerWeights.cloneAndAppend(osim.MocoWeight("RSHO", 10))
    markerWeights.cloneAndAppend(osim.MocoWeight("RUPA", 5))
    markerWeights.cloneAndAppend(osim.MocoWeight("RELB", 10))
    markerWeights.cloneAndAppend(osim.MocoWeight("RFRM", 2))
    markerWeights.cloneAndAppend(osim.MocoWeight("RWRA", 10))
    markerWeights.cloneAndAppend(osim.MocoWeight("RWRB", 10))
    markerWeights.cloneAndAppend(osim.MocoWeight("RFIN", 2))
    track.set_markers_weight_set(markerWeights)

    # This setting allows extra data columns contained in the states
    # reference that don't correspond to model coordinates.
    track.set_allow_unused_references(True)

    # Since there is only coordinate position data in the states references,
    # this setting is enabled to fill in the missing coordinate speed data using
    # the derivative of splined position data.
    track.set_track_reference_position_derivatives(True)

    # Initial time, final time, and mesh interval.
    track.set_initial_time(0.5)
    track.set_final_time(3.0)
    # motion_1: 0.5 - 3.0
    # motion 2: 0.7 - 2.5
    # motion 3: 0.5 - 3.5

    track.set_mesh_interval(0.05)

    # Instead of calling solve(), call initialize() to receive a pre-configured
    # MocoStudy object based on the settings above. Use this to customize the
    # problem beyond the MocoTrack interface.
    study = track.initialize()

    # Get a reference to the MocoControlCost that is added to every MocoTrack
    # problem by default.
    problem = study.updProblem()
    effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("control_effort"))
    effort.setWeight(1e-2)

    # Put a large weight on the pelvis CoordinateActuators, which act as the
    # residual, or 'hand-of-god', forces which we would like to keep as small
    # as possible.
    model = modelProcessor.process()
    model.initSystem()
    forceSet = model.getForceSet()
    # for i in range(forceSet.getSize()):
    #     forcePath = forceSet.get(i).getAbsolutePathString()
    #     if 'pelvis' in str(forcePath):
    #         effort.setWeightForControl(forcePath, 10)

    # Constrain the muscle activations at the initial time point to equal
    # the initial muscle excitation value.
    problem.addGoal(osim.MocoInitialActivationGoal('initial_activation'))

    # Update the solver tolerances.
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())

    # solver.set_num_mesh_intervals(50)
    # solver.set_verbosity(2)
    solver.set_optim_solver("ipopt")
    # solver.set_scale_variables_using_bounds(True)
    solver.set_optim_convergence_tolerance(1e-3)
    solver.set_optim_constraint_tolerance(1e-4)
    solver.set_optim_max_iterations(20000)

    solver.resetProblem(problem)

    # create random initial guess
    guess = solver.createGuess("bounds")

    # generate random number between 1 to 10
    
    for _ in range(random.randint(1, 10)):
        guess.randomizeAdd()

    solver.setGuess(guess)

    print("created initial guess")

    guess_file_path = f"{marker_file.split('.')[0]}_guess.sto"
    guess.write(guess_file_path)

    track.set_guess_file(guess_file_path)
    
    # Solve and visualize.
    solution = study.solve()
    solution.write(f"{marker_file.split('.')[0]}_muscle_solution.sto")

# Solve the muscle-driven state tracking problem.
muscleDrivenStateTracking()