#
# Copyright (c) 2008-2018 the Urho3D project.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

if (NOT URHO3D_URHO2D)
    return ()
endif ()

# Define target name
set (TARGET_NAME MayaScape)

# Define source files
#define_source_files (EXTRA_H_FILES ${COMMON_SAMPLE_H_FILES} ./Sample2D.h ./Sample2D.cpp ../Utilities2D/Mover.h ../Utilities2D/Mover.cpp ai/genotype.cpp ai/genotype.h util/random_d.h ai/genetic_algorithm.cpp ai/genetic_algorithm.h ai/evolution_manager.cpp ai/evolution_manager.h ai/agent.cpp ai/agent.h ai/neural_layer.cpp ai/neural_layer.h ai/neural_network.cpp ai/neural_network.h util/math_helper.cpp util/math_helper.h util/event.cpp util/event.h ai/agent_controller.cpp ai/agent_controller.h shared_libs.h ai/sensor.cpp ai/sensor.h app.cpp app.h ai/agent_movement.cpp ai/agent_movement.h ai/fsm_event_data.cpp ai/fsm_event_data.h ai/fsm.cpp ai/fsm.h util/semaphore.h ai/agent_fsm.cpp ai/agent_fsm.h util/event.cpp util/event.h weapondef.h visibility.cpp visibility.h terrain.cpp terrain.h types.h structuredef.h structure.cpp structure.h statsdef.h stats.cpp stats.h stringdef.h researchdef.h research.cpp research.h raycast.cpp raycast.h projectiledef.h projectile.cpp projectile.h pointtree.cpp pointtree.h orderdef.h order.cpp order.h objectdef.h objmem.cpp objmem.h objects.cpp objects.h movedef.h move.cpp move.h mapgrid.cpp mapgrid.h mapdisplay.cpp mapdisplay.h map.cpp map.h messagedef.h message.cpp message.h geometry.cpp geometry.h featuredef.h feature.cpp feature.h fpath.cpp fpath.h droiddef.h droid.cpp droid.h displaydef.h display3ddef.h display3d.cpp display3d.h display.cpp display.h component.cpp component.h console.cpp console.h basedef.h baseobject.cpp baseobject.h astar.cpp astar.h actiondef.h action.cpp action.h)


#define_source_files (EXTRA_H_FILES ${COMMON_SAMPLE_H_FILES} not-used/Vehicle.cpp not-used/Vehicle.h ./Sample2D.h ./Sample2D.cpp ../Utilities2D/Mover.h ../Utilities2D/Mover.cpp ai/genotype.cpp ai/genotype.h util/random_d.h ai/genetic_algorithm.cpp ai/genetic_algorithm.h ai/evolution_manager.cpp ai/evolution_manager.h ai/agent.cpp ai/agent.h ai/neural_layer.cpp ai/neural_layer.h ai/neural_network.cpp ai/neural_network.h util/math_helper.cpp util/math_helper.h util/event.cpp util/event.h ai/agent_controller.cpp ai/agent_controller.h shared_libs.h ai/sensor.cpp ai/sensor.h app.cpp app.h ai/agent_movement.cpp ai/agent_movement.h ai/fsm_event_data.cpp ai/fsm_event_data.h ai/fsm.cpp ai/fsm.h util/semaphore.h ai/agent_fsm.cpp ai/agent_fsm.h util/event.cpp util/event.h types.h stringdef.h pointtree.cpp pointtree.h objectdef.h objmem.cpp objmem.h objects.cpp objects.h movedef.h mapgrid.cpp mapgrid.h map.cpp map.h geometry.cpp geometry.h fpath.cpp fpath.h basedef.h baseobject.cpp baseobject.h astar.cpp astar.h actiondef.h action.cpp action.h)


# Define source files
#define_source_files (EXTRA_H_FILES ${COMMON_SAMPLE_H_FILES} ./Sample2D.h ./Sample2D.cpp ../Utilities2D/Mover.h ../Utilities2D/Mover.cpp ai/genotype.cpp ai/genotype.h util/random_d.h ai/genetic_algorithm.cpp ai/genetic_algorithm.h ai/evolution_manager.cpp ai/evolution_manager.h ai/agent.cpp ai/agent.h ai/neural_layer.cpp ai/neural_layer.h ai/neural_network.cpp ai/neural_network.h util/math_helper.cpp util/math_helper.h util/event.cpp util/event.h ai/agent_controller.cpp ai/agent_controller.h shared_libs.h ai/sensor.cpp ai/sensor.h app.cpp app.h ai/agent_movement.cpp ai/agent_movement.h ai/fsm_event_data.cpp ai/fsm_event_data.h ai/fsm.cpp ai/fsm.h util/semaphore.h ai/agent_fsm.cpp ai/agent_fsm.h util/event.cpp util/event.h weapondef.h visibility.cpp visibility.h terrain.cpp terrain.h types.h  stringdef.h researchdef.h research.cpp research.h raycast.cpp raycast.h projectiledef.h projectile.cpp projectile.h pointtree.cpp pointtree.h orderdef.h order.cpp order.h objectdef.h objmem.cpp objmem.h objects.cpp objects.h mapgrid.cpp mapgrid.h map.cpp map.h messagedef.h message.cpp message.h geometry.cpp geometry.h fpath.cpp fpath.h droiddef.h droid.cpp droid.h displaydef.h display3ddef.h display3d.cpp display3d.h display.cpp display.h component.cpp component.h bucket3d.cpp bucket3d.h basedef.h baseobject.cpp baseobject.h astar.cpp astar.h actiondef.h action.cpp action.h)
#stats.cpp stats.h
#structuredef.h structure.cpp structure.h statsdef.h
#movedef.h move.cpp move.h
#featuredef.h feature.cpp feature.h
#console.cpp console.h

define_source_files (EXTRA_H_FILES ${COMMON_SAMPLE_H_FILES} WheelTrackModel.cpp WheelTrackModel.h Vehicle.cpp Vehicle.h RaycastVehicle.cpp RaycastVehicle.h SmoothStep.cpp SmoothStep.h ./Sample2D.h ./Sample2D.cpp ../Utilities2D/Mover.h ../Utilities2D/Mover.cpp ai/genotype.cpp ai/genotype.h util/random_d.h ai/genetic_algorithm.cpp ai/genetic_algorithm.h ai/evolution_manager.cpp ai/evolution_manager.h ai/agent.cpp ai/agent.h ai/neural_layer.cpp ai/neural_layer.h ai/neural_network.cpp ai/neural_network.h util/math_helper.cpp util/math_helper.h util/event.cpp util/event.h ai/agent_controller.cpp ai/agent_controller.h shared_libs.h ai/sensor.cpp ai/sensor.h app.cpp app.h ai/agent_movement.cpp ai/agent_movement.h ai/fsm_event_data.cpp ai/fsm_event_data.h ai/fsm.cpp ai/fsm.h util/semaphore.h ai/agent_fsm.cpp ai/agent_fsm.h util/event.cpp util/event.h types.h stringdef.h pointtree.cpp pointtree.h objectdef.h objmem.cpp objmem.h objects.cpp objects.h movedef.h mapgrid.cpp mapgrid.h map.cpp map.h geometry.cpp geometry.h fpath.cpp fpath.h basedef.h baseobject.cpp baseobject.h astar.cpp astar.h actiondef.h action.cpp action.h)


# Setup target with resource copying
setup_main_executable ()

# Setup test cases
setup_test ()
