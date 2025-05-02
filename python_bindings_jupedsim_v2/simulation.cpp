// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"
#include "wrapper.hpp"

#include <Unreachable.hpp>
#include <jupedsim/jupedsim.h>

#include <CollisionGeometry.hpp>
#include <OperationalModel.hpp>
#include <Simulation.hpp>
#include <Stage.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_simulation(py::module_& m)
{
    py::class_<OperationalModel>(m, "OperationalModel");
    py::class_<Simulation>(m, "Simulation")
        .def(
            py::init([](const OperationalModel* model, CollisionGeometry geometry, double dT) {
                return std::make_unique<Simulation>(
                    model->Clone(), std::make_unique<CollisionGeometry>(geometry), dT);
            }),
            py::kw_only(),
            py::arg("model"),
            py::arg("geometry"),
            py::arg("dt"))
        .def(
            "add_waypoint_stage",
            [](Simulation& sim, Point position, double distance) {
                return sim.AddStage(WaypointDescription{position, distance}).getID();
            })
        .def(
            "add_queue_stage",
            [](Simulation& sim, const std::vector<Point>& positions) {
                return sim.AddStage(NotifiableQueueDescription{positions}).getID();
            })
        .def(
            "add_waiting_set_stage",
            [](Simulation& sim, const std::vector<Point>& positions) {
                return sim.AddStage(NotifiableWaitingSetDescription{positions}).getID();
            })
        .def(
            "add_exit_stage",
            [](Simulation& sim, const std::vector<Point>& polygon) {
                return sim.AddStage(ExitDescription{Polygon{polygon}}).getID();
            })
        .def(
            /// TODO(kkratz): CONTINUE HERE YOU MUPPET! :D
            "add_direct_steering_stage",
            [](Simulation& sim) { return sim.AddStage(DirectSteeringDescription{}).getID(); })
        .def(
            "add_journey",
            [](Simulation& sim, std::map<BaseStage::ID, TransitionDescription>& journey) {
                return sim.AddJourney(journey).getID();
            })
        .def(
            "add_agent",
            [](Simulation& sim, GenericAgent& agent) { return sim.AddAgent(agent).getID(); })
        .def(
            "mark_agent_for_removal",
            [](Simulation& sim, uint64_t id) { sim.MarkAgentForRemoval(id); })
        .def("removed_agents", [](const Simulation& sim) { return sim.RemovedAgents(); })
        .def("iterate", [](Simulation& sim) { sim.Iterate(); })
        .def(
            "switch_agent_journey",
            [](Simulation& sim,
               GenericAgent::ID agentId,
               Journey::ID journeyId,
               BaseStage::ID stageId) { sim.SwitchAgentJourney(agentId, journeyId, stageId); },
            py::kw_only(),
            py::arg("agent_id"),
            py::arg("journey_id"),
            py::arg("stage_id"))
        .def("agent_count", [](const Simulation& sim) { return sim.AgentCount(); })
        .def("elapsed_time", [](const Simulation& sim) { return sim.ElapsedTime(); })
        .def("delta_time", [](const Simulation& sim) { return sim.DT(); })
        .def("iteration_count", [](const Simulation& sim) { return sim.Iteration(); })
        .def(
            "agents",
            [](Simulation& sim) { return py::make_iterator(sim.Agents()); },
            py::keep_alive<0, 1>())
        .def(
            "agent",
            [](const Simulation& sim, GenericAgent::ID agentId) { return sim.Agent(agentId); },
            py::arg("agent_id"))
        .def(
            "agents_in_range",
            [](Simulation& sim, Point pos, double distance) {
                return sim.AgentsInRange(pos, distance);
            })
        .def(
            "agents_in_polygon",
            [](Simulation& sim, const std::vector<Point>& poly) {
                return sim.AgentsInPolygon(poly);
            })
        .def(
            // TODO(kkratz): CONTINUE HERE!
            "get_stage_proxy",
            [](JPS_Simulation_Wrapper& w, JPS_StageId id)
                -> std::variant<
                    std::unique_ptr<JPS_WaypointProxy_Wrapper>,
                    std::unique_ptr<JPS_NotifiableQueueProxy_Wrapper>,
                    std::unique_ptr<JPS_WaitingSetProxy_Wrapper>,
                    std::unique_ptr<JPS_ExitProxy_Wrapper>,
                    std::unique_ptr<JPS_DirectSteeringProxy_Wrapper>> {
                const auto type = JPS_Simulation_GetStageType(w.handle, id);
                JPS_ErrorMessage errorMessage{};
                const auto raise = [](JPS_ErrorMessage err) {
                    const auto msg = std::string(JPS_ErrorMessage_GetMessage(err));
                    JPS_ErrorMessage_Free(err);
                    throw std::runtime_error{msg};
                };

                switch(type) {
                    case JPS_NotifiableQueueType: {
                        auto ptr = std::make_unique<JPS_NotifiableQueueProxy_Wrapper>(
                            JPS_Simulation_GetNotifiableQueueProxy(w.handle, id, &errorMessage));
                        if(!ptr) {
                            raise(errorMessage);
                        }
                        return ptr;
                    }
                    case JPS_WaitingSetType: {
                        auto ptr = std::make_unique<JPS_WaitingSetProxy_Wrapper>(
                            JPS_Simulation_GetWaitingSetProxy(w.handle, id, &errorMessage));
                        if(!ptr) {
                            raise(errorMessage);
                        }
                        return ptr;
                    }
                    case JPS_WaypointType: {
                        auto ptr = std::make_unique<JPS_WaypointProxy_Wrapper>(
                            JPS_Simulation_GetWaypointProxy(w.handle, id, &errorMessage));
                        if(!ptr) {
                            raise(errorMessage);
                        }
                        return ptr;
                    }
                    case JPS_ExitType: {
                        auto ptr = std::make_unique<JPS_ExitProxy_Wrapper>(
                            JPS_Simulation_GetExitProxy(w.handle, id, &errorMessage));
                        if(!ptr) {
                            raise(errorMessage);
                        }
                        return ptr;
                    }
                    case JPS_DirectSteeringType: {
                        auto ptr = std::make_unique<JPS_DirectSteeringProxy_Wrapper>(
                            JPS_Simulation_GetDirectSteeringProxy(w.handle, id, &errorMessage));
                        if(!ptr) {
                            raise(errorMessage);
                        }
                        return ptr;
                    }
                }
                UNREACHABLE();
            })
        .def(
            "set_tracing",
            [](JPS_Simulation_Wrapper& w, bool status) {
                JPS_Simulation_SetTracing(w.handle, status);
            })
        .def(
            "get_last_trace",
            [](JPS_Simulation_Wrapper& w) { return JPS_Simulation_GetTrace(w.handle); })
        .def(
            "get_geometry",
            [](const JPS_Simulation_Wrapper& w) {
                return std::make_unique<JPS_Geometry_Wrapper>(JPS_Simulation_GetGeometry(w.handle));
            })
        .def("switch_geometry", [](JPS_Simulation_Wrapper& w, JPS_Geometry_Wrapper& geometry) {
            JPS_ErrorMessage errorMsg{};

            auto success =
                JPS_Simulation_SwitchGeometry(w.handle, geometry.handle, nullptr, &errorMsg);

            if(!success) {
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            }
            return success;
        });
}
