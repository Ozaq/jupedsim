// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"
#include <Unreachable.hpp>

#include <CollisionGeometry.hpp>
#include <OperationalModel.hpp>
#include <Simulation.hpp>
#include <Stage.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_simulation(py::module_& m)
{
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
            [](Simulation& sim, std::tuple<double, double> position, double distance) {
                return sim.AddStage(WaypointDescription{intoPoint(position), distance}).getID();
            })
        .def(
            "add_queue_stage",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& positions) {
                return sim.AddStage(NotifiableQueueDescription{intoPoints(positions)}).getID();
            })
        .def(
            "add_waiting_set_stage",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& positions) {
                return sim.AddStage(NotifiableWaitingSetDescription{intoPoints(positions)}).getID();
            })
        .def(
            "add_exit_stage",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& polygon) {
                return sim.AddStage(ExitDescription{Polygon{intoPoints(polygon)}}).getID();
            })
        .def(
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
            [](Simulation& sim, std::tuple<double, double> pos, double distance) {
                return sim.AgentsInRange(intoPoint(pos), distance);
            })
        .def(
            "agents_in_polygon",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& poly) {
                return sim.AgentsInPolygon(intoPoints(poly));
            })
        .def("get_stage_proxy", [](Simulation& sim, BaseStage::ID id) { return sim.Stage(id); })
        .def("set_tracing", [](Simulation& sim, bool status) { sim.SetTracing(status); })
        .def("get_last_trace", [](Simulation& sim) { return sim.GetLastStats(); })
        .def("get_geometry", [](Simulation& sim) { return sim.Geo(); })
        .def("switch_geometry", [](Simulation& sim, CollisionGeometry& geometry) {
            sim.SwitchGeometry(std::make_unique<CollisionGeometry>(geometry));
        });
}
