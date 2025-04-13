// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GenericAgent.hpp"
#include "conversion.hpp"
#include <Unreachable.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <AnticipationVelocityModel.hpp>
#include <CollisionFreeSpeedModel.hpp>
#include <CollisionFreeSpeedModelV2.hpp>
#include <GeneralizedCentrifugalForceModel.hpp>
#include <SocialForceModel.hpp>

namespace py = pybind11;

void init_agent(py::module_& m)
{
    py::class_<JPS_AgentIterator_Wrapper>(m, "CollisionFreeSpeedModelAgentIterator")
        .def(
            "__iter__",
            [](JPS_AgentIterator_Wrapper& w) -> JPS_AgentIterator_Wrapper& { return w; })
        .def("__next__", [](JPS_AgentIterator_Wrapper& w) {
            const auto result = JPS_AgentIterator_Next(w.handle);
            if(result) {
                return std::make_unique<JPS_Agent_Wrapper>(result);
            }
            throw py::stop_iteration{};
        });
    py::class_<JPS_AgentIdIterator_Wrapper>(m, "AgentIdIterator")
        .def(
            "__iter__",
            [](JPS_AgentIdIterator_Wrapper& w) -> JPS_AgentIdIterator_Wrapper& { return w; })
        .def("__next__", [](JPS_AgentIdIterator_Wrapper& w) {
            const auto id = JPS_AgentIdIterator_Next(w.handle);
            if(id != 0) {
                return id;
            }
            throw py::stop_iteration{};
        });
    py::class_<GenericAgent>(m, "Agent")
        .def_property_readonly("id", [](const GenericAgent& agent) { agent.id.getID(); })
        .def_property_readonly(
            "journey_id", [](const GenericAgent& agent) { return agent.journeyId.getID(); })
        .def_property_readonly(
            "stage_id", [](const GenericAgent& agent) { return agent.stageId.getID(); })
        .def_property_readonly(
            "position", [](const GenericAgent& agent) { return intoTuple(agent.position); })
        .def_property_readonly(
            "orientation", [](const GenericAgent& agent) { return intoTuple(agent.orientation); })
        .def_property(
            "target",
            [](const GenericAgent& agent) { return intoTuple(agent.target); },
            [](GenericAgent& agent, std::tuple<double, double> target) {
                agent.target = intoJPS_Point(target);
            })
        .def_property_readonly(
            "model",
            [](const GenericAgent& agnet)
                -> std::variant<
                    std::unique_ptr<GeneralizedCentrifugalForceModelState>,
                    std::unique_ptr<CollisionFreeSpeedModelState>,
                    std::unique_ptr<CollisionFreeSpeedModelV2State>,
                    std::unique_ptr<AnticipationVelocityModelState>,
                    std::unique_ptr<SocialForceModelState>>

            {
                switch(JPS_Agent_GetModelType(w.handle)) {
                    case JPS_GeneralizedCentrifugalForceModel:
                        return std::make_unique<JPS_GeneralizedCentrifugalForceModelState_Wrapper>(
                            JPS_Agent_GetGeneralizedCentrifugalForceModelState(w.handle, nullptr));
                    case JPS_CollisionFreeSpeedModel:
                        return std::make_unique<JPS_CollisionFreeSpeedModelState_Wrapper>(
                            JPS_Agent_GetCollisionFreeSpeedModelState(w.handle, nullptr));
                    case JPS_CollisionFreeSpeedModelV2:
                        return std::make_unique<JPS_CollisionFreeSpeedModelV2State_Wrapper>(
                            JPS_Agent_GetCollisionFreeSpeedModelV2State(w.handle, nullptr));
                    case JPS_AnticipationVelocityModel:
                        return std::make_unique<JPS_AnticipationVelocityModelState_Wrapper>(
                            JPS_Agent_GetAnticipationVelocityModelState(w.handle, nullptr));
                    case JPS_SocialForceModel:
                        return std::make_unique<JPS_SocialForceModelState_Wrapper>(
                            JPS_Agent_GetSocialForceModelState(w.handle, nullptr));
                }

                UNREACHABLE();
            });
}
