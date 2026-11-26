// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "agent_removal_system.hpp"
#include "destination.hpp"
#include "floorfield_routing_engine.hpp"
#include "generic_agent.hpp"
#include "geometry/geometry.hpp"
#include "geometry/location.hpp"
#include "neighborhood_search.hpp"
#include "operational_decision_system.hpp"
#include "operational_model.hpp"
#include "operational_model_state.hpp"
#include "operational_model_type.hpp"
#include "polygon.hpp"
#include "records.hpp"
#include "route.hpp"
#include "routing_engine.hpp"
#include "simulation_clock.hpp"
#include "stage.hpp"
#include "stage_description.hpp"
#include "stage_manager.hpp"
#include "strategical_desicion_system.hpp"
#include "tactical_decision_system.hpp"
#include "timing.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

class Simulation
{
    /// Marks an entry point that must not be re-entered as running, for as long as it lives:
    /// its name is in `_running` from construction to scope exit, which includes an exception
    /// unwinding out of the iteration pipeline or out of a callback of the model.
    class ReentrancyGuard
    {
        const char*& _running;

    public:
        ReentrancyGuard(const char*& running, const char* operation) : _running(running)
        {
            _running = operation;
        }
        ~ReentrancyGuard() { _running = nullptr; }
        ReentrancyGuard(const ReentrancyGuard&) = delete;
        ReentrancyGuard& operator=(const ReentrancyGuard&) = delete;
    };

    SimulationClock _clock;
    StrategicalDecisionSystem _stategical_decision_system{};
    TacticalDecisionSystem _tactical_decision_system{};
    OperationalDecisionSystem _operational_decision_system;
    AgentRemovalSystem<GenericAgent> _agent_removal_system{};
    StageManager _stage_manager{};
    NeighborhoodSearch<GenericAgent> _neighborhood_search{2.2};
    std::shared_ptr<const Geometry> _geometry{};
    std::unique_ptr<RoutingEngine> _routing_engine{};
    AgentContainer<GenericAgent> _agents;
    /// Agents that reached an exit or were marked, removed at the beginning of the next
    /// iteration.
    std::vector<RemovalRecord> _marked_for_removal;
    std::vector<TargetReachedRecord> _reached_targets;
    std::vector<RemovalRecord> _removals;
    std::unordered_map<Route::ID, std::unique_ptr<Route>> _routes;
    /// Only named agents.
    std::unordered_map<std::string, GenericAgent::ID> _agent_ids_by_name;
    Timer _timer{};
    /// The entry point that is running and must not be re-entered, by the name its callers
    /// know it under; nullptr if none is. Entry points that change the simulation must not run
    /// while one of them does: they work on the agent containers, and some of them run
    /// callbacks of the model, which may call back into the simulation. Entry points that only
    /// read may run at any time.
    const char* _running{nullptr};
    /// A closed simulation rejects every entry point but close() and the timer's.
    bool _closed{false};
    enum LogLevel { General = 1, Detailed = 2, Debug = 3 };

    /// Throws if the simulation is closed. Every entry point that only reads starts with this.
    void check_open(const char* operation) const;
    /// Throws if an entry point that must not be re-entered is running.
    void check_not_running(const char* operation) const;
    /// Throws if the simulation is closed, or if an entry point that must not be re-entered is
    /// running; else @p operation is that entry point until the guard returned goes out of
    /// scope. Every entry point that changes the simulation starts with this, whether or not it
    /// runs callbacks itself.
    [[nodiscard]] ReentrancyGuard enter(const char* operation);
    /// Throws unless @p location lies on this simulation's geometry, where the floor field can
    /// route from it. The precondition of every entry point that takes a location: no routing
    /// query fails later for where it is asked.
    void check_routable(const Location& location, const char* what) const;
    Route& route(Route::ID route_id);
    /// The stage @p stage_id of @p route; throws if it is not part of it.
    BaseStage* stage_in(const Route& route, BaseStage::ID stage_id) const;
    /// The agent @p agent_id; throws if it leaves the simulation.
    GenericAgent& reroutable(GenericAgent::ID agent_id);
    /// Lets @p agent follow @p route_id from @p stage on.
    void follow(GenericAgent& agent, Route::ID route_id, BaseStage& stage);
    void validate_new_agent(const GenericAgent& agent) const;
    /// Lets the model seed the state of the new agent @p agent, which a custom model may fail
    /// to do. The last thing that can reject an agent, so it comes before whatever leaves a
    /// trace, see follow(); and only once the agent is validated, so that a rejected one takes
    /// no stream from the model's sequence.
    void seed(GenericAgent& agent);
    /// Throws if another agent has @p name.
    void check_agent_name(const std::string& name) const;
    /// Adds @p agent, with its orientation, and returns its id.
    GenericAgent::ID emplace_agent(GenericAgent&& agent);

public:
    /// Shares ownership of the geometry, which is never modified: several simulations, or a
    /// simulation and a routing engine, may use the same geometry. `geo()` shares it too.
    Simulation(
        std::unique_ptr<OperationalModel>&& operational_model,
        std::shared_ptr<const Geometry> geometry,
        double dt,
        FloorfieldOptions floorfield_options);

    Simulation(const Simulation& other) = delete;
    Simulation& operator=(const Simulation& other) = delete;
    Simulation(Simulation&& other) = delete;
    Simulation& operator=(Simulation&& other) = delete;
    ~Simulation() = default;
    const SimulationClock& clock() const;
    void set_tracing(bool on);
    void iterate();
    /// Closes the simulation freeing memory. Every entry point (except timer's) throws afterwards.
    void close();
    /// @p name has to be unique among the stages, if given.
    BaseStage::ID
    add_stage(const StageDescription& stage_description, std::optional<std::string> name = {});
    /// See Route::Route for what @p transitions must hold. @p name has to be unique among the
    /// routes, if given.
    Route::ID add_route(
        const std::vector<RouteTransitionDescription>& transitions,
        std::optional<std::string> name = {});
    /// Removes the agent at the beginning of the next iteration. Nothing changes for an agent
    /// that is about to leave already.
    void mark_agent_for_removal(GenericAgent::ID id);
    /// The targets reached in the most recent iteration, and by the agents added since.
    const std::vector<TargetReachedRecord>& reached_targets() const;
    /// The agents removed at the beginning of the most recent iteration.
    const std::vector<RemovalRecord>& removed_agents() const;
    size_t agent_count() const;
    double elapsed_time() const;
    double dt() const;
    /// Lets the agent follow @p route_id, from @p first_stage on, or from where the route's
    /// first transition leads if none is given. The agent reaches its new target after the next
    /// step at the earliest, even if it stands in it already. Every reroute() throws for an agent
    /// that leaves the simulation: it reached an exit, or was marked for removal.
    void reroute(
        GenericAgent::ID agent_id,
        Route::ID route_id,
        std::optional<BaseStage::ID> first_stage = {});
    /// Lets the agent go on from @p first_stage along the route it follows.
    void reroute(GenericAgent::ID agent_id, BaseStage::ID first_stage);
    /// Steers the agent to @p steer_to, off its route.
    void reroute(GenericAgent::ID agent_id, const Location& steer_to);
    uint64_t iteration() const;
    /// Ids of the agents within @p distance of @p from that are neighbors in the sense of the
    /// operational models: in plain sight over the surface, see Geometry::no_geometry_between.
    /// Agents on a floor above or below are not.
    std::vector<GenericAgent::ID> agents_in_range(const Location& from, double distance);
    /// Ids of the agents inside @p polygon, clipped to region @p region_id as a stage area is.
    /// Throws if @p polygon does not intersect that region.
    std::vector<GenericAgent::ID> agents_in_polygon(const Polygon& polygon, std::size_t region_id);
    /// Adds an agent at @p location that follows @p route_id, see reroute(). An agent added
    /// inside its first stage has reached it; it moves on in the next iteration.
    /// @p name has to be unique among the agents, unless empty, which means no name; it is free
    /// again once its agent has left.
    GenericAgent::ID add_agent(
        const Location& location,
        Route::ID route_id,
        std::optional<BaseStage::ID> first_stage,
        OperationalModelState model,
        std::string name = {});
    /// Adds an agent at @p location that is steered to @p steer_to.
    GenericAgent::ID add_agent(
        const Location& location,
        const Location& steer_to,
        OperationalModelState model,
        std::string name = {});
    /// The agent named @p name, if there is one; never one for the empty name.
    std::optional<GenericAgent::ID> agent_named(const std::string& name) const;
    /// See `Geometry::get_location`.
    Location get_location(double x, double y, std::size_t region_id) const;
    const GenericAgent& agent(GenericAgent::ID id) const;
    GenericAgent& agent(GenericAgent::ID id);
    AgentContainer<GenericAgent>& agents();
    OperationalModelType model_type() const;
    const BaseStage* stage(BaseStage::ID stage_id) const;
    /// Ids of all stages, in ascending order.
    std::vector<BaseStage::ID> stage_ids() const;
    const Route& route(Route::ID route_id) const;
    /// Ids of all routes, in ascending order.
    std::vector<Route::ID> route_ids() const;
    /// Travel times to the area of stage @p stage_id over the grid of every region, by region id.
    std::map<std::size_t, TravelTimeGrid> travel_times(BaseStage::ID stage_id);
    /// Travel time in seconds from @p from to the area of stage @p stage_id, see
    /// FloorfieldRoutingEngine::travel_time.
    double travel_time(BaseStage::ID stage_id, const Location& from);
    /// Speed fields every stage without an override is routed with, by region id.
    std::map<std::size_t, SpeedGrid> default_speed_fields() const;
    /// Placement of the floor field grid of @p region_id, which its travel times and speed
    /// fields lie on. Throws for an unknown region.
    GridInfo grid_info(std::size_t region_id) const;
    /// Replaces the default speed field of @p region_id, see
    /// FloorfieldRoutingEngine::set_default_speed for what @p values must hold.
    void set_default_speed_field(std::size_t region_id, const std::vector<double>& values);
    /// The speed fields of stage @p stage_id's own, by region id: empty (std::nullopt) where it
    /// is routed with the default.
    std::map<std::size_t, std::optional<SpeedGrid>>
    custom_speed_fields(BaseStage::ID stage_id) const;
    /// Whether stage @p stage_id has a speed field of its own in @p region_id, without copying
    /// it. Throws for an unknown region.
    bool has_custom_speed_field(BaseStage::ID stage_id, std::size_t region_id) const;
    /// Routes agents to the area of stage @p stage_id with @p values in @p region_id instead of
    /// the default speed field. Other stages are not affected.
    void set_custom_speed_field(
        BaseStage::ID stage_id,
        std::size_t region_id,
        const std::vector<double>& values);
    /// Routes agents to the area of stage @p stage_id with the default speed field of
    /// @p region_id again.
    void clear_custom_speed_field(BaseStage::ID stage_id, std::size_t region_id);
    /// The geometry this simulation runs on. Borrowed: it lives as long as the simulation.
    const std::shared_ptr<const Geometry>& geo() const;
    void push_timer(const std::string_view name, size_t probe_log_level = 0);
    void pop_timer(const std::string_view name);
    void set_timer_log_level(int level) { _timer.set_log_level(level); };
    TimerEntry::DurationType get_timer_duration(const std::string_view name) const;
    std::map<std::string, TimerEntry::DurationType> get_timer_durations() const;
};
