// SPDX-License-Identifier: LGPL-3.0-or-later
//
// WarpDriver collision-avoidance model.
// Based on Wolinski, Lin, and Pettré (2016) — PhD thesis Chapter 4, Appendix B.
//
// Warp operators implemented (B.1–B.15):
//   W_ref  — local frame change (W_local: A's frame → B's frame)
//   W_th   — time horizon normalization (B.1–B.3)
//   W_tu   — time uncertainty with probability scaling (B.4–B.6)
//   W_r    — radius normalization via Minkowski sum (B.7–B.9)
//   W_v    — velocity shear (B.10–B.12)
//   W_vu   — anisotropic velocity uncertainty with probability scaling (B.13–B.15)
//
// Non-thesis safety mechanisms (practical additions):
//   - Short-range repulsion (3× combined radius) to prevent overlaps
//   - Boundary wall steering
//   - Stuck detection with lateral detour to break narrow-passage deadlocks
//   - Lateral perturbation for symmetry breaking
//
// TODO: W_ref currently uses simple W_local (straight-line frame change).
//   The thesis defines graph-based variants (Algorithm 3) that warp space
//   along navigable paths — W_el (environment layout), W_io (obstacle
//   interactions), W_ob (observed behaviors). These would enable anticipatory
//   avoidance around corners and bends. The routing infrastructure exists
//   (RoutingEngine::ComputeAllWaypoints provides the full waypoint path);
//   the path could serve as the graph for Algorithm 3's spatial projection.
//
#include "WarpDriverModel.hpp"

#include "GenericAgent.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"
#include "WarpDriverModelData.hpp"
#include "WarpDriverModelUpdate.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <variant>

// ============================================================================
// IntrinsicField
// ============================================================================

void WarpDriverModel::IntrinsicField::Compute(double sigma)
{
    Nx = static_cast<int>(std::round((XMax - XMin) / Dx)) + 1;
    Ny = static_cast<int>(std::round((YMax - YMin) / Dy)) + 1;
    Values.resize(static_cast<size_t>(Nx * Ny), 0.0);
    Gradients.resize(static_cast<size_t>(Nx * Ny), Point{0.0, 0.0});

    const double sigma_squared = sigma * sigma;

    // Compute I(x,y) = (f * g)(x,y) where g = unit disk, f = Gaussian(sigma).
    // For each grid point, numerically integrate the convolution over the disk.
    const double integrationStep = 0.05;
    const double integrationRadius = 1.0; // unit disk support

    for(int ix = 0; ix < Nx; ++ix) {
        for(int iy = 0; iy < Ny; ++iy) {
            const double px = XMin + ix * Dx;
            const double py = YMin + iy * Dy;

            double val = 0.0;
            // Integrate f(px-u, py-v) * g(u,v) du dv over g's support (unit disk)
            for(double u = -integrationRadius; u <= integrationRadius; u += integrationStep) {
                for(double v = -integrationRadius; v <= integrationRadius; v += integrationStep) {
                    if(u * u + v * v <= 1.0) {
                        double dx2 = px - u;
                        double dy2 = py - v;
                        val += std::exp(-(dx2 * dx2 + dy2 * dy2) / (2.0 * sigma_squared));
                    }
                }
            }
            val *= integrationStep * integrationStep;
            Values[static_cast<size_t>(ix * Ny + iy)] = val;
        }
    }

    // Normalize so peak ≈ 1
    const double maxVal = *std::max_element(Values.begin(), Values.end());
    if(maxVal > 0.0) {
        for(auto& v : Values) {
            v /= maxVal;
        }
    }

    // Compute gradients via central differences
    for(int ix = 0; ix < Nx; ++ix) {
        for(int iy = 0; iy < Ny; ++iy) {
            double dIdx = 0.0;
            double dIdy = 0.0;
            if(ix > 0 && ix < Nx - 1) {
                dIdx = (Values[static_cast<size_t>((ix + 1) * Ny + iy)] -
                        Values[static_cast<size_t>((ix - 1) * Ny + iy)]) /
                       (2.0 * Dx);
            }
            if(iy > 0 && iy < Ny - 1) {
                dIdy = (Values[static_cast<size_t>(ix * Ny + (iy + 1))] -
                        Values[static_cast<size_t>(ix * Ny + (iy - 1))]) /
                       (2.0 * Dy);
            }
            Gradients[static_cast<size_t>(ix * Ny + iy)] = Point{dIdx, dIdy};
        }
    }
}

std::pair<double, Point> WarpDriverModel::IntrinsicField::Sample(double x, double y) const
{
    if(x < XMin || x > XMax || y < YMin || y > YMax) {
        return {0.0, Point{0.0, 0.0}};
    }

    const double fx = (x - XMin) / Dx;
    const double fy = (y - YMin) / Dy;
    const int ix = std::clamp(static_cast<int>(fx), 0, Nx - 2);
    const int iy = std::clamp(static_cast<int>(fy), 0, Ny - 2);
    const double sx = fx - ix;
    const double sy = fy - iy;

    const auto idx = [&](int i, int j) -> size_t { return static_cast<size_t>(i * Ny + j); };

    // Bilinear interpolation
    const double v00 = Values[idx(ix, iy)];
    const double v10 = Values[idx(ix + 1, iy)];
    const double v01 = Values[idx(ix, iy + 1)];
    const double v11 = Values[idx(ix + 1, iy + 1)];
    const double val =
        v00 * (1 - sx) * (1 - sy) + v10 * sx * (1 - sy) + v01 * (1 - sx) * sy + v11 * sx * sy;

    const Point g00 = Gradients[idx(ix, iy)];
    const Point g10 = Gradients[idx(ix + 1, iy)];
    const Point g01 = Gradients[idx(ix, iy + 1)];
    const Point g11 = Gradients[idx(ix + 1, iy + 1)];
    const Point grad = g00 * ((1 - sx) * (1 - sy)) + g10 * (sx * (1 - sy)) + g01 * ((1 - sx) * sy) +
                       g11 * (sx * sy);

    return {val, grad};
}

// ============================================================================
// Warp Operators
// ============================================================================

namespace
{

using STP = WarpDriverModel::SpaceTimePoint;

// W_local: change of reference frame from agent a to agent b
STP WarpLocalForward(const STP& s, Point posA, Point orientA, Point posB, Point orientB)
{
    // Rotate from a's frame to world
    const double cosA = orientA.X;
    const double sinA = orientA.Y;
    const double wx = cosA * s.X - sinA * s.Y + posA.X;
    const double wy = sinA * s.X + cosA * s.Y + posA.Y;

    // World to b's frame
    const double dx = wx - posB.X;
    const double dy = wy - posB.Y;
    const double cosB = orientB.X;
    const double sinB = orientB.Y;
    return STP{cosB * dx + sinB * dy, -sinB * dx + cosB * dy, s.T};
}

// W_v: velocity shear. In b's frame, x' = x - speed_b * t
STP WarpVelocityForward(const STP& s, double speedB)
{
    return STP{s.X - speedB * s.T, s.Y, s.T};
}

// W_r: radius scaling (B.7). W_r(s) = s ★ (1/α, 1/α, 1).
STP WarpRadiusForward(const STP& s, double radiusB)
{
    const double invR = 1.0 / std::max(radiusB, 1e-6);
    return STP{s.X * invR, s.Y * invR, s.T};
}

// W_ts: time uncertainty. Scale (x,y) by 1/(1 + lambda*t)
STP WarpTimeUncertaintyForward(const STP& s, double lambda)
{
    const double scale = 1.0 / (1.0 + lambda * std::max(s.T, 0.0));
    return STP{s.X * scale, s.Y * scale, s.T};
}

struct VelocityUncertaintyScale {
    double Beta1;
    double Beta2;
};

// B.13: β₁ = 1/(1 + α₁·v/v_pref), β₂ = 1 + α₂·v/v_pref.
// Since we use v0 for both current and preferred speed, v/v_pref = 1.
VelocityUncertaintyScale VelocityUncertaintyFactors(double uncertaintyX, double uncertaintyY)
{
    return {1.0 / (1.0 + uncertaintyX), 1.0 + uncertaintyY};
}

// W_vu: velocity uncertainty (B.13). Anisotropic scaling:
// β₁ = 1/(1 + α₁) compresses x, β₂ = 1 + α₂ expands y.
STP WarpVelocityUncertaintyForward(const STP& s, double uncertaintyX, double uncertaintyY)
{
    const auto [beta1, beta2] = VelocityUncertaintyFactors(uncertaintyX, uncertaintyY);
    return STP{s.X * beta1, s.Y * beta2, s.T};
}

// Full composition: forward from a's frame to b's Intrinsic Field space
// Order: W_local -> W_v -> W_r -> W_ts -> W_vu
// Then check W_th (time validity)
struct WarpParams {
    Point PosA;
    Point OrientA;
    Point PosB;
    Point OrientB;
    double SpeedB;
    double RadiusB;
    double Lambda;
    double VelocityUncertaintyX;
    double VelocityUncertaintyY;
    double TimeHorizon;
};

// Probability scaling (B.5 + B.14): product of inverse probability transforms.
// W_tu^{-1}(p) = p*beta^2, W_vu^{-1}(p) = p*beta1*beta2.
double ProbabilityScale(const STP& sOriginal, const WarpParams& p)
{
    const double beta_tu = 1.0 / (1.0 + p.Lambda * std::max(sOriginal.T, 0.0));
    const auto [beta1, beta2] =
        VelocityUncertaintyFactors(p.VelocityUncertaintyX, p.VelocityUncertaintyY);
    return beta_tu * beta_tu * beta1 * beta2;
}

STP ComposeForward(const STP& s, const WarpParams& p)
{
    auto s1 = WarpLocalForward(s, p.PosA, p.OrientA, p.PosB, p.OrientB);
    auto s2 = WarpVelocityForward(s1, p.SpeedB);
    auto s3 = WarpRadiusForward(s2, p.RadiusB);
    auto s4 = WarpTimeUncertaintyForward(s3, p.Lambda);
    auto s5 = WarpVelocityUncertaintyForward(s4, p.VelocityUncertaintyX, p.VelocityUncertaintyY);
    // Normalize time: map [0, timeHorizon] -> [0, 1]
    s5.T = (p.TimeHorizon > 0.0) ? s5.T / p.TimeHorizon : 0.0;
    return s5;
}

// Gradient transform: takes 2D gradient from IntrinsicField, returns 3D space-time gradient
// in a's frame. Applies inverse Jacobians in reverse order.
STP ComposeGradientInverse(const Point& gradI, const STP& sOriginal, const WarpParams& p)
{
    // Start with 3-component gradient in Intrinsic Field space: (gradI.X, gradI.Y, 0)
    // since dI/dt = 0
    double gx = gradI.X;
    double gy = gradI.Y;
    double gt = 0.0;

    // Time normalization inverse Jacobian: dt_original = dt_normalized * timeHorizon
    // So dI/dt_original = dI/dt_normalized / timeHorizon
    // But dI/dt = 0, so gt stays 0 at this point. However, the spatial components
    // pick up time contributions from the velocity shear.

    // W_vu^-1: anisotropic scaling (B.15). Inverse scales gradient by the
    // forward factors (beta1, beta2) since J_vu = diag(beta1, beta2, 1).
    {
        const auto [beta1, beta2] =
            VelocityUncertaintyFactors(p.VelocityUncertaintyX, p.VelocityUncertaintyY);
        gx *= beta1;
        gy *= beta2;
    }

    // W_tu^-1 (B.6): spatial gradient scaled by beta, temporal gets cross-terms.
    // Coordinates at W_tu input = after W_ref -> W_v -> W_r on sOriginal.
    // TODO(perf): ComposeForward already computes this intermediate; cache and
    // reuse instead of recomputing the three warps here. ~15% per-sample saving.
    {
        const double t = sOriginal.T;
        const double beta = 1.0 / (1.0 + p.Lambda * std::max(t, 0.0));
        auto sAtTu = WarpLocalForward(sOriginal, p.PosA, p.OrientA, p.PosB, p.OrientB);
        sAtTu = WarpVelocityForward(sAtTu, p.SpeedB);
        sAtTu = WarpRadiusForward(sAtTu, p.RadiusB);
        const double gamma1 = -p.Lambda * beta * beta * sAtTu.X;
        const double gamma2 = -p.Lambda * beta * beta * sAtTu.Y;
        const double gxOld = gx;
        const double gyOld = gy;
        gx *= beta;
        gy *= beta;
        gt = gamma1 * gxOld + gamma2 * gyOld + gt;
    }

    // W_r^-1: identity (B.9).

    // W_v^-1 (B.12): g + (0, 0, -v·g.X).
    {
        gt -= p.SpeedB * gx;
    }

    // W_local inverse Jacobian: rotate from b's frame back to a's frame
    {
        const double cosA = p.OrientA.X;
        const double sinA = p.OrientA.Y;
        const double cosB = p.OrientB.X;
        const double sinB = p.OrientB.Y;

        // Combined rotation: b's frame -> world -> a's frame
        // R_a^T * R_b applied to gradient
        const double cos_ab = cosA * cosB + sinA * sinB;
        const double sin_ab = sinA * cosB - cosA * sinB;
        const double gx_new = cos_ab * gx + sin_ab * gy;
        const double gy_new = -sin_ab * gx + cos_ab * gy;
        gx = gx_new;
        gy = gy_new;
    }

    return STP{gx, gy, gt};
}

} // anonymous namespace

// ============================================================================
// WarpDriverModel
// ============================================================================

WarpDriverModel::WarpDriverModel(
    double timeHorizon,
    double stepSize,
    double sigma,
    double timeUncertainty,
    double velocityUncertaintyX,
    double velocityUncertaintyY,
    int numSamples,
    uint64_t rngSeed)
    : timeHorizon(timeHorizon)
    , stepSize(stepSize)
    , timeUncertainty(timeUncertainty)
    , velocityUncertaintyX(velocityUncertaintyX)
    , velocityUncertaintyY(velocityUncertaintyY)
    , numSamples(numSamples)
    // Neighborhood cutoff: maximum distance at which a neighbor can still
    // collide with us within timeHorizon. Two agents closing head-on cover
    // 2 * v_max * timeHorizon, plus their combined radii, plus a small margin.
    // v_max and r_max are hardcoded pedestrian defaults; promote to constructor
    // parameters if mixed-speed populations need a tighter or wider cutoff.
    , cutOffRadius(2.0 * 1.5 * timeHorizon + 2.0 * 0.3 + 0.5)
    , rng(rngSeed)
{
    intrinsicField.Compute(sigma);
}

OperationalModelType WarpDriverModel::Type() const
{
    return OperationalModelType::WARP_DRIVER;
}

std::unique_ptr<OperationalModel> WarpDriverModel::Clone() const
{
    return std::make_unique<WarpDriverModel>(*this);
}

void WarpDriverModel::ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const
{
    const auto& upd = std::get<WarpDriverModelUpdate>(update);
    auto& data = std::get<WarpDriverModelData>(agent.Model);
    agent.Pos = upd.Position;
    agent.Orientation = upd.Orientation;
    data.StuckTime = upd.StuckTime;
    data.AnchorX = upd.AnchorX;
    data.AnchorY = upd.AnchorY;
    data.DetourTime = upd.DetourTime;
    data.DetourSide = upd.DetourSide;
}

void WarpDriverModel::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& /*neighborhoodSearch*/,
    const CollisionGeometry& /*geometry*/) const
{
    const auto* data = std::get_if<WarpDriverModelData>(&agent.Model);
    if(!data) {
        throw SimulationError(
            "WarpDriverModel constraint check: agent {} does not have WarpDriverModelData",
            agent.AgentID);
    }
    if(data->Radius <= 0.0) {
        throw SimulationError(
            "WarpDriverModel constraint check: agent {} has invalid radius {}",
            agent.AgentID,
            data->Radius);
    }
    if(data->V0 < 0.0) {
        throw SimulationError(
            "WarpDriverModel constraint check: agent {} has invalid v0 {}", agent.AgentID, data->V0);
    }
}

OperationalModelUpdate WarpDriverModel::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    const auto& agentData = std::get<WarpDriverModelData>(ped.Model);
    const double speed = agentData.V0;

    // Agent orientation (unit vector). If zero, default to +x.
    Point orient = ped.Orientation;
    if(orient.Norm() < 1e-9) {
        orient = Point{1.0, 0.0};
    } else {
        orient = orient.Normalized();
    }

    // Direction towards destination
    Point toTarget = ped.Destination - ped.Pos;
    const double distToTarget = toTarget.Norm();
    if(distToTarget < 1e-9) {
        return WarpDriverModelUpdate{ped.Pos, orient};
    }
    Point desiredDir = toTarget.Normalized();

    // Use desired direction as agent's effective orientation for the frame
    Point effectiveOrient = desiredDir;

    // === Step 1: Projected trajectory in agent-centric space ===
    // r(t) = (speed * t, 0, t) for t in [0, timeHorizon]
    const double dtSample = timeHorizon / std::max(numSamples - 1, 1);

    // === Step 2: Perceive - build collision probability field ===
    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(ped.Pos, cutOffRadius);

    // Short-range repulsion: not part of the original Wolinski et al. (2016)
    // model, which is purely anticipatory. Added as a practical safety net
    // because the collision probability field alone cannot guarantee separation
    // when agents are already close (dense crowds, late reactions).
    // Similar to the pushout mechanisms in CFS and AVM.
    Point repulsion{0.0, 0.0};
    for(const auto& neighbor : neighbors) {
        if(neighbor.AgentID == ped.AgentID) {
            continue;
        }
        const auto* nbData = std::get_if<WarpDriverModelData>(&neighbor.Model);
        if(!nbData) {
            continue;
        }
        Point diff = ped.Pos - neighbor.Pos;
        const double dist = diff.Norm();
        const double combinedRadius = agentData.Radius + nbData->Radius;
        if(dist < combinedRadius * 3.0 && dist > 1e-6) {
            const double overlap = combinedRadius * 3.0 - dist;
            repulsion = repulsion + diff.Normalized() * (speed * overlap / dist);
        } else if(dist <= 1e-6) {
            repulsion = repulsion + Point{-desiredDir.Y, desiredDir.X} * speed;
        }
    }

    // Random perturbation: small lateral offset on trajectory samples to break
    // symmetry in perfectly aligned head-on encounters where the gradient field
    // cancels by symmetry, producing no lateral avoidance.
    std::uniform_real_distribution<double> perturbDist(-0.05, 0.05);

    // Storage for per-sample combined probability and gradient
    struct Sample {
        double T;
        STP R; // trajectory point in agent-centric space-time
        double PTotal;
        STP GradTotal;
    };
    std::vector<Sample> samples(static_cast<size_t>(numSamples));

    for(int i = 0; i < numSamples; ++i) {
        const double t = i * dtSample;
        const double lateralPerturbation = perturbDist(rng);
        samples[static_cast<size_t>(i)] =
            Sample{t, STP{speed * t, lateralPerturbation, t}, 0.0, STP{0, 0, 0}};
    }

    for(const auto& neighbor : neighbors) {
        if(neighbor.AgentID == ped.AgentID) {
            continue;
        }

        const auto* nbData = std::get_if<WarpDriverModelData>(&neighbor.Model);
        if(!nbData) {
            continue;
        }

        // Neighbor orientation
        Point nbOrient = neighbor.Orientation;
        if(nbOrient.Norm() < 1e-9) {
            nbOrient = Point{1.0, 0.0};
        } else {
            nbOrient = nbOrient.Normalized();
        }

        // Neighbor speed (from v0)
        const double nbSpeed = nbData->V0;

        // TODO(perf): WarpParams and all neighbor-derived constants (orientation,
        // speed, Minkowski radius, rotation matrix cos_ab/sin_ab used in the
        // gradient inverse) are loop-invariant w.r.t. the sample index. Hoist
        // them out of the sample loop and precompute once per (ped, neighbor).
        WarpParams wp{};
        wp.PosA = ped.Pos;
        wp.OrientA = effectiveOrient;
        wp.PosB = neighbor.Pos;
        wp.OrientB = nbOrient;
        wp.SpeedB = nbSpeed;
        wp.RadiusB = agentData.Radius + nbData->Radius; // Minkowski sum
        wp.Lambda = timeUncertainty;
        wp.VelocityUncertaintyX = velocityUncertaintyX;
        wp.VelocityUncertaintyY = velocityUncertaintyY;
        wp.TimeHorizon = timeHorizon;

        for(auto& s : samples) {
            // Forward warp sample point to neighbor's Intrinsic Field space
            STP warped = ComposeForward(s.R, wp);

            // Time validity check: must be in [0, 1] (normalized)
            if(warped.T < 0.0 || warped.T > 1.0) {
                continue;
            }

            // Lookup Intrinsic Field (2D) and apply probability scaling (B.5, B.14)
            auto [intrinsicP, gradI] = intrinsicField.Sample(warped.X, warped.Y);
            const double pB = intrinsicP * ProbabilityScale(s.R, wp);

            if(pB < 1e-12) {
                continue;
            }

            // Transform gradient back to agent's frame
            STP gradB = ComposeGradientInverse(gradI, s.R, wp);

            // Union formula: p_new = p + pB - p * pB
            double pOld = s.PTotal;
            s.PTotal = pOld + pB - pOld * pB;
            s.GradTotal.X = s.GradTotal.X + gradB.X - pOld * gradB.X - pB * s.GradTotal.X;
            s.GradTotal.Y = s.GradTotal.Y + gradB.Y - pOld * gradB.Y - pB * s.GradTotal.Y;
            s.GradTotal.T = s.GradTotal.T + gradB.T - pOld * gradB.T - pB * s.GradTotal.T;
        }
    }

    // === Step 3: Solve - gradient descent on trajectory ===
    // Integrate N, P, G, S per Eq. 4-7
    double N = 0.0;
    double P = 0.0;
    STP G{0, 0, 0};
    STP S{0, 0, 0};

    for(const auto& s : samples) {
        N += s.PTotal * dtSample;
        P += s.PTotal * s.PTotal * dtSample;
        G.X += s.PTotal * s.GradTotal.X * dtSample;
        G.Y += s.PTotal * s.GradTotal.Y * dtSample;
        G.T += s.PTotal * s.GradTotal.T * dtSample;
        S.X += s.PTotal * s.R.X * dtSample;
        S.Y += s.PTotal * s.R.Y * dtSample;
        S.T += s.PTotal * s.R.T * dtSample;
    }

    Point newVelLocal;

    if(N < 1e-9) {
        // No collision risk — follow projected trajectory
        newVelLocal = Point{speed, 0.0};
    } else {
        P /= N;
        G.X /= N;
        G.Y /= N;
        G.T /= N;
        S.X /= N;
        S.Y /= N;
        S.T /= N;

        // q = S - alpha * P * G  (Eq. 8)
        STP q{};
        q.X = S.X - stepSize * P * G.X;
        q.Y = S.Y - stepSize * P * G.Y;
        q.T = S.T - stepSize * P * G.T;

        if(q.T > 1e-9) {
            newVelLocal = Point{q.X / q.T, q.Y / q.T};
        } else {
            newVelLocal = Point{speed, 0.0};
        }
    }

    // Clamp speed to [0, v0]
    const double newSpeed = std::min(newVelLocal.Norm(), agentData.V0);

    // Convert to world coordinates: rotate by effectiveOrient
    Point newVelWorld;
    if(newSpeed > 1e-9) {
        Point newDirLocal = newVelLocal.Normalized();
        // Rotate from agent-centric to world
        newVelWorld =
            Point{
                effectiveOrient.X * newDirLocal.X - effectiveOrient.Y * newDirLocal.Y,
                effectiveOrient.Y * newDirLocal.X + effectiveOrient.X * newDirLocal.Y} *
            newSpeed;
    } else {
        newVelWorld = desiredDir * agentData.V0 * 0.01; // tiny push towards goal
    }

    // Agent repulsion
    newVelWorld = newVelWorld + repulsion;

    // Boundary avoidance: steer agents away from walls
    const auto& walls = geometry.LineSegmentsInApproxDistanceTo(ped.Pos);
    for(const auto& wall : walls) {
        const Point wallVec = wall.P2 - wall.P1;
        const double wallLen2 = wallVec.ScalarProduct(wallVec);
        if(wallLen2 < 1e-12) {
            continue; // degenerate wall segment
        }
        const Point toAgent = ped.Pos - wall.P1;
        const double t = std::clamp(toAgent.ScalarProduct(wallVec) / wallLen2, 0.0, 1.0);
        const Point closest = wall.P1 + wallVec * t;
        const Point diff = ped.Pos - closest;
        const double dist = diff.Norm();
        if(dist < agentData.Radius * 3.0 && dist > 1e-6) {
            const double steering = agentData.V0 * (agentData.Radius * 3.0 - dist) / dist;
            newVelWorld = newVelWorld + diff.Normalized() * steering;
        }
    }

    // Re-clamp speed to v0 after wall steering
    double finalSpeed = newVelWorld.Norm();
    if(finalSpeed > agentData.V0 && finalSpeed > 1e-9) {
        newVelWorld = newVelWorld * (agentData.V0 / finalSpeed);
        finalSpeed = agentData.V0;
    }

    // Stuck detection: measure net displacement from an anchor position over a
    // time window. Catches oscillating agents that periodically spike above the
    // speed threshold but make no real progress.
    double stuckTime = agentData.StuckTime;
    double anchorX = agentData.AnchorX;
    double anchorY = agentData.AnchorY;
    double detourTime = agentData.DetourTime;
    int detourSide = agentData.DetourSide;

    // Detour mode: agent is currently on a lateral detour to break a deadlock
    if(detourTime > 0.0) {
        detourTime -= dT;
        Point lateral{-desiredDir.Y * detourSide, desiredDir.X * detourSide};
        Point detourDir = (lateral * 0.8 + desiredDir * 0.2).Normalized();
        Point detourVel = detourDir * agentData.V0 * 0.5;
        Point newPos = ped.Pos + detourVel * dT;
        // If detour would leave the walkable area, try the other side
        if(!geometry.InsideGeometry(newPos)) {
            detourSide = -detourSide;
            lateral = Point{-desiredDir.Y * detourSide, desiredDir.X * detourSide};
            detourDir = (lateral * 0.8 + desiredDir * 0.2).Normalized();
            detourVel = detourDir * agentData.V0 * 0.5;
            newPos = ped.Pos + detourVel * dT;
            // If both sides fail, just creep toward goal
            if(!geometry.InsideGeometry(newPos)) {
                newPos = ped.Pos + desiredDir * agentData.V0 * 0.1 * dT;
                detourDir = desiredDir;
            }
        }
        if(detourTime <= 0.0) {
            detourTime = 0.0;
            stuckTime = 0.0;
            anchorX = newPos.X;
            anchorY = newPos.Y;
        }
        return WarpDriverModelUpdate{
            newPos, detourDir, stuckTime, anchorX, anchorY, detourTime, detourSide};
    }

    // Measure net displacement from anchor over the stuck window
    constexpr double stuckThreshold = 5.0; // seconds before triggering detour
    constexpr double detourDuration = 1.0; // seconds of lateral movement
    constexpr double progressRadius = 0.3; // must move this far from anchor to count as progress

    stuckTime += dT;
    const double netDisplacement = std::hypot(ped.Pos.X - anchorX, ped.Pos.Y - anchorY);

    if(netDisplacement > progressRadius) {
        // Real progress — reset anchor to current position
        stuckTime = 0.0;
        anchorX = ped.Pos.X;
        anchorY = ped.Pos.Y;
    } else if(stuckTime >= stuckThreshold) {
        // Stuck: no net progress for stuckThreshold seconds — enter detour
        std::uniform_int_distribution<int> sideDist(0, 1);
        detourSide = sideDist(rng) * 2 - 1; // -1 or +1
        detourTime = detourDuration;
        stuckTime = 0.0;
    }

    // Velocity smoothing: blend new velocity with previous orientation to damp
    // oscillations in dense clusters where agents flip direction every frame.
    const double smoothing = 0.5; // weight of new velocity (1.0 = no smoothing)
    Point smoothedVel = newVelWorld * smoothing + orient * (newVelWorld.Norm() * (1.0 - smoothing));
    double smoothedSpeed = smoothedVel.Norm();
    if(smoothedSpeed > agentData.V0 && smoothedSpeed > 1e-9) {
        smoothedVel = smoothedVel * (agentData.V0 / smoothedSpeed);
    }

    Point newPos = ped.Pos + smoothedVel * dT;
    Point newOrient = (smoothedVel.Norm() > 1e-9) ? smoothedVel.Normalized() : orient;

    return WarpDriverModelUpdate{
        newPos, newOrient, stuckTime, anchorX, anchorY, detourTime, detourSide};
}
