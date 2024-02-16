// Copyright © 2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "ortho_camera.hpp"
#include <glm/vec2.hpp>
#include <memory>

struct AppState {
    std::unique_ptr<OrthoCamera> cam{};
    glm::dvec2 clicked_pos{};
    std::optional<glm::dvec2> from{};
    std::optional<glm::dvec2> to{};
};
