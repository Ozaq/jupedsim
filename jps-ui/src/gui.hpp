// Copyright © 2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "app_state.hpp"
#include "path.hpp"
#include "rendering_mesh.hpp"
#include "wkt.hpp"

#include <filesystem>
#include <searchinstance.h>

#include <imgui.h>
#include <memory>

class Gui
{
private:
    std::optional<std::filesystem::path> wkt_path{};
    bool should_exit = false;
    bool should_recenter = false;

public:
    Gui() = default;
    ~Gui() = default;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    void Draw(const AppState& state);
    bool ShouldExit() const { return should_exit; }
    bool UpdateGeometry() const { return wkt_path.has_value(); }
    bool RecenterOnGeometry() const { return should_recenter; }
    const std::filesystem::path& WktPath() const { return wkt_path.value(); }
};
