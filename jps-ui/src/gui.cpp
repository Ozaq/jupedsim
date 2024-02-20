// Copyright © 2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "gui.hpp"
#include "mesh.h"
#include "mesh.hpp"
#include "point.h"
#include "rendering_mesh.hpp"
#include "searchinstance.h"
#include "wkt.hpp"

#include <exception>
#include <imgui_internal.h>

#include <ImGuiFileDialog.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

std::string to_shortcut(const ImGuiKeyChord key)
{
    std::array<char, 32> shortcut{};
    ImGui::GetKeyChordName(key, shortcut.data(), shortcut.size());
    return {shortcut.data()};
}

void Gui::Draw(const AppState& state)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    should_recenter = false;
    wkt_path = std::nullopt;

    if(ImGui::BeginMainMenuBar()) {
        if(ImGui::BeginMenu("Menu")) {
            static const auto open_shortcut = to_shortcut(ImGuiMod_Shortcut | ImGuiKey_O);
            if(ImGui::MenuItem("Open", open_shortcut.c_str())) {
                ImGuiFileDialog::Instance()->OpenDialog(
                    "ChooseFileDlgKey",
                    "Choose File",
                    ".wkt",
                    ".",
                    1,
                    nullptr,
                    ImGuiFileDialogFlags_Modal);
            }
            if(ImGui::MenuItem("Center View", "C")) {
                should_recenter = true;
            }
            ImGui::Separator();
            static const auto exit_shortcut = to_shortcut(ImGuiMod_Shortcut | ImGuiKey_Q);
            if(ImGui::MenuItem("Exit", exit_shortcut.c_str())) {
                should_exit = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    if(ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_O)) {
        ImGuiFileDialog::Instance()->OpenDialog(
            "ChooseFileDlgKey", "Choose File", ".wkt", ".", 1, nullptr, ImGuiFileDialogFlags_Modal);
    }

    if(ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_Q)) {
        should_exit = true;
    }

    if(ImGui::IsKeyChordPressed(ImGuiKey_C)) {
        should_recenter = true;
    }

    if(ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey")) {
        if(ImGuiFileDialog::Instance()->IsOk()) {
            std::filesystem::path filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
            std::filesystem::path filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
            wkt_path = filePath / filePathName;
        }
        ImGuiFileDialog::Instance()->Close();
    }

    ImGui::Text(
        "View_Projection Matrix\n %f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f",
        state.cam->ViewProjection()[0][0],
        state.cam->ViewProjection()[0][1],
        state.cam->ViewProjection()[0][2],
        state.cam->ViewProjection()[0][3],
        state.cam->ViewProjection()[1][0],
        state.cam->ViewProjection()[1][1],
        state.cam->ViewProjection()[1][2],
        state.cam->ViewProjection()[1][3],
        state.cam->ViewProjection()[2][0],
        state.cam->ViewProjection()[2][1],
        state.cam->ViewProjection()[2][2],
        state.cam->ViewProjection()[2][3],
        state.cam->ViewProjection()[3][0],
        state.cam->ViewProjection()[3][1],
        state.cam->ViewProjection()[3][2],
        state.cam->ViewProjection()[3][3]);
    ImGui::Text("Clicked @ {%f, %f}", state.clicked_pos.x, state.clicked_pos.y);
    if(state.from) {
        ImGui::Text("From {%f, %f}", state.from->x, state.from->y);
    } else {
        ImGui::Text("From { -, -}");
    }
    if(state.to) {
        ImGui::Text("To {%f, %f}", state.to->x, state.to->y);
    } else {
        ImGui::Text("To { -, -}");
    }
    ImGui::Begin("Mesh Description");
    ImGui::TextUnformatted(state.mesh_text.c_str());
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
