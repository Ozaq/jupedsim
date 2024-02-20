// Copyright © 2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "app_state.hpp"
#include "gui.hpp"
#include "ortho_camera.hpp"
#include "shader.hpp"
#include "wkt.hpp"

#include <GLFW/glfw3.h>
#include <geos_c.h>
#include <glad/glad.h>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/vec3.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

using vec3 = glm::dvec3;

static void geos_msg_handler(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

const std::string vertex_shader_code = R"(
    #version 330 core
    layout (location = 0) in vec2 inPos;
    uniform mat4 model;
    uniform mat4 view_projection;
    void main()
    {
       gl_Position = view_projection * model * vec4(inPos.x, inPos.y, 0.0, 1.0);
    }
)";

const std::string fragment_shader_code = R"(
    #version 330 core
    uniform vec4 color;
    out vec4 outColor;

    void main()
    {
        outColor = color;
    }
)";

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    ImGuiIO& io = ImGui::GetIO();
    io.AddMouseButtonEvent(button, action == GLFW_PRESS);

    if(!io.WantCaptureMouse && action == GLFW_RELEASE) {
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glm::dvec2 pos{};
        glfwGetCursorPos(window, &pos.x, &pos.y);
        pos.x = (2.0 * pos.x) / width - 1.0;
        pos.y = 1.0 - (2.0 * pos.y) / height;
        auto state = reinterpret_cast<AppState*>(glfwGetWindowUserPointer(window));
        state->clicked_pos = state->cam->ViewportToXYPlane(pos);

        if(!state->from || state->to) {
            state->to = std::nullopt;
            state->from = state->clicked_pos;
        } else {
            state->to = state->clicked_pos;
        }
    }
}

int main(int argc, char** argv)
{
    glfwSetErrorCallback(glfw_error_callback);
    initGEOS(geos_msg_handler, geos_msg_handler);
    GLFWwindow* window;

    /* Initialize the library */
    if(!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    window = glfwCreateWindow(640, 480, "Mesh Viewer", NULL, NULL);

    if(!window) {
        glfwTerminate();
        return 1;
    }
    AppState state{};

    glfwSetWindowUserPointer(window, reinterpret_cast<void*>(&state));
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    if(!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        return -1;
    }
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    glfwSetWindowFocusCallback(window, ImGui_ImplGlfw_WindowFocusCallback);
    glfwSetCursorEnterCallback(window, ImGui_ImplGlfw_CursorEnterCallback);
    glfwSetCursorPosCallback(window, ImGui_ImplGlfw_CursorPosCallback);
    // glfwSetMouseButtonCallback(window, ImGui_ImplGlfw_MouseButtonCallback);
    glfwSetScrollCallback(window, ImGui_ImplGlfw_ScrollCallback);
    glfwSetKeyCallback(window, ImGui_ImplGlfw_KeyCallback);
    glfwSetCharCallback(window, ImGui_ImplGlfw_CharCallback);
    glfwSetMonitorCallback(ImGui_ImplGlfw_MonitorCallback);
    ImGui_ImplOpenGL3_Init(glsl_version);

    state.cam = std::make_unique<OrthoCamera>();
    Shader shader(vertex_shader_code, fragment_shader_code);
    shader.Activate();

    shader.SetUniform("model", glm::mat4x4(1.0f));

    Gui gui{};
    std::unique_ptr<DrawableGEOS> geo{nullptr};
    std::unique_ptr<RenderingMesh> render_mesh{nullptr};
    std::unique_ptr<polyanya::Mesh> polyanya_mesh{};
    std::unique_ptr<polyanya::SearchInstance> search{};
    Path path{};
    path.Update(std::vector<glm::vec2>{{-0.5, 0}, {0.5, 0}});

    /* Loop until the user closes the window */
    while(!glfwWindowShouldClose(window)) {
        /* Poll for and process events */
        glfwPollEvents();
        GLint display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(
            gui.clear_color.x * gui.clear_color.w,
            gui.clear_color.y * gui.clear_color.w,
            gui.clear_color.z * gui.clear_color.w,
            gui.clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        if(gui.RecenterOnGeometry() && geo != nullptr) {
            state.cam->CenterOn(geo->Bounds());
        }
        state.cam->Update(shader);

        if(gui.UpdateGeometry()) {
            const auto wkt = read_wkt(gui.WktPath());
            if(wkt) {
                geo = std::make_unique<DrawableGEOS>(wkt);
                Mesh m(geo->tri());
                m.MergeGreedy();
                auto buf = m.intoLibPolyanyaMeshDescription();
                state.mesh_text = buf.str();
                polyanya_mesh = std::make_unique<polyanya::Mesh>(buf);
                search = std::make_unique<polyanya::SearchInstance>(polyanya_mesh.get());

                render_mesh = std::make_unique<RenderingMesh>(m);
                state.cam->CenterOn(geo->Bounds());
                state.cam->Update(shader);
            }
        }

        if(search && state.from && state.to) {
            search->set_start_goal({state.from->x, state.from->y}, {state.to->x, state.to->y});
            search->search();
            std::vector<polyanya::Point> p{};
            p.reserve(8);
            search->get_path_points(p);
            std::vector<glm::vec2> p2{};
            p2.reserve(p.size());
            std::transform(std::begin(p), std::end(p), std::back_inserter(p2), [](const auto& c) {
                return glm::vec2{c.x, c.y};
            });
            std::cout << std::flush;
            path.Update(p2);
        }
        if(!state.to.has_value()) {
            path.Update({});
        }

        if(render_mesh) {
            render_mesh->Draw(shader);
        }
        path.Draw(shader);
        gui.Draw(state);

        if(gui.ShouldExit()) {
            glfwSetWindowShouldClose(window, gui.ShouldExit());
        }

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
