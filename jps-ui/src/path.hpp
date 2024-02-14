// Copyright © 2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "shader.hpp"

#include <GLFW/glfw3.h>
#include <glad/glad.h>

class Path
{
    GLuint vao{};
    GLuint buffer[2]{};
    GLuint index_count{};

public:
    Path();
    ~Path();
    void Update(const std::vector<glm::vec2>& coordinates);
    void SetColor();
    void Draw(Shader& shader) const;
};
