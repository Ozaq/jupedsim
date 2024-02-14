// Copyright © 2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "path.hpp"

Path::Path()
{
    glGenVertexArrays(1, &vao);
    glGenBuffers(2, buffer);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, buffer[0]);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_count * sizeof(glm::vec2), nullptr, GL_STATIC_DRAW);
}

Path::~Path()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(2, buffer);
}

void Path::Update(const std::vector<glm::vec2>& coordinates)
{
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, buffer[0]);
    glBufferData(
        GL_ARRAY_BUFFER,
        coordinates.size() * sizeof(glm::vec2),
        reinterpret_cast<const GLvoid*>(coordinates.data()),
        GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
    glEnableVertexAttribArray(0);

    std::vector<size_t> indices{};
    if(coordinates.size() > 1) {
        indices.reserve(coordinates.size() - 1 * 2);
        for(size_t index = 1; index < coordinates.size(); ++index) {
            indices.emplace_back(index - 1);
            indices.emplace_back(index);
        }
    }
    index_count = indices.size();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer[1]);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        index_count * sizeof(size_t),
        reinterpret_cast<const GLvoid*>(indices.data()),
        GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Path::Draw(Shader& shader) const
{
    shader.Activate();

    shader.SetUniform("color", glm::vec4(1.0f, 1.0f, 0.75f, 1.0f));

    glBindVertexArray(vao);
    glDrawElements(GL_LINES, index_count, GL_UNSIGNED_SHORT, nullptr);
}
