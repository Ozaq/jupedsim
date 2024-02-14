// Copyright © 2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "ortho_camera.hpp"
#include "glm/matrix.hpp"

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/vec2.hpp>

#include <iostream>

OrthoCamera::OrthoCamera()
{
}

void OrthoCamera::CenterOn(AABB bounds)
{
    const float padding = 1.05; // Pad view by 5% on most constraint axis
    const float bounds_aspect = bounds.Aspect();
    if(bounds_aspect < aspect) {
        frustrum_half_width = bounds.Height() * padding / 2.0f * aspect;
    } else {
        frustrum_half_width = bounds.Width() * padding / 2.0f;
    }

    const glm::vec2 cp = bounds.Center();
    eye = {cp, 0.0f};
    center = {cp, -1.0f};
    dirty = true;
}

void OrthoCamera::ChangeViewport(float width, float height)
{
    aspect = width / height;
    dirty = true;
}

void OrthoCamera::Update(Shader& shader)
{
    if(dirty) {
        const auto view = glm::lookAt(eye, center, up);
        const float frustrum_half_height = frustrum_half_width * (1 / aspect);
        const auto projection = glm::ortho(
            -frustrum_half_width, frustrum_half_width, -frustrum_half_height, frustrum_half_height);
        view_projection = projection * view;
        view_projection_inv = glm::inverse(view_projection);
        dirty = false;
    }
    shader.SetUniform("view_projection", view_projection);
}

glm::dvec2 OrthoCamera::ViewportToXYPlane(glm::dvec2 pos) const
{
    glm::vec4 pos4{pos, 0, 1};
    const auto world_pos = view_projection_inv * pos4;
    return {world_pos.x, world_pos.y};
}
