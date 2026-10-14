#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;
flat out vec2 coverage;
uniform vec2 range;
uniform mat3 rot;
uniform vec3 pos;
uniform vec2 fov;
uniform vec2 angular_step;
uniform float point_spacing;

void main()
{
    const float PI = 3.14159265358;
    vec3 body = rot * (aPos - pos);
    float depth = length(body);
    ourColor = aColor;
    coverage = vec2(1.0);
    if (depth > range.y || depth < range.x) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 1.0;
        return;
    }
    gl_Position = vec4(-atan(body.y, body.x) / PI * (360.0 / fov.x),
                       atan(body.z, length(body.xy)) / PI * (360.0 / fov.y),
                       depth / range.y, 1.0);
    // GL point sprites are square in pixels, whereas the two angular steps
    // need not match. Rasterize the larger extent and clip the other axis.
    float cover_angle = degrees(asin(0.55 * 1.7321 * point_spacing / depth));
    vec2 extent = ceil(vec2(cover_angle) / angular_step);
    float half_size = max(extent.x, extent.y);
    gl_PointSize = 2.0 * half_size;
    coverage = extent / half_size;
}
