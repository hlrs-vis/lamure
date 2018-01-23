#version 440 core

in vec2 texture_coord;
flat in uint max_level;
flat in uint toggle_view;
flat in uvec2 index_texture_dim;
flat in uvec2 physical_texture_dim;

flat in vec2 tile_size;
flat in vec2 tile_padding;

uniform float time;
uniform uvec2 resolution;
uniform float scale;

layout(std430, binding = 0) buffer out_feedback_ssbo { uint[] out_feedback_values; };

layout(binding = 0) uniform sampler2DArray physical_texture_array;
layout(binding = 1) uniform usampler2D index_texture;

layout(location = 0) out vec4 out_color;

// Written by GLtracy

// math const
const float PI = 3.14159265359;
const float DEG_TO_RAD = PI / 180.0;
const float MAX = 10000.0;

// scatter const
const float K_R = 0.166;
const float K_M = 0.0025;
const float E = 14.3;                 // light intensity
const vec3 C_R = vec3(0.3, 0.7, 1.0); // 1 / wavelength ^ 4
const float G_M = -0.85;              // Mie g

const float R = 1.0;
const float R_INNER = 0.7;
const float SCALE_H = 4.0 / (R - R_INNER);
const float SCALE_L = 1.0 / (R - R_INNER);

const int NUM_OUT_SCATTER = 10;
const float FNUM_OUT_SCATTER = 10.0;

const int NUM_IN_SCATTER = 10;
const float FNUM_IN_SCATTER = 10.0;

// angle : pitch, yaw
mat3 rot3xy(vec2 angle)
{
    vec2 c = cos(angle);
    vec2 s = sin(angle);

    return mat3(c.y, 0.0, -s.y, s.y * s.x, c.x, c.y * s.x, s.y * c.x, -s.x, c.y * c.x);
}

// ray direction
vec3 ray_dir(float fov, vec2 size, vec2 pos)
{
    vec2 xy = pos - size * 0.5;

    float cot_half_fov = tan((90.0 - fov * 0.5) * DEG_TO_RAD);
    float z = size.y * 0.5 * cot_half_fov;

    return normalize(vec3(xy, -z));
}

// ray intersects sphere
// e = -b +/- sqrt( b^2 - c )
vec2 ray_vs_sphere(vec3 p, vec3 dir, float r)
{
    float b = dot(p, dir);
    float c = dot(p, p) - r * r;

    float d = b * b - c;
    if(d < 0.0)
    {
        return vec2(MAX, -MAX);
    }
    d = sqrt(d);

    return vec2(-b - d, -b + d);
}

// Mie
// g : ( -0.75, -0.999 )
//      3 * ( 1 - g^2 )               1 + c^2
// F = ----------------- * -------------------------------
//      2 * ( 2 + g^2 )     ( 1 + g^2 - 2 * g * c )^(3/2)
float phase_mie(float g, float c, float cc)
{
    float gg = g * g;

    float a = (1.0 - gg) * (1.0 + cc);

    float b = 1.0 + gg - 2.0 * g * c;
    b *= sqrt(b);
    b *= 2.0 + gg;

    return 1.5 * a / b;
}

// Reyleigh
// g : 0
// F = 3/4 * ( 1 + c^2 )
float phase_reyleigh(float cc) { return 0.75 * (1.0 + cc); }

float density(vec3 p) { return exp(-(length(p) - R_INNER) * SCALE_H); }

float optic(vec3 p, vec3 q)
{
    vec3 step = (q - p) / FNUM_OUT_SCATTER;
    vec3 v = p + step * 0.5;

    float sum = 0.0;
    for(int i = 0; i < NUM_OUT_SCATTER; i++)
    {
        sum += density(v);
        v += step;
    }
    sum *= length(step) * SCALE_L;

    return sum;
}

vec3 in_scatter(vec3 o, vec3 dir, vec2 e, vec3 l)
{
    float len = (e.y - e.x) / FNUM_IN_SCATTER;
    vec3 step = dir * len;
    vec3 p = o + dir * e.x;
    vec3 v = p + dir * (len * 0.5);

    vec3 sum = vec3(0.0);
    for(int i = 0; i < NUM_IN_SCATTER; i++)
    {
        vec2 f = ray_vs_sphere(v, l, R);
        vec3 u = v + l * f.y;

        float n = (optic(p, v) + optic(v, u)) * (PI * 4.0);

        sum += density(v) * exp(-n * (K_R * C_R + K_M));

        v += step;
    }
    sum *= len * SCALE_L;

    float c = dot(dir, -l);
    float cc = c * c;

    return sum * (K_R * C_R * phase_reyleigh(cc) + K_M * phase_mie(G_M, c, cc)) * E;
}

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    // default ray dir
    vec3 dir = ray_dir(45.0, resolution.xy, fragCoord.xy);

    // default ray origin
    vec3 eye = vec3(0.0, 0.0, 2.4);

    // rotate camera
    mat3 rot = rot3xy(vec2(0.0, time * 0.5));
    dir = rot * dir;
    eye = rot * eye;

    // sun light dir
    vec3 l = vec3(0, 0, 1);

    vec2 e = ray_vs_sphere(eye, dir, R);
    if(e.x > e.y)
    {
        discard;
    }

    vec2 f = ray_vs_sphere(eye, dir, R_INNER);
    e.y = min(e.y, f.x);

    vec3 I = in_scatter(eye, dir, e, l);

    fragColor = vec4(I, 1.0);
}

void main()
{
    // swap y axis
    vec3 texture_coordinates = vec3(texture_coord, 0.0);
    texture_coordinates.y = 1.0 - texture_coordinates.y;

    uvec4 index_quadruple = texture(index_texture, texture_coordinates.xy).rgba;
    texture_coordinates.z = index_quadruple.a;
    vec4 c;

    uint reference_count = 0;
    if(toggle_view == 0)
    { // Show the physical texture
        c = texture(physical_texture_array, texture_coordinates);
        out_color = c;
    }
    else
    { // Show the image viewer

        uint current_level = index_quadruple.z;

        // exponent for calculating the occupied pixels in our index texture, based on which level the tile is in
        uint tile_occupation_exponent = max_level - current_level;

        // 2^tile_occupation_exponent defines how many pixel (of the index texture) are used by the given tile
        uint occupied_index_pixel_per_dimension = uint(1 << tile_occupation_exponent);

        // offset represented as tiles is divided by total num tiles per axis
        // (replace max_width_tiles later by correct uniform)
        // extracting x,y from index texture
        uvec2 base_xy_offset = index_quadruple.xy;

        // just to be conformant to the modf interface (integer parts are ignored)
        vec2 dummy;

        // base x,y coordinates * number of tiles / number of used index texture pixel
        // taking the factional part by modf
        vec2 physical_tile_ratio_xy = modf((texture_coordinates.xy * index_texture_dim / vec2(occupied_index_pixel_per_dimension)), dummy);

        // Use only tile_size - 2*tile_padding pixels to render scene
        // Therefore, scale reduced tile size to full size and translate it
        vec2 padding_scale = 1 - 2 * tile_padding / tile_size;
        vec2 padding_offset = tile_padding / tile_size;

        // adding the ratio for every texel to our base offset to get the right pixel in our tile
        // and dividing it by the dimension of the phy. tex.
        vec2 physical_texture_coordinates = (base_xy_offset.xy + physical_tile_ratio_xy * padding_scale + padding_offset) / physical_texture_dim;

        // c = vec4(physical_tile_ratio_xy, 0.0, 1.0);

        // outputting the calculated coordinate from our physical texture
        c = texture(physical_texture_array, vec3(physical_texture_coordinates, texture_coordinates.z));

        // feedback calculation based on accumulated use of each rendered tile
        // TODO: maybe a bug here: take level of physical texture into account
        uint one_d_feedback_ssbo_index = base_xy_offset.x + base_xy_offset.y * physical_texture_dim.x;
        reference_count = atomicAdd(out_feedback_values[one_d_feedback_ssbo_index], 1);
        // reference_count += 1;

        // c = vec4( float(reference_count) / (10*375542.857 / float( (current_level+1) * (current_level+1) )), (float(reference_count) / (10*375542.857 / float(current_level+1) * (current_level+1) ))
        // * 0.3, 0.0, 1.0 );

        /*if(texture_coordinates.z == 0) {
            c = vec4(1.0, 0.0, 0.0, 1.0);
        }*/
    }

    out_color = c;

//    vec4 atmospheric_glow = vec4(1., 1., 1., 1.);
//    mainImage(atmospheric_glow, ((vec2(0.5, 0.5) - gl_FragCoord.xy / resolution.xy) * (1.2 - scale) + vec2(0.5, 0.5))*resolution.xy);
//
//    out_color = c * 0.5 + atmospheric_glow * 0.5;
}