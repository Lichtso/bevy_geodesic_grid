#import bevy_render::maths::{mat4x4_to_mat3x3, PI}
#import bevy_pbr::{
    mesh_bindings::mesh,
    mesh_view_bindings::view,
    pbr_types::{PbrInput, pbr_input_new},
}
#import ellipsoid_billboard

@group(#{MATERIAL_BIND_GROUP}) @binding(0) var tile_texture: texture_2d<u32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(1) var selection_texture: texture_2d<u32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(2) var palette_texture: texture_2d<f32>;

@vertex
fn vertex(vertex: ellipsoid_billboard::Vertex) -> ellipsoid_billboard::VertexOutput {
    return ellipsoid_billboard::vertex(vertex);
}

const ICOSAHEDRON_RADIUS_BY_EDGE_LENGTH = sin(PI * 2.0 / 5.0);
const INVERSESQRT5 = inverseSqrt(5.0);
const ICOSAHEDRON_VERTICES = array<vec3<f32>, 12>(
    vec3(0.0, -1.0, 0.0),
    vec3( 0.0               , -INVERSESQRT5, -0.8944271802902222 ),
    vec3(-0.8506507873535156, -INVERSESQRT5, -0.27639320492744446),
    vec3(-0.525731086730957 , -INVERSESQRT5,  0.7236068248748779 ),
    vec3( 0.525731086730957 , -INVERSESQRT5,  0.7236068248748779 ),
    vec3( 0.8506507873535156, -INVERSESQRT5, -0.27639320492744446),
    vec3(-0.525731086730957 ,  INVERSESQRT5, -0.7236068248748779 ),
    vec3(-0.8506507873535156,  INVERSESQRT5,  0.27639320492744446),
    vec3( 0.0               ,  INVERSESQRT5,  0.8944271802902222 ),
    vec3( 0.8506507873535156,  INVERSESQRT5,  0.27639320492744446),
    vec3( 0.525731086730957 ,  INVERSESQRT5, -0.7236068248748779 ),
    vec3(0.0, 1.0, 0.0),
);
/*const ICOSAHEDRON_POLE_INDICES = array<array<vec3<i32>, 5>, 4>(
    array<vec3<i32>, 5>(
        vec3<i32>(2, 1, 0),
        vec3<i32>(3, 2, 0),
        vec3<i32>(4, 3, 0),
        vec3<i32>(5, 4, 0),
        vec3<i32>(1, 5, 0),
    ),
    array<vec3<i32>, 5>(
        vec3<i32>(1, 2, 6),
        vec3<i32>(2, 3, 7),
        vec3<i32>(3, 4, 8),
        vec3<i32>(4, 5, 9),
        vec3<i32>(5, 1, 10),
    ),
    array<vec3<i32>, 5>(
        vec3<i32>(6, 10, 1),
        vec3<i32>(7, 6, 2),
        vec3<i32>(8, 7, 3),
        vec3<i32>(9, 8, 4),
        vec3<i32>(10, 9, 5),
    ),
    array<vec3<i32>, 5>(
        vec3<i32>(10, 6, 11),
        vec3<i32>(6, 7, 11),
        vec3<i32>(7, 8, 11),
        vec3<i32>(8, 9, 11),
        vec3<i32>(9, 10, 11),
    ),
);*/

fn side_of_half_plane(point: vec3<f32>, pole_index_a: i32, pole_index_b: i32) -> bool {
    let aux_a = cross(ICOSAHEDRON_VERTICES[pole_index_a], ICOSAHEDRON_VERTICES[pole_index_b]);
    return dot(point, aux_a) > 0.0;
}

fn into_cartesian(barycentric: vec3<f32>, pole_indices: vec3<i32>) -> vec3<f32> {
    var planes: array<vec4<f32>, 3>;
    for(var i = 0; i < 3; i += 1) {
        let b = ICOSAHEDRON_VERTICES[pole_indices[(i + 1) % 3]];
        let c = ICOSAHEDRON_VERTICES[pole_indices[(i + 2) % 3]];
        let tangent = tan(PI / 10.0 * barycentric[i]);
        let normal = cross(b, c) - tangent * (b + c);
        planes[i].x = tangent * (1.0 + INVERSESQRT5);
        planes[i].y = normal.x;
        planes[i].z = normal.y;
        planes[i].w = normal.z;
    }
    let l_g0 = planes[0].x * planes[1].yzw - planes[0].yzw * planes[1].x;
    let l_g1 = planes[0].z * vec3(planes[1].w, 0.0, -planes[1].y) + planes[0].w * vec3(-planes[1].z, planes[1].y, 0.0) + planes[0].y * vec3(0.0, -planes[1].w, planes[1].z);
    let p = l_g0.y * vec4(0.0, -planes[2].w, 0.0, planes[2].y) + l_g0.z * vec4(0.0, planes[2].z, -planes[2].y, 0.0) + l_g1.y * vec4(planes[2].z, 0.0, -planes[2].x, 0.0) + l_g1.z * vec4(planes[2].w, 0.0, 0.0, -planes[2].x) + vec4(l_g1.x, -l_g1.x, l_g0.x, -l_g0.x) * planes[2].yxwz;
    return -p.yzw / p.x;
}

fn from_cartesian(cartesian: vec3<f32>, triangle_latitude: ptr<function, i32>, triangle_longitude: ptr<function, i32>, pole_indices: ptr<function, vec3<i32>>) -> vec3<f32> {
    // Polar coordinates
    let latitude = 0.5 + 0.5 * atan2(cartesian.x, cartesian.z) / PI;
    /* let longitude = 1.0 - acos(cartesian.y) / PI; */
    let is_nothern_hemisphere = cartesian.y > 0.0;
    let latitude_sector = i32(latitude * 10.0);
    // Icosahedron triangle
    *triangle_latitude = (latitude_sector + i32(is_nothern_hemisphere)) >> 1; // / 2;
    *triangle_latitude -= i32(*triangle_latitude >= 5) * 5; // *triangle_latitude %= 5;
    *pole_indices = vec3<i32>(*triangle_latitude + select(1, 4, is_nothern_hemisphere), 1 + *triangle_latitude, 0);
    *triangle_longitude = 0;
    (*pole_indices).x -= i32((*pole_indices).x >= 5) * 5; // pole_indices.x %= 5;
    (*pole_indices).x += 1;
    if is_nothern_hemisphere {
        *pole_indices += vec3(5, 5, 11);
        *triangle_longitude = 3;
    }
    /* *triangle_longitude = select(0, 3, is_nothern_hemisphere);
    *pole_indices = ICOSAHEDRON_POLE_INDICES[*triangle_longitude][*triangle_latitude]; */
    if side_of_half_plane(cartesian, (*pole_indices).y, (*pole_indices).x) {
        *triangle_latitude = latitude_sector >> 1; // / 2;
        *triangle_longitude = 1;
        *pole_indices = vec3(1 + *triangle_latitude, *triangle_latitude + 1, 6 + *triangle_latitude);
        (*pole_indices).y -= i32((*pole_indices).y >= 5) * 5; // (*pole_indices).y %= 5;
        (*pole_indices).y += 1;
        /* pole_indices = ICOSAHEDRON_POLE_INDICES[*triangle_longitude][*triangle_latitude]; */
        let border_indices = select((*pole_indices).zy, (*pole_indices).xz, (latitude_sector & 1) == 0);
        if side_of_half_plane(cartesian, border_indices.x, border_indices.y) {
            *triangle_latitude = (latitude_sector + 1) >> 1; // / 2;
            *triangle_latitude -= i32(*triangle_latitude >= 5) * 5; // *triangle_latitude %= 5;
            *triangle_longitude = 2;
            *pole_indices = vec3(6 + *triangle_latitude, *triangle_latitude + 4, 1 + *triangle_latitude);
            (*pole_indices).y -= i32((*pole_indices).y >= 5) * 5; // (*pole_indices).y %= 5;
            (*pole_indices).y += 6;
            /* pole_indices = ICOSAHEDRON_POLE_INDICES[*triangle_longitude][*triangle_latitude]; */
        }
    }
    // Cartesian to barycentric
    var bary: vec3<f32>;
    for(var i = 0; i < 3; i += 1) {
        let b = ICOSAHEDRON_VERTICES[(*pole_indices)[(i + 1) % 3]];
        let c = ICOSAHEDRON_VERTICES[(*pole_indices)[(i + 2) % 3]];
        let numerator = dot(cartesian, cross(b, c));
        let denominator = 1.0 + INVERSESQRT5 + dot(cartesian, b) + dot(cartesian, c);
        bary[i] = atan2(numerator, denominator) * 10.0 / PI;
    }
    // bary /= dot(vec3(1.0), bary);
    return bary;
}

fn into_tile(floored: vec3<i32>, triangle_latitude: i32, triangle_longitude: i32, gp_index: i32) -> vec2<i32> {    
    var tile = vec2(floored.x - floored.y - floored.z, floored.y);
    tile = select(
        vec2(-tile.x, gp_index - 1 - tile.y),
        vec2(tile.x - 1, tile.y),
        (triangle_longitude & 1) == 0
    );
    tile += vec2(triangle_latitude * 2 + 1, triangle_longitude / 2) * gp_index;
    return tile;
}

fn from_tile(tile: vec2<i32>, triangle_latitude: ptr<function, i32>, triangle_longitude: ptr<function, i32>, gp_index: i32) -> vec3<f32> {
    let double_gp_index = gp_index * 2;
    let diagonal = tile.x + tile.y * 2 + 1;
    *triangle_latitude = tile.x / double_gp_index;
    *triangle_longitude = diagonal / double_gp_index + tile.y / gp_index - *triangle_latitude;
    let triangle_latitude_double_gp_index = *triangle_latitude * double_gp_index;
    var barycentric = vec3<f32>(vec3(
        tile.x - triangle_latitude_double_gp_index,
        0,
        diagonal + 1 - triangle_latitude_double_gp_index - *triangle_longitude * gp_index,
    )) * 0.5;
    let float_gp_index = f32(gp_index);
    if (*triangle_longitude & 1) == 0 {
        barycentric.z = float_gp_index - barycentric.z;
    } else {
        barycentric.x = float_gp_index - barycentric.x - 0.5;
        barycentric.z = -0.5 * float_gp_index + barycentric.z - 0.5;
    }
    barycentric.y = float_gp_index - barycentric.x - barycentric.z;
    barycentric += select(
        vec3(0.0, -0.5, 0.0) + vec3(1.0 / 6.0),
        vec3(0.5, -0.5, 0.5) - vec3(1.0 / 6.0),
        (tile.x & 1) == (*triangle_longitude & 1),
    );
    return barycentric;
}

fn tile_midpoint_offset(barycentric: ptr<function, vec3<f32>>, floored: vec3<i32>) -> vec3<f32> {
    let is_flipped = ((floored.x + floored.y + floored.z) & 1) == 1;
    let midpoint = vec3<f32>(floored) + select(vec3(2.0 / 3.0), vec3(1.0 / 3.0), is_flipped);
    *barycentric = (*barycentric - midpoint) * select(3.0, -3.0, is_flipped);
    return midpoint;
}

@fragment
fn fragment(vertex_output: ellipsoid_billboard::VertexOutput) -> ellipsoid_billboard::FragmentOutput {
    var cartesian: vec3<f32>;
    var world_position: vec3<f32>;
    var world_normal: vec3<f32>;
    var fragment_output: ellipsoid_billboard::FragmentOutput;
    fragment_output.frag_depth = ellipsoid_billboard::fragment_ray_tracing(vertex_output, &cartesian, &world_position, &world_normal);
    /* fragment_output.color = vec4(world_normal, 1.0);
    return fragment_output;*/

    #ifndef PREPASS_PIPELINE
    /* fragment_output.color = vec4(f32(triangle_latitude) / 4.0, f32(triangle_longitude) / 3.0, 0.0, 1.0); */

    // Barycentric coordinates
    var triangle_latitude: i32;
    var triangle_longitude: i32;
    var pole_indices: vec3<i32>;
    var barycentric = from_cartesian(cartesian, &triangle_latitude, &triangle_longitude, &pole_indices);
    const gp_index = 128;
    const mip_level = 0u;
    // gp_index /= (1 << mip_level);
    barycentric *= f32(gp_index);
    let floored = vec3<i32>(barycentric);
    let barycentric_midpoint = tile_midpoint_offset(&barycentric, floored);
    let tile = into_tile(floored, triangle_latitude, triangle_longitude, gp_index);
    // let barycentric_midpoint = from_tile(tile, &triangle_latitude, &triangle_longitude, gp_index);
    // fragment_output.color = vec4(f32(tile.x) / f32(gp_index * 10), f32(tile.y) / f32(gp_index * 4), 0.0, 1.0);
    // fragment_output.color = vec4(barycentric_midpoint / f32(gp_index), 1.0);
    // fragment_output.color = vec4(barycentric / f32(gp_index), 1.0);
    let tile_kind = textureLoad(tile_texture, tile, mip_level).x;
    let color = textureLoad(palette_texture, vec2(tile_kind, 0), 0).rgb;
    let metallic_roughness_emission = textureLoad(palette_texture, vec2(tile_kind, 1), 0).rgb;
    // fragment_output.color = vec4(0.0, f32(textureLoad(tile_texture, tile, 0).x) / 4.0, 0.0, 1.0);
    /*let neighboor = textureLoad(selection_texture, tile, 0).x;
    if neighboor > 0 {
        fragment_output.color = vec4(0.0, 0.0, 0.0, 1.0);
        fragment_output.color[neighboor - 1] = 1.0;
    }*/

    /*let is_flipped = ((floored.x + floored.y + floored.z) & 1) == 1;
    /*let cart = into_cartesian((barycentric_midpoint + vec3(-2.0, 1.0, 1.0) / select(3.0, -3.0, is_flipped)) / f32(gp_index), pole_indices);
    fragment_output.color = vec4(vec3(step(length(cart - cartesian), 0.01)), 1.0);*/
    let plane = normalize(vec3(1.0, -2.0, 3.0));
    var dots = vec3(0.0);
    for(var i = 0; i < 3; i += 1) {
        var offset = vec3(1.0);
        offset[i] = -2.0;
        let tile_corner = into_cartesian((barycentric_midpoint + offset / select(3.0, -3.0, is_flipped)) / f32(gp_index), pole_indices);
        dots[i] = dot(tile_corner, plane);
    }
    let sides = dot(step(dots, vec3(0.0)), vec3(1.0));
    let close = dot(step(abs(dots), vec3(0.002)), vec3(1.0)) * 10.0;
    fragment_output.color = vec4(vec3(step(abs(sides - 1.5), 1.0)), 1.0);*/

    // Grid lines
    var edge_distance = vec3(2.0) - barycentric;
    let selection = textureLoad(selection_texture, tile, 0).x;
    edge_distance -= vec3<f32>(vec3<u32>((selection >> 1) & 1, (selection >> 2) & 1, (selection >> 3) & 1));
    let hit_angle = acos(-dot(world_normal, view.world_from_view[2].xyz)) / PI;
    let grid_line_width = 0.5 + 0.25 * (1.0 - sqrt(hit_angle));
    let grid_line_intensity = max(0.0, 1.0 - min(edge_distance.x, min(edge_distance.y, edge_distance.z)) / grid_line_width) + f32(selection & 1) * 0.5;
    fragment_output.color = vec4(mix(color, vec3(1.0, 0.0, 0.0), grid_line_intensity), 1.0);
    #endif

    // Illumination
    var pbr_input: PbrInput = pbr_input_new();
    pbr_input.flags = mesh[vertex_output.instance_index].flags;
    pbr_input.frag_coord = vertex_output.position;
    pbr_input.world_position = vec4(world_position, 1.0);
    pbr_input.world_normal = world_normal;
    #ifndef PREPASS_PIPELINE
    pbr_input.material.metallic = metallic_roughness_emission.r;
    pbr_input.material.perceptual_roughness = metallic_roughness_emission.g;
    pbr_input.material.emissive = vec4(color, 0.0) * metallic_roughness_emission.b;
    #endif
    ellipsoid_billboard::fragment_illumination(&pbr_input, &fragment_output);
    return fragment_output;
}
