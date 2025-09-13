use crate::{geodesic_grid::GeodesicGrid, tiles::Tile};
use bevy::prelude::*;
use std::f32::consts::PI;

// const ICOSAHEDRON_RADIUS_BY_EDGE_LENGTH: f32 = 0.9510565163; // (PI * 2.0_f32 / 5.0_f32).sin();
const INVERSESQRT5: f32 = 0.4472135955; // 1.0_f32 / (5.0_f32).sqrt();
const ICOSAHEDRON_VERTICES: [Vec3; 12] = [
    vec3(0.0, -1.0, 0.0),
    vec3(0.0, -INVERSESQRT5, -0.8944271802902222),
    vec3(-0.8506507873535156, -INVERSESQRT5, -0.27639320492744446),
    vec3(-0.525731086730957, -INVERSESQRT5, 0.7236068248748779),
    vec3(0.525731086730957, -INVERSESQRT5, 0.7236068248748779),
    vec3(0.8506507873535156, -INVERSESQRT5, -0.27639320492744446),
    vec3(-0.525731086730957, INVERSESQRT5, -0.7236068248748779),
    vec3(-0.8506507873535156, INVERSESQRT5, 0.27639320492744446),
    vec3(0.0, INVERSESQRT5, 0.8944271802902222),
    vec3(0.8506507873535156, INVERSESQRT5, 0.27639320492744446),
    vec3(0.525731086730957, INVERSESQRT5, -0.7236068248748779),
    vec3(0.0, 1.0, 0.0),
];
const ICOSAHEDRON_POLE_INDICES: [[[u8; 3]; 5]; 4] = [
    [[2, 1, 0], [3, 2, 0], [4, 3, 0], [5, 4, 0], [1, 5, 0]],
    [[1, 2, 6], [2, 3, 7], [3, 4, 8], [4, 5, 9], [5, 1, 10]],
    [[6, 10, 1], [7, 6, 2], [8, 7, 3], [9, 8, 4], [10, 9, 5]],
    [[10, 6, 11], [6, 7, 11], [7, 8, 11], [8, 9, 11], [9, 10, 11]],
];

fn side_of_half_plane(local_position: Vec3, pole_index_a: u8, pole_index_b: u8) -> bool {
    let aux_a = ICOSAHEDRON_VERTICES[pole_index_a as usize]
        .cross(ICOSAHEDRON_VERTICES[pole_index_b as usize]);
    return local_position.dot(aux_a) > 0.0;
}

/// Intermediate barycentric coordinate system to convert between cartesian coordinates and Tiles of a GeodesicGrid
#[derive(Clone, Debug)]
pub struct IcosahedronCoordinates {
    triangle_latitude: u8,
    triangle_longitude: u8,
    barycentric: Vec3,
}

impl IcosahedronCoordinates {
    /// Returns coordinates in the same triangle but offset by the given barycentric coordinates
    pub fn offset(&self, barycentric_offset: Vec3) -> Self {
        Self {
            triangle_latitude: self.triangle_latitude,
            triangle_longitude: self.triangle_longitude,
            barycentric: self.barycentric + barycentric_offset,
        }
    }

    /// Converts from icosahedron coordinates to cartesian coordinates
    pub fn into_cartesian(&self) -> Vec3 {
        let pole_indices = &ICOSAHEDRON_POLE_INDICES[self.triangle_longitude as usize]
            [self.triangle_latitude as usize];
        let mut planes = [Vec4::default(); 3];
        for i in 0..3 {
            let b = ICOSAHEDRON_VERTICES[pole_indices[(i + 1) % 3] as usize];
            let c = ICOSAHEDRON_VERTICES[pole_indices[(i + 2) % 3] as usize];
            let tangent = (PI / 10.0 * self.barycentric[i]).tan();
            let normal = b.cross(c) - tangent * (b + c);
            planes[i].x = tangent * (1.0 + INVERSESQRT5);
            planes[i].y = normal.x;
            planes[i].z = normal.y;
            planes[i].w = normal.z;
        }
        let l_g0 = planes[0].x * planes[1].yzw() - planes[0].yzw() * planes[1].x;
        let l_g1 = planes[0].z * vec3(planes[1].w, 0.0, -planes[1].y)
            + planes[0].w * vec3(-planes[1].z, planes[1].y, 0.0)
            + planes[0].y * vec3(0.0, -planes[1].w, planes[1].z);
        let p = l_g0.y * vec4(0.0, -planes[2].w, 0.0, planes[2].y)
            + l_g0.z * vec4(0.0, planes[2].z, -planes[2].y, 0.0)
            + l_g1.y * vec4(planes[2].z, 0.0, -planes[2].x, 0.0)
            + l_g1.z * vec4(planes[2].w, 0.0, 0.0, -planes[2].x)
            + vec4(l_g1.x, -l_g1.x, l_g0.x, -l_g0.x) * planes[2].yxwz();
        return -p.yzw() / p.x;
    }

    /// Converts from cartesian coordinates to icosahedron coordinates
    pub fn from_cartesian(local_position: Vec3) -> Self {
        let latitude = 0.5 + 0.5 * local_position.x.atan2(local_position.z) / PI;
        /* let longitude = 1.0 - acos(local_position.y) / PI; */
        let is_nothern_hemisphere = local_position.y > 0.0;
        let latitude_sector = (latitude * 10.0) as u8;
        let mut triangle_latitude = (latitude_sector + is_nothern_hemisphere as u8) / 2 % 5;
        let mut triangle_longitude = 0;
        let mut pole_indices = [0, 0, 0];
        pole_indices[0] = 1 + (triangle_latitude + if is_nothern_hemisphere { 4 } else { 1 }) % 5;
        pole_indices[1] = 1 + triangle_latitude;
        if is_nothern_hemisphere {
            pole_indices[0] += 5;
            pole_indices[1] += 5;
            pole_indices[2] += 11;
            triangle_longitude = 3;
        }
        /* var triangle_longitude: i32 = select(0, 3, is_nothern_hemisphere);
        pole_indices = ICOSAHEDRON_POLE_INDICES[triangle_longitude][triangle_latitude]; */
        if side_of_half_plane(local_position, pole_indices[1], pole_indices[0]) {
            triangle_latitude = latitude_sector / 2;
            triangle_longitude = 1;
            pole_indices[0] = 1 + triangle_latitude;
            pole_indices[1] = 1 + (triangle_latitude + 1) % 5;
            pole_indices[2] = 6 + triangle_latitude;
            /* pole_indices = ICOSAHEDRON_POLE_INDICES[triangle_longitude][triangle_latitude]; */
            let border_indices = if (latitude_sector & 1) == 0 {
                [pole_indices[0], pole_indices[2]]
            } else {
                [pole_indices[2], pole_indices[1]]
            };
            if side_of_half_plane(local_position, border_indices[0], border_indices[1]) {
                triangle_latitude = (latitude_sector + 1) / 2 % 5;
                triangle_longitude = 2;
                pole_indices[0] = 6 + triangle_latitude;
                pole_indices[1] = 6 + (triangle_latitude + 4) % 5;
                pole_indices[2] = 1 + triangle_latitude;
                /* pole_indices = ICOSAHEDRON_POLE_INDICES[triangle_longitude][triangle_latitude]; */
            }
        }
        let mut barycentric = Vec3::default();
        for i in 0..3 {
            let b = ICOSAHEDRON_VERTICES[pole_indices[(i + 1) % 3] as usize];
            let c = ICOSAHEDRON_VERTICES[pole_indices[(i + 2) % 3] as usize];
            let numerator = local_position.dot(b.cross(c));
            let denominator = 1.0 + INVERSESQRT5 + local_position.dot(b) + local_position.dot(c);
            barycentric[i] = numerator.atan2(denominator) * 10.0 / PI;
        }
        // barycentric /= vec3(1.0, 1.0, 1.0).dot(barycentric);
        return Self {
            triangle_latitude,
            triangle_longitude,
            barycentric,
        };
    }

    /// Converts from icosahedron coordinates to geodesic grid tiles
    pub fn into_tile(&self, geodesic_grid: &GeodesicGrid) -> Tile {
        let float_tiles_per_edge = geodesic_grid.tiles_per_edge() as f32;
        let scaled_barycentric = self.barycentric * float_tiles_per_edge;
        let floored = [
            scaled_barycentric.x as usize,
            scaled_barycentric.y as usize,
            scaled_barycentric.z as usize,
        ];
        let mut tile = if (self.triangle_longitude & 1) == 0 {
            [floored[0] - floored[1] - floored[2] - 1, floored[1]]
        } else {
            [
                floored[1] + floored[2] - floored[0],
                geodesic_grid.tiles_per_edge() - 1 - floored[1],
            ]
        };
        tile[0] += (self.triangle_latitude as usize * 2 + 1) * geodesic_grid.tiles_per_edge();
        tile[1] += self.triangle_longitude as usize / 2 * geodesic_grid.tiles_per_edge();
        return tile;
    }

    /// Converts from geodesic grid tiles to icosahedron coordinates
    pub fn from_tile(tile: Tile, geodesic_grid: &GeodesicGrid) -> Self {
        let grid_height = geodesic_grid.height();
        let diagonal = tile[0] + tile[1] * 2 + 1;
        let triangle_latitude = tile[0] / grid_height;
        let triangle_longitude =
            diagonal / grid_height + tile[1] / geodesic_grid.tiles_per_edge() - triangle_latitude;
        let triangle_latitude_grid_height = triangle_latitude * grid_height;
        let mut barycentric = vec3(
            (tile[0] - triangle_latitude_grid_height) as f32,
            0.0,
            (diagonal + 1
                - triangle_latitude_grid_height
                - triangle_longitude * geodesic_grid.tiles_per_edge()) as f32,
        ) * 0.5;
        let float_tiles_per_edge = geodesic_grid.tiles_per_edge() as f32;
        if (triangle_longitude & 1) == 0 {
            barycentric.z = float_tiles_per_edge - barycentric.z;
        } else {
            barycentric.x = float_tiles_per_edge - barycentric.x - 0.5;
            barycentric.z = -0.5 * float_tiles_per_edge + barycentric.z - 0.5;
        }
        barycentric.y = float_tiles_per_edge - barycentric.x - barycentric.z;
        barycentric += if (tile[0] & 1) == (triangle_longitude & 1) {
            vec3(0.5, -0.5, 0.5) - Vec3::splat(1.0 / 6.0)
        } else {
            vec3(0.0, -0.5, 0.0) + Vec3::splat(1.0 / 6.0)
        };
        barycentric *= 1.0 / float_tiles_per_edge;
        return Self {
            triangle_latitude: triangle_latitude as u8,
            triangle_longitude: triangle_longitude as u8,
            barycentric,
        };
    }

    /// Returns the next closest tile midpoint, the tile corners and the difference to the midpoint
    pub fn tile_midpoint_corners_and_offset(
        &self,
        geodesic_grid: &GeodesicGrid,
    ) -> (Self, [Self; 3], Vec3) {
        let float_tiles_per_edge = geodesic_grid.tiles_per_edge() as f32;
        let scaled_barycentric = self.barycentric * float_tiles_per_edge;
        let floored = [
            scaled_barycentric.x as usize,
            scaled_barycentric.y as usize,
            scaled_barycentric.z as usize,
        ];
        let is_flipped = ((floored[0] + floored[1] + floored[2]) & 1) == 1;
        let inverse_tiles_per_edge = 1.0 / float_tiles_per_edge;
        let (midpoint_correction, offset_correction, corner_correction) = if is_flipped {
            (
                1.0 / 3.0,
                -inverse_tiles_per_edge * 3.0,
                -inverse_tiles_per_edge / 3.0,
            )
        } else {
            (
                2.0 / 3.0,
                inverse_tiles_per_edge * 3.0,
                inverse_tiles_per_edge / 3.0,
            )
        };
        let midpoint = vec3(floored[0] as f32, floored[1] as f32, floored[2] as f32)
            + Vec3::splat(midpoint_correction);
        let offset = (scaled_barycentric - midpoint) * offset_correction;
        let midpoint = Self {
            triangle_latitude: self.triangle_latitude,
            triangle_longitude: self.triangle_longitude,
            barycentric: midpoint * inverse_tiles_per_edge,
        };
        let corners = [
            midpoint.offset(vec3(-2.0, 1.0, 1.0) * corner_correction),
            midpoint.offset(vec3(1.0, -2.0, 1.0) * corner_correction),
            midpoint.offset(vec3(1.0, 1.0, -2.0) * corner_correction),
        ];
        return (midpoint, corners, offset);
    }
}
