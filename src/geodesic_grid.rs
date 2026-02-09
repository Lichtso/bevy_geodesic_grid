use crate::{
    icosahedron::IcosahedronCoordinates,
    tiles::{Tile, TileSet},
};
use bevy::prelude::*;

/// Converts the tile coordinates to a grid with tiles_per_edge / 2
pub fn divide_by_two([x, y]: Tile) -> Tile {
    [x / 4 * 2 + (x % 4 > 3 - y % 2 * 3) as usize, y / 2]
}

/// Subdivision of a geodesic polyhedron
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct GeodesicGrid {
    tiles_per_edge: usize,
}

impl GeodesicGrid {
    /// Creates a new GeodesicGrid with the given tiles_per_edge
    #[inline(always)]
    pub const fn new(tiles_per_edge: usize) -> Self {
        Self { tiles_per_edge }
    }

    /// Returns the number of tiles per icosahedron edge
    #[inline(always)]
    pub fn tiles_per_edge(&self) -> usize {
        self.tiles_per_edge
    }

    /// Returns the number of tiles in X direction
    #[inline(always)]
    pub fn width(&self) -> usize {
        self.tiles_per_edge * 10
    }

    /// Returns the number of tiles in Y direction
    #[inline(always)]
    pub fn height(&self) -> usize {
        self.tiles_per_edge * 2
    }

    /// Returns the tile which is on the opposite side
    pub fn antipode(&self, tile: Tile) -> Tile {
        let grid_height = self.height();
        [
            ((grid_height - 1 - tile[1]) * 2 + 1 - (tile[0] & 1)
                + (tile[0] / grid_height + 2) % 5 * grid_height)
                % self.width(),
            (grid_height - 1 - tile[0] % grid_height) / 2 + self.tiles_per_edge
                - tile[1] / self.tiles_per_edge * self.tiles_per_edge,
        ]
    }

    /// Returns the three neighbor tiles of the given tile
    pub fn tile_neighbors(&self, tile: Tile) -> ([Tile; 3], [usize; 3]) {
        // Basic neighborhood inside an icosahedron triangle
        let offset_sign = (tile[0] as isize & 1) * 2 - 1;
        let mut neighbors = [
            [(tile[0] as isize + offset_sign) as usize, tile[1]],
            [
                (tile[0] as isize - offset_sign) as usize,
                (tile[1] as isize + offset_sign) as usize,
            ],
            [(tile[0] as isize - offset_sign) as usize, tile[1]],
        ];
        let mut edges = [0, 1, 2];
        // Extended neighborhood across icosahedron triangles
        let grid_height = self.height();
        let tile_x_in_stripe = tile[0] % grid_height;
        if tile_x_in_stripe == 0 {
            neighbors[0] = if tile[1] >= self.tiles_per_edge {
                let grid_width = self.width();
                [
                    (tile[0] + grid_width - 1) % grid_width,
                    tile[1] - self.tiles_per_edge,
                ]
            } else {
                edges[0] = 1;
                [
                    (tile[0] / grid_height + 4) % 5 * grid_height + tile[1] * 2,
                    0,
                ]
            };
        } else if tile_x_in_stripe == grid_height - 1 {
            neighbors[0] = if tile[1] < self.tiles_per_edge {
                [(tile[0] + 1) % self.width(), tile[1] + self.tiles_per_edge]
            } else {
                edges[0] = 1;
                [
                    (tile[0] / grid_height + 1) % 5 * grid_height
                        + (tile[1] - self.tiles_per_edge) * 2
                        + 1,
                    grid_height - 1,
                ]
            };
        }
        if tile[1] == 0 {
            if offset_sign < 0 {
                neighbors[1] = [
                    (tile[0] / grid_height + 1) % 5 * grid_height,
                    tile[0] % grid_height / 2,
                ];
                edges[1] = 0;
            }
        } else if tile[1] == grid_height - 1 {
            if offset_sign > 0 {
                neighbors[1] = [
                    (tile[0] / grid_height + 4) % 5 * grid_height + grid_height - 1,
                    tile[0] % grid_height / 2 + self.tiles_per_edge,
                ];
                edges[1] = 0;
            }
        }
        (neighbors, edges)
    }

    /// Returns whether or not the plane passes though the tile
    pub fn tile_plane_intersection(
        &self,
        icosahedron_coordinates: &IcosahedronCoordinates,
        plane_normal: Vec3,
        plane_origin: f32,
    ) -> bool {
        let (_midpoint, corners, _offset) =
            icosahedron_coordinates.tile_midpoint_corners_and_offset(self);
        let mut count = 0;
        for i in 0..3 {
            let cartesian = corners[i].into_cartesian();
            let side = cartesian.dot(plane_normal) - plane_origin;
            if side.abs() < 0.00001 {
                return true;
            }
            count += (side < 0.0) as usize;
        }
        count == 1 || count == 2
    }

    /// Returns the polygon of tiles between the given points (only stroke, no fill)
    ///
    /// Assumes that no two (cyclically) consecuitive vertices are antipodal.
    pub fn tiles_of_spherical_polygon(&self, vertices: &[(Vec3, Tile)], tile_set: &mut TileSet) {
        assert_eq!(self, tile_set.grid());
        if vertices.len() < 2 {
            if !vertices.is_empty() {
                tile_set.insert(vertices[0].1);
            };
            return;
        }
        let start_index = (vertices.len() == 2) as usize;
        let mut prev_cartesian = vertices[0].0;
        for i in start_index..vertices.len() {
            let (end_cartesian, end_tile) = if i == vertices.len() - 1 {
                vertices[0]
            } else {
                vertices[i + 1]
            };
            let line_plane_normal = prev_cartesian.cross(end_cartesian);
            let mut tile = vertices[i].1;
            let mut prev_edge = 3;
            while tile != end_tile {
                tile_set.insert(tile);
                let (neighbor_tiles, neighbor_edges) = self.tile_neighbors(tile);
                let mut selected_edge = 3;
                let mut best = -1.0;
                for (edge, (neighbor_tile, neighbor_edge)) in
                    neighbor_tiles.iter().zip(neighbor_edges.iter()).enumerate()
                {
                    if edge == prev_edge {
                        continue;
                    }
                    let icosahedron_coordinate =
                        IcosahedronCoordinates::from_tile(*neighbor_tile, self);
                    let cartesian = IcosahedronCoordinates::into_cartesian(&icosahedron_coordinate);
                    let dist = cartesian.dot(end_cartesian);
                    if dist > best
                        && self.tile_plane_intersection(
                            &icosahedron_coordinate,
                            line_plane_normal,
                            0.0,
                        )
                    {
                        (tile, selected_edge, best) = (*neighbor_tile, *neighbor_edge, dist);
                    }
                }
                prev_edge = selected_edge;
            }
            prev_cartesian = end_cartesian;
        }
    }

    /// Returns the polygon of tiles surrounded by the given points (only fill, no stroke)
    ///
    /// Assumes that no two (cyclically) consecuitive vertices are antipodal.
    pub fn tiles_in_spherical_polygon(&self, vertices: &[(Vec3, Tile)], tile_set: &mut TileSet) {
        assert_eq!(self, tile_set.grid());
        if vertices.len() < 3 {
            return;
        }
        let mut prev_cartesian = vertices[1].0;
        let start_spoke = prev_cartesian.cross(vertices[0].0);
        let mut triangle_fan = Vec::default();
        for i in 2..vertices.len() {
            let next_cartesian = vertices[i].0;
            let spoke = next_cartesian.cross(vertices[0].0);
            let rim = prev_cartesian.cross(next_cartesian);
            let winding = spoke.cross(rim).dot(next_cartesian) > 0.0;
            triangle_fan.push((spoke, rim, winding));
            prev_cartesian = next_cartesian;
        }
        for y in 0..self.height() {
            for x in 0..self.width() {
                let tile = [x, y];
                let icosahedron_coordinate = IcosahedronCoordinates::from_tile(tile, self);
                let cartesian = IcosahedronCoordinates::into_cartesian(&icosahedron_coordinate);
                let mut prev_spoke_side = start_spoke.dot(cartesian) > 0.0;
                let mut winding_counter = 0;
                for (spoke, rim, winding) in &triangle_fan {
                    let spoke_side = spoke.dot(cartesian) > 0.0;
                    let rim_side = rim.dot(cartesian) > 0.0;
                    let inside_triangle = (prev_spoke_side == *winding
                        && spoke_side != *winding
                        && rim_side != *winding)
                        != *winding;
                    winding_counter += inside_triangle as usize;
                    prev_spoke_side = spoke_side;
                }
                if winding_counter % 2 == 1 {
                    tile_set.insert(tile);
                }
            }
        }
    }
}
