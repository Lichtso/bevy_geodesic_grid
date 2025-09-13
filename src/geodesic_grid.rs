use crate::{
    icosahedron::IcosahedronCoordinates,
    tiles::{Tile, TileSet},
};
use bevy::prelude::*;

/// Subdivision of a geodesic polyhedron
#[derive(Clone, Copy, Debug)]
pub struct GeodesicGrid {
    tiles_per_edge: usize,
}

impl GeodesicGrid {
    /// Creates a new GeodesicGrid with the given tiles_per_edge
    #[inline(always)]
    pub fn new(tiles_per_edge: usize) -> Self {
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
            if side.abs() < 0.001 {
                return true;
            }
            count += (side < 0.0) as usize;
        }
        count == 1 || count == 2
    }

    fn tiles_of_great_circle_arc_internal(
        &self,
        start_tile: Tile,
        end_tile: Tile,
        start_local_position: Vec3,
        end_local_position: Vec3,
        tile_set: &mut TileSet,
    ) -> bool {
        if start_tile == end_tile {
            return true;
        }
        let line_plane_normal = start_local_position.cross(end_local_position);
        let anti_reversal_normal = line_plane_normal.cross(start_local_position);
        let (neighbor_tiles, neighbor_edges) = self.tile_neighbors(start_tile);
        let mut selected = None;
        for (neighbor_tile, neighbor_edge) in neighbor_tiles.iter().zip(neighbor_edges.iter()) {
            let icosahedron_coordinate = IcosahedronCoordinates::from_tile(*neighbor_tile, self);
            let local_position = IcosahedronCoordinates::into_cartesian(&icosahedron_coordinate);
            if local_position.dot(anti_reversal_normal) > -0.001
                && self.tile_plane_intersection(&icosahedron_coordinate, line_plane_normal, 0.0)
            {
                selected = Some((*neighbor_tile, *neighbor_edge, local_position));
                break;
            }
        }
        let Some((mut tile, mut prev_edge, mut prev_local_position)) = selected else {
            return false;
        };
        while tile != end_tile {
            tile_set.insert(tile);
            let (neighbor_tiles, neighbor_edges) = self.tile_neighbors(tile);
            let mut best = (None, prev_local_position.dot(end_local_position) - 0.001);
            for (edge, (neighbor_tile, neighbor_edge)) in
                neighbor_tiles.iter().zip(neighbor_edges.iter()).enumerate()
            {
                if edge == prev_edge {
                    continue;
                }
                let icosahedron_coordinate =
                    IcosahedronCoordinates::from_tile(*neighbor_tile, self);
                let local_position =
                    IcosahedronCoordinates::into_cartesian(&icosahedron_coordinate);
                let dist = local_position.dot(end_local_position);
                if dist > best.1
                    && self.tile_plane_intersection(&icosahedron_coordinate, line_plane_normal, 0.0)
                {
                    best = (Some((*neighbor_tile, *neighbor_edge, local_position)), dist);
                }
            }
            let Some(triple) = best.0 else {
                break;
            };
            (tile, prev_edge, prev_local_position) = triple;
        }
        true
    }

    /// Returns the line segment of tiles between the two given end points
    pub fn tiles_of_great_circle_arc(&self, start_tile: Tile, end_tile: Tile) -> Option<TileSet> {
        if start_tile == self.antipode(end_tile) {
            return None;
        }
        let start_icosahedron_coordinate = IcosahedronCoordinates::from_tile(start_tile, self);
        let end_icosahedron_coordinate = IcosahedronCoordinates::from_tile(end_tile, self);
        let start_local_position =
            IcosahedronCoordinates::into_cartesian(&start_icosahedron_coordinate);
        let end_local_position =
            IcosahedronCoordinates::into_cartesian(&end_icosahedron_coordinate);
        let mut tile_set = TileSet::new(*self);
        if self.tiles_of_great_circle_arc_internal(
            start_tile,
            end_tile,
            start_local_position,
            end_local_position,
            &mut tile_set,
        ) {
            tile_set.insert(start_tile);
            tile_set.insert(end_tile);
            Some(tile_set)
        } else {
            None
        }
    }

    /// Returns the polygon of tiles between the given points (only stroke, no fill)
    pub fn tiles_of_spherical_polygon(&self, vertices: &[Tile]) -> Option<TileSet> {
        if vertices.len() < 2 {
            return if vertices.is_empty() {
                None
            } else {
                let mut tile_set = TileSet::new(*self);
                tile_set.insert(vertices[0]);
                Some(tile_set)
            };
        }
        let start_icosahedron_coordinate = IcosahedronCoordinates::from_tile(vertices[0], self);
        let start_local_position =
            IcosahedronCoordinates::into_cartesian(&start_icosahedron_coordinate);
        let mut prev_local_position = start_local_position;
        let mut tile_set = TileSet::new(*self);
        for i in 0..vertices.len() {
            let end_tile;
            let end_local_position;
            if i == vertices.len() - 1 {
                end_tile = vertices[0];
                end_local_position = start_local_position;
            } else {
                end_tile = vertices[i + 1];
                let end_icosahedron_coordinate = IcosahedronCoordinates::from_tile(end_tile, self);
                end_local_position =
                    IcosahedronCoordinates::into_cartesian(&end_icosahedron_coordinate);
            }
            if self.tiles_of_great_circle_arc_internal(
                vertices[i],
                end_tile,
                prev_local_position,
                end_local_position,
                &mut tile_set,
            ) {
                tile_set.insert(vertices[i]);
            } else {
                return None;
            }
            prev_local_position = end_local_position;
        }
        Some(tile_set)
    }
}
