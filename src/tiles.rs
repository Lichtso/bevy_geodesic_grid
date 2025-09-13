use crate::geodesic_grid::GeodesicGrid;

/// One grid cell of a GeodesicGrid
pub type Tile = [usize; 2];

/*
https://en.wikipedia.org/wiki/Succinct_data_structure ?
- Top level: Split in 10 quads (grid.tiles_per_edge * 2, grid.tiles_per_edge) covering two icosahedron triangles
- Mid levels, recursive:
    - All full
    - Mostly full (partial vacancy list)
    - Bitmap and child list
    - Mostly empty (partial vacancy list)
    - All empty
- Lowest level: bitmap
*/

/// Set of Tiles
#[derive(Clone, Debug)]
pub struct TileSet {
    grid: GeodesicGrid,
    hash_set: std::collections::HashSet<Tile>,
}

impl TileSet {
    /// Creates an empty TileSet.
    pub fn new(grid: GeodesicGrid) -> Self {
        Self {
            grid,
            hash_set: std::collections::HashSet::default(),
        }
    }

    /// Adds a Tile to the TileSet. Returns whether the Tile was not present in the TileSet.
    pub fn insert(&mut self, tile: Tile) -> bool {
        self.hash_set.insert(tile)
    }

    /// Removes a Tile from the TileSet. Returns whether the Tile was present in the TileSet.
    pub fn remove(&mut self, tile: Tile) -> bool {
        self.hash_set.remove(&tile)
    }

    /// Returns true if the TileSet contains the given Tile.
    pub fn contains(&self, tile: Tile) -> bool {
        self.hash_set.contains(&tile)
    }

    /// Returns the number of Tiles in the TileSet.
    pub fn len(&self) -> usize {
        self.hash_set.len()
    }

    /// Returns true if the TileSet contains no Tiles.
    pub fn is_empty(&self) -> bool {
        self.hash_set.is_empty()
    }

    /// Returns true if the TileSet contains all Tiles.
    pub fn is_full(&self) -> bool {
        self.hash_set.len() == self.grid.width() * self.grid.height()
    }

    /// Returns true if self has no Tiles in common with other. This is equivalent to checking for an empty intersection.
    pub fn is_disjoint(&self, other: &Self) -> bool {
        self.hash_set.is_disjoint(&other.hash_set)
    }

    /// Returns true if the TileSet is a subset of another, i.e., other contains at least all the Tiles in self.
    pub fn is_subset(&self, other: &Self) -> bool {
        self.hash_set.is_subset(&other.hash_set)
    }

    /// Returns true if the TileSet is a superset of another, i.e., self contains at least all the Tiles in other.
    pub fn is_superset(&self, other: &Self) -> bool {
        self.hash_set.is_superset(&other.hash_set)
    }

    /// An iterator visiting all Tiles in the TileSet in arbitrary order.
    pub fn iter(&self) -> std::collections::hash_set::Iter<'_, Tile> {
        self.hash_set.iter()
    }

    // difference
    // symmetric_difference
    // intersection
    // union
}
