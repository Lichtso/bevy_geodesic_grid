use crate::geodesic_grid::GeodesicGrid;

/// One grid cell of a GeodesicGrid
pub type Tile = [usize; 2];

const LEAF_WIDTH_BITS: u32 = 5;
const LEAF_HEIGHT_BITS: u32 = 4;
const LEAF_WIDTH: usize = 1 << LEAF_WIDTH_BITS;
const LEAF_HEIGHT: usize = 1 << LEAF_HEIGHT_BITS;
const LEAF_CHILDREN: usize = LEAF_WIDTH * LEAF_HEIGHT;
const BRANCH_WIDTH_BITS: u32 = 3;
const BRANCH_HEIGHT_BITS: u32 = 3;
const BRANCH_WIDTH: usize = 1 << BRANCH_WIDTH_BITS;
const BRANCH_HEIGHT: usize = 1 << BRANCH_HEIGHT_BITS;
const BRANCH_CHILDREN: usize = BRANCH_WIDTH * BRANCH_HEIGHT;
const MAX_LEVELS: usize = 5;
const LINK_EMPTY: Link = Link::MAX - 1;
const LINK_FULL: Link = Link::MAX;

type Link = u16;
type TileCounter = u32;
type LeafBitfield = [u64; LEAF_CHILDREN / 64];
type BranchLinks = [Link; BRANCH_CHILDREN];

macro_rules! traverse_tree {
    (child_index, 0, $tile:expr) => {{
        let child_position = [$tile[0] & (LEAF_WIDTH - 1), $tile[1] & (LEAF_HEIGHT - 1)];
        let child_index = (child_position[1] << LEAF_WIDTH_BITS) + child_position[0];
        (child_index >> 6, child_index & 63)
    }};
    (child_index, $next_level:expr, $tile:expr) => {{
        let child_position = [
            ($tile[0] >> (LEAF_WIDTH_BITS + $next_level as u32 * BRANCH_WIDTH_BITS))
                & (BRANCH_WIDTH - 1),
            ($tile[1] >> (LEAF_HEIGHT_BITS + $next_level as u32 * BRANCH_HEIGHT_BITS))
                & (BRANCH_HEIGHT - 1),
        ];
        (child_position[1] << BRANCH_WIDTH_BITS) + child_position[0]
    }};
    (insert_or_remove, &mut $self:expr, $tile:expr, $is_set:expr) => {{
        let branch_levels = $self.branch_levels();
        let mut link = $self.root;
        let mut levels = [(LINK_EMPTY, usize::MAX); MAX_LEVELS];
        levels[branch_levels].0 = link;
        for reverse_level in 0..branch_levels {
            let level = branch_levels - reverse_level;
            let next_level = level - 1;
            let child_index = traverse_tree!(child_index, next_level, $tile);
            let branch_links = &$self.branch_links[link as usize];
            let mut next_link = branch_links[child_index];
            if next_link >= LINK_EMPTY {
                if (next_link == LINK_FULL) as usize == $is_set {
                    return false;
                }
                next_link = $self.allocate(next_level, $is_set == 0);
                let branch_links = &mut $self.branch_links[link as usize];
                branch_links[child_index] = next_link;
            }
            levels[next_level].0 = next_link;
            levels[level].1 = child_index;
            link = next_link;
        }
        let (leaf_bit_index, leaf_bit_shift) = traverse_tree!(child_index, 0, $tile);
        let leaf_bitfield = &mut $self.leaf_bitfields[link as usize];
        if ((leaf_bitfield[leaf_bit_index] >> leaf_bit_shift) & 1) == $is_set {
            return false;
        }
        if $is_set == 1 {
            leaf_bitfield[leaf_bit_index] |= 1 << leaf_bit_shift;
        } else {
            leaf_bitfield[leaf_bit_index] &= !(1 << leaf_bit_shift);
        }
        for level in 0..=branch_levels {
            let link = levels[level].0;
            let counter = if level == 0 {
                &mut $self.leaf_counters[link as usize]
            } else {
                &mut $self.branch_counters[link as usize]
            };
            *counter = (*counter as isize + $is_set as isize * 2 - 1) as TileCounter;
            if *counter == Self::subtree_coverage(level) as TileCounter * $is_set as TileCounter {
                $self.free(level, link);
                if level == branch_levels {
                    $self.root = LINK_EMPTY + $is_set as Link;
                } else {
                    let (prev_link, prev_child_index) = levels[level + 1];
                    let branch_links = &mut $self.branch_links[prev_link as usize];
                    branch_links[prev_child_index] = LINK_EMPTY + $is_set as Link;
                }
            }
        }
    }};
}

/// A set of tiles using a hierarchical bitfield
pub struct TileSet {
    grid: GeodesicGrid,
    leaf_bitfields: Vec<LeafBitfield>,
    leaf_counters: Vec<TileCounter>,
    branch_links: Vec<BranchLinks>,
    branch_counters: Vec<TileCounter>,
    free_leaf: Link,
    free_branch: Link,
    root: Link,
}

impl TileSet {
    fn debug_print_leaf(&self, link: Link) {
        println!("[{}:{}](", link, self.leaf_counters[link as usize]);
        let leaf_bitfield = &self.leaf_bitfields[link as usize];
        for child_index in 0..LEAF_CHILDREN {
            let leaf_bitfield = &leaf_bitfield[child_index / 64];
            let leaf_bit_shift = child_index % 64;
            if ((*leaf_bitfield >> leaf_bit_shift) & 1) == 1 {
                print!(" {}/{}", child_index % LEAF_WIDTH, child_index / LEAF_WIDTH);
            }
        }
        print!(")");
    }

    fn debug_print_branch(&self, mut link: Link, level: usize) {
        println!("[{}:{}]{{", link, self.branch_counters[link as usize]);
        let branch_links = &self.branch_links[link as usize];
        for child_index in 0..BRANCH_CHILDREN {
            link = branch_links[child_index];
            if link >= LINK_EMPTY {
            } else {
                print!(" {}/{}", child_index % LEAF_WIDTH, child_index / LEAF_WIDTH);
                if level == 1 {
                    self.debug_print_leaf(link);
                } else {
                    self.debug_print_branch(link, level - 1);
                }
            }
        }
        println!("}}");
    }

    pub fn debug_print(&self) {
        let branch_levels = self.branch_levels();
        if self.is_empty() {
            println!("{{}}");
        } else {
            if branch_levels == 0 {
                self.debug_print_leaf(self.root);
            } else {
                self.debug_print_branch(self.root, branch_levels);
            }
        }
    }

    /// Creates an empty TileSet.
    pub fn new(grid: GeodesicGrid) -> Self {
        Self {
            grid,
            leaf_bitfields: Vec::default(),
            leaf_counters: Vec::default(),
            branch_links: Vec::default(),
            branch_counters: Vec::default(),
            free_leaf: LINK_EMPTY,
            free_branch: LINK_EMPTY,
            root: LINK_EMPTY,
        }
    }

    /// Returns the GeodesicGrid underpinning the TileSet.
    pub fn grid(&self) -> &GeodesicGrid {
        &self.grid
    }

    /// Returns the number of Tiles in the TileSet.
    pub fn len(&self) -> usize {
        if self.root == LINK_EMPTY {
            0
        } else if self.branch_levels() == 0 {
            self.leaf_counters[self.root as usize] as usize
        } else {
            self.branch_counters[self.root as usize] as usize
        }
    }

    /// Returns true if the TileSet contains no Tiles.
    pub fn is_empty(&self) -> bool {
        self.root == LINK_EMPTY
    }

    /// Returns true if the TileSet contains all Tiles.
    pub fn is_full(&self) -> bool {
        self.len() as usize == self.grid.width() * self.grid.height()
    }

    /// Returns true if the TileSet contains the given Tile.
    pub fn contains(&self, tile: Tile) -> bool {
        if self.is_empty() {
            return false;
        }
        let mut link = self.root;
        let branch_levels = self.branch_levels();
        for reverse_level in 0..branch_levels {
            let next_level = branch_levels - reverse_level - 1;
            let child_index = traverse_tree!(child_index, next_level, tile);
            let branch_links = &self.branch_links[link as usize];
            link = branch_links[child_index];
            if link >= LINK_EMPTY {
                return link == LINK_FULL;
            }
        }
        let (leaf_bit_index, leaf_bit_shift) = traverse_tree!(child_index, 0, tile);
        let leaf_bitfield = &self.leaf_bitfields[link as usize];
        ((leaf_bitfield[leaf_bit_index] >> leaf_bit_shift) & 1) == 1
    }

    /// Adds a Tile to the TileSet. Returns whether the Tile was not present in the TileSet.
    pub fn insert(&mut self, tile: Tile) -> bool {
        if self.is_empty() {
            self.root = self.allocate(self.branch_levels(), false);
        }
        traverse_tree!(insert_or_remove, &mut self, tile, 1);
        true
    }

    /// Removes a Tile from the TileSet. Returns whether the Tile was present in the TileSet.
    pub fn remove(&mut self, tile: Tile) -> bool {
        if self.is_empty() {
            return false;
        }
        traverse_tree!(insert_or_remove, &mut self, tile, 0);
        true
    }

    /// An iterator visiting all Tiles in the TileSet in arbitrary order.
    pub fn iter<'a>(&'a self) -> TileSetIterator<'a> {
        let mut iter = TileSetIterator {
            tile_set: self,
            levels: [LINK_EMPTY; MAX_LEVELS],
            level: self.branch_levels(),
            position: [0, 0],
            step_size: [0, 0],
            range_min: [0, 0],
            range_max: [0, 0],
            single_stepping: false,
        };
        iter.levels[iter.level] = self.root;
        iter.recalculate_level();
        iter
    }

    // difference
    // symmetric_difference
    // intersection
    // union

    fn branch_levels(&self) -> usize {
        (self.grid.width() as u32)
            .ilog2()
            .saturating_sub(LEAF_WIDTH_BITS)
            .div_ceil(BRANCH_WIDTH_BITS) as usize
    }

    fn subtree_coverage(level: usize) -> TileCounter {
        (LEAF_CHILDREN as TileCounter) << level as u32 * (BRANCH_WIDTH_BITS + BRANCH_HEIGHT_BITS)
    }

    fn allocate(&mut self, level: usize, filled: bool) -> Link {
        if level == 0 {
            if self.free_leaf == LINK_EMPTY {
                self.leaf_bitfields
                    .push([u64::MAX * filled as u64; LEAF_CHILDREN / 64]);
                self.leaf_counters
                    .push(Self::subtree_coverage(0) * (filled as TileCounter));
                self.leaf_counters.len() as Link - 1
            } else {
                let link = self.free_leaf;
                self.free_leaf = self.leaf_counters[link as usize] as Link;
                self.leaf_bitfields[link as usize] = [u64::MAX * filled as u64; LEAF_CHILDREN / 64];
                self.leaf_counters[link as usize] =
                    Self::subtree_coverage(level) * (filled as TileCounter);
                link
            }
        } else if self.free_branch == LINK_EMPTY {
            self.branch_links
                .push([LINK_EMPTY + filled as Link; BRANCH_CHILDREN]);
            self.branch_counters
                .push(Self::subtree_coverage(level) * (filled as TileCounter));
            self.branch_counters.len() as Link - 1
        } else {
            let link = self.free_branch;
            self.free_branch = self.branch_counters[link as usize] as Link;
            self.branch_links[link as usize] = [LINK_EMPTY + filled as Link; BRANCH_CHILDREN];
            self.branch_counters[link as usize] =
                Self::subtree_coverage(level) * (filled as TileCounter);
            link
        }
    }

    fn free(&mut self, level: usize, link: Link) {
        if level == 0 {
            if link as usize + 1 == self.leaf_counters.len() {
                self.leaf_bitfields.pop();
                self.leaf_counters.pop();
            } else {
                self.leaf_counters[link as usize] = self.free_leaf as TileCounter;
                self.free_leaf = link;
            }
        } else if link as usize + 1 == self.branch_counters.len() {
            self.branch_links.pop();
            self.branch_counters.pop();
        } else {
            self.branch_counters[link as usize] = self.free_branch as TileCounter;
            self.free_branch = link;
        }
    }
}

/// Iterator over a set of Tiles
pub struct TileSetIterator<'a> {
    tile_set: &'a TileSet,
    levels: [Link; MAX_LEVELS],
    level: usize,
    position: Tile,
    step_size: Tile,
    range_min: Tile,
    range_max: Tile,
    single_stepping: bool,
}

impl<'a> TileSetIterator<'a> {
    fn recalculate_level(&mut self) {
        let level_bits = [
            LEAF_WIDTH_BITS + self.level as u32 * BRANCH_WIDTH_BITS,
            LEAF_HEIGHT_BITS + self.level as u32 * BRANCH_HEIGHT_BITS,
        ];
        let level_size = [1 << level_bits[0], 1 << level_bits[1]];
        self.step_size = if self.single_stepping {
            [1, 1]
        } else {
            [
                level_size[0] >> BRANCH_WIDTH_BITS,
                level_size[1] >> BRANCH_HEIGHT_BITS,
            ]
        };
        self.range_min = [
            self.position[0] & !(level_size[0] - 1),
            self.position[1] & !(level_size[1] - 1),
        ];
        self.range_max = [
            self.range_min[0] + level_size[0],
            self.range_min[1] + level_size[1],
        ];
    }
}

impl<'a> Iterator for TileSetIterator<'a> {
    type Item = Tile;

    fn next(&mut self) -> Option<Self::Item> {
        while self.level <= self.tile_set.branch_levels() {
            let mut link = self.levels[self.level];
            while !self.single_stepping {
                let next_level = self.level - 1;
                let child_index = traverse_tree!(child_index, next_level, self.position);
                let branch_links = &self.tile_set.branch_links[link as usize];
                let next_link = branch_links[child_index];
                if next_link != LINK_EMPTY {
                    link = next_link;
                    self.level = next_level;
                    self.levels[next_level] = next_link;
                    self.single_stepping = next_link == LINK_FULL || next_level == 0;
                    self.recalculate_level();
                } else {
                    break;
                }
            }
            let mut result = None;
            if self.single_stepping {
                result = if link == LINK_FULL {
                    Some(self.position)
                } else {
                    let (leaf_bit_index, leaf_bit_shift) =
                        traverse_tree!(child_index, 0, self.position);
                    let leaf_bitfield = &self.tile_set.leaf_bitfields[link as usize];
                    (((leaf_bitfield[leaf_bit_index] >> leaf_bit_shift) & 1) == 1)
                        .then_some(self.position)
                };
            }
            loop {
                self.position[0] += self.step_size[0];
                if self.position[0] >= self.range_max[0] {
                    self.position[0] = self.range_min[0];
                    self.position[1] += self.step_size[1];
                }
                if self.position[1] >= self.range_max[1] {
                    self.position[1] = self.range_min[1];
                    self.level += 1;
                    self.single_stepping = false;
                    self.recalculate_level();
                } else {
                    break;
                }
            }
            if result.is_some() {
                debug_assert!(self.tile_set.contains(result.unwrap()));
                return result;
            }
        }
        None
    }
}

/*pub struct TileSet {
    grid: GeodesicGrid,
    tile_count: u32,
    bitfield: Vec<u64>,
}

impl TileSet {
    /// Creates an empty TileSet.
    pub fn new(grid: GeodesicGrid) -> Self {
        Self {
            grid,
            tile_count: 0,
            bitfield: vec![0; (grid.width() * grid.height() + 63) / 64],
        }
    }

    /// Returns the number of Tiles in the TileSet.
    pub fn len(&self) -> usize {
        self.tile_count as usize
    }

    /// Returns true if the TileSet contains no Tiles.
    pub fn is_empty(&self) -> bool {
        self.tile_count == 0
    }

    /// Returns true if the TileSet contains all Tiles.
    pub fn is_full(&self) -> bool {
        self.tile_count as usize == self.grid.width() * self.grid.height()
    }

    /// Returns true if the TileSet contains the given Tile.
    pub fn contains(&self, tile: Tile) -> bool {
        let tile_index = tile[1] * self.grid.width() + tile[0];
        (self.bitfield[tile_index / 64] >> (tile_index % 64)) & 1 == 1
    }

    /// Adds a Tile to the TileSet. Returns whether the Tile was not present in the TileSet.
    pub fn insert(&mut self, tile: Tile) -> bool {
        let tile_index = tile[1] * self.grid.width() + tile[0];
        if (self.bitfield[tile_index / 64] >> (tile_index % 64)) & 1 == 1 {
            return false;
        }
        self.bitfield[tile_index / 64] |= 1 << (tile_index % 64);
        self.tile_count += 1;
        true
    }

    /// Removes a Tile from the TileSet. Returns whether the Tile was present in the TileSet.
    pub fn remove(&mut self, tile: Tile) -> bool {
        let tile_index = tile[1] * self.grid.width() + tile[0];
        if (self.bitfield[tile_index / 64] >> (tile_index % 64)) & 1 == 0 {
            return false;
        }
        self.bitfield[tile_index / 64] &= !(1 << (tile_index % 64));
        self.tile_count -= 1;
        true
    }

    /// An iterator visiting all Tiles in the TileSet in arbitrary order.
    pub fn iter<'a>(&'a self) -> TileSetIterator<'a> {
        TileSetIterator {
            tile_set: self,
            position: [0, 0],
        }
    }
}

/// Iterator over a set of Tiles
pub struct TileSetIterator<'a> {
    tile_set: &'a TileSet,
    position: Tile,
}

impl<'a> Iterator for TileSetIterator<'a> {
    type Item = Tile;

    fn next(&mut self) -> Option<Self::Item> {
        while self.position[1] < self.tile_set.grid.height() {
            let result = self
                .tile_set
                .contains(self.position)
                .then_some(self.position);
            self.position[0] += 1;
            if self.position[0] >= self.tile_set.grid.width() {
                self.position[0] = 0;
                self.position[1] += 1;
            }
            if result.is_some() {
                return result;
            }
        }
        None
    }
}*/
