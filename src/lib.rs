use bevy::{
    asset::RenderAssetUsages,
    asset::{load_internal_asset, uuid_handle},
    prelude::*,
    reflect::TypePath,
    render::{
        render_resource::AsBindGroup,
        render_resource::{Extent3d, TextureDimension, TextureFormat},
    },
    shader::ShaderRef,
};
use bevy_ellipsoid_billboard::EllipsoidBillboardPlugin;
use geodesic_grid::GeodesicGrid;

pub mod geodesic_grid;
pub mod icosahedron;
pub mod surface_structures;
pub mod tiles;

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
#[bind_group_data(GeodesicGridMaterialKey)]
pub struct GeodesicGridMaterial {
    #[texture(0, sample_type = "u_int")]
    pub tile_texture: Handle<Image>,
    #[texture(1, sample_type = "u_int")]
    pub selection_texture: Handle<Image>,
    #[texture(2, sample_type = "float")]
    pub palette_texture: Handle<Image>,
}

impl GeodesicGridMaterial {
    pub fn new(
        images: &mut ResMut<Assets<Image>>,
        grid: &GeodesicGrid,
        palette_image: Handle<Image>,
    ) -> GeodesicGridMaterial {
        let tile_image = Image::new_fill(
            Extent3d {
                width: grid.width() as u32,
                height: grid.height() as u32,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            &[0u8],
            TextureFormat::R8Uint,
            RenderAssetUsages::RENDER_WORLD,
        );
        let selection_image = Image::new_fill(
            Extent3d {
                width: grid.width() as u32,
                height: grid.height() as u32,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            &[0u8],
            TextureFormat::R8Uint,
            RenderAssetUsages::RENDER_WORLD,
        );
        GeodesicGridMaterial {
            tile_texture: images.add(tile_image),
            selection_texture: images.add(selection_image),
            palette_texture: palette_image,
        }
    }

    pub fn render_tiles(
        &self,
        images: &mut ResMut<Assets<Image>>,
        mut tile_data: Vec<u8>,
        mip_map: bool,
    ) {
        let tile_image = images.get_mut(&self.tile_texture).unwrap();
        if mip_map {
            let (mut width, mut height) =
                (tile_image.width() as usize, tile_image.height() as usize);
            tile_image.texture_descriptor.mip_level_count = tile_image.height().ilog2();
            tile_data.resize(width * height * 4 / 3, 0u8);
            const CHANNELS: usize = 12;
            let mut super_accumulators = vec![[0; CHANNELS]; width * height / 4];
            let (super_width, super_height) = (width / 2, height / 2);
            for y in 0..height {
                for x in 0..width {
                    let [super_x, super_y] = geodesic_grid::divide_by_two([x, y]);
                    super_accumulators[super_y * super_width + super_x]
                        [tile_data[y * width + x] as usize] += 1;
                }
            }
            let mut offset = width * height;
            let mut accumulators = vec![[0; CHANNELS]; width * height / 16];
            (width, height) = (super_width, super_height);
            core::mem::swap(&mut accumulators, &mut super_accumulators);
            for _mip_level in 1..tile_image.texture_descriptor.mip_level_count {
                let (super_width, super_height) = (width / 2, height / 2);
                for y in 0..height {
                    for x in 0..width {
                        let [super_x, super_y] = geodesic_grid::divide_by_two([x, y]);
                        let mut max = (0, 0);
                        for channel in 0..CHANNELS {
                            let count = accumulators[y * width + x][channel];
                            if count > max.1 {
                                max = (channel as u8, count);
                            }
                            super_accumulators[super_y * super_width + super_x][channel] += count;
                        }
                        tile_data[offset + y * width + x] = max.0;
                    }
                }
                offset += width * height;
                (width, height) = (super_width, super_height);
                core::mem::swap(&mut accumulators, &mut super_accumulators);
            }
        } else {
            tile_image.texture_descriptor.mip_level_count = 1;
        }
        tile_image.data = Some(tile_data);
    }
}

const GEODESIC_GRID_SHADER: Handle<Shader> = uuid_handle!("139ad40b-cae2-4eba-9c5c-4d69b5df2b23");

impl Material for GeodesicGridMaterial {
    fn vertex_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER.into()
    }

    fn fragment_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER.into()
    }

    fn prepass_vertex_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER.into()
    }

    fn prepass_fragment_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER.into()
    }

    #[inline]
    fn alpha_mode(&self) -> AlphaMode {
        // Workaround to force MeshPipelineKey::MAY_DISCARD
        AlphaMode::AlphaToCoverage
    }
}

#[derive(Eq, PartialEq, Hash, Clone)]
pub struct GeodesicGridMaterialKey {}

impl From<&GeodesicGridMaterial> for GeodesicGridMaterialKey {
    fn from(_material: &GeodesicGridMaterial) -> Self {
        Self {}
    }
}

pub struct GeodesicGridPlugin;

impl Plugin for GeodesicGridPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            MaterialPlugin::<GeodesicGridMaterial>::default(),
            EllipsoidBillboardPlugin::<GeodesicGridMaterial>::default(),
        ));
        load_internal_asset!(
            app,
            GEODESIC_GRID_SHADER,
            "../assets/geodesic_grid.wgsl",
            Shader::from_wgsl
        );
    }
}
