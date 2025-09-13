use bevy::{
    asset::{load_internal_asset, uuid_handle},
    camera::visibility::RenderLayers,
    picking::{
        Pickable, PickingSystems,
        backend::{HitData, PointerHits, ray::RayMap},
    },
    // math::FloatOrd,
    prelude::*,
    reflect::TypePath,
    render::render_resource::AsBindGroup,
    shader::ShaderRef,
};

pub mod geodesic_grid;
pub mod icosahedron;
pub mod tiles;

pub fn ray_ellipsoid_intersection(
    ray: Ray3d,
    ellipsoid_inverse_scale_and_rotation: &Mat3A,
    ellipsoid_translation: &Vec3A,
) -> Option<(f32, Vec3A, Vec3A)> {
    let o =
        *ellipsoid_inverse_scale_and_rotation * (Vec3A::from(ray.origin) - ellipsoid_translation);
    let mut r = *ellipsoid_inverse_scale_and_rotation * Vec3A::from(ray.direction.as_vec3());
    let a = r.dot(r);
    let b = -r.dot(o);
    let c = o.dot(o) - 1.0;
    let mut d = (b * b) - a * c;
    if d < 0.0 || b - d < 0.0 {
        return None;
    }
    d = d.sqrt();
    r = r / a;
    let local_position = o + r * (b - d);
    let world_normal =
        (ellipsoid_inverse_scale_and_rotation.transpose() * local_position).normalize();
    return Some((b - d, local_position, world_normal));
}

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
#[bind_group_data(GeodesicGridMaterialKey)]
pub struct GeodesicGridMaterial {
    #[texture(0, sample_type = "u_int")]
    pub selection_texture: Handle<Image>,
}

const GEODESIC_GRID_SHADER_LIB: Handle<Shader> =
    uuid_handle!("139ad40b-cae2-4eba-9c5c-4d69b5df2b23");

impl Material for GeodesicGridMaterial {
    fn vertex_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER_LIB.into()
    }

    fn fragment_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER_LIB.into()
    }

    fn prepass_vertex_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER_LIB.into()
    }

    fn prepass_fragment_shader() -> ShaderRef {
        GEODESIC_GRID_SHADER_LIB.into()
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
        app.add_plugins(MaterialPlugin::<GeodesicGridMaterial>::default())
            .add_systems(PreUpdate, update_hits.in_set(PickingSystems::Backend));
        load_internal_asset!(
            app,
            GEODESIC_GRID_SHADER_LIB,
            "../assets/geodesic_grid.wgsl",
            Shader::from_wgsl
        );
    }
}

fn update_hits(
    ray_map: Res<RayMap>,
    picking_cameras: Query<(&Camera, Option<&RenderLayers>)>,
    pickables: Query<&Pickable>,
    ellipsoid_billboards: Query<(
        Entity,
        &GlobalTransform,
        &MeshMaterial3d<GeodesicGridMaterial>,
    )>,
    layers: Query<&RenderLayers>,
    mut output: MessageWriter<PointerHits>,
) {
    for (&ray_id, &ray) in ray_map.iter() {
        let Ok((camera, cam_layers)) = picking_cameras.get(ray_id.camera) else {
            continue;
        };
        let cam_layers = cam_layers.to_owned().unwrap_or_default();
        let mut picks = Vec::new();
        for (entity, global_transform, _material) in ellipsoid_billboards.iter() {
            let entity_layers = layers.get(entity).cloned().unwrap_or_default();
            let render_layers_match = cam_layers.intersects(&entity_layers);
            let is_pickable = pickables.get(entity).ok().is_none_or(|p| p.is_hoverable);
            if !render_layers_match || !is_pickable {
                continue;
            }
            let transform = global_transform.affine();
            let Some((distance, local_position, world_normal)) = ray_ellipsoid_intersection(
                ray,
                &transform.matrix3.inverse(),
                &transform.translation,
            ) else {
                continue;
            };
            let world_position = transform.matrix3 * local_position + transform.translation;
            picks.push((
                entity,
                HitData::new(
                    ray_id.camera,
                    distance,
                    Some(world_position.into()),
                    Some(world_normal.into()),
                ),
            ));
        }
        // picks.sort_by_key(|(_entity, hit)| FloatOrd(hit.depth));
        if !picks.is_empty() {
            output.write(PointerHits::new(ray_id.pointer, picks, camera.order as f32));
        }
    }
}
