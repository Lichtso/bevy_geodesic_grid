//! This example demonstrates the geodesic grid.

#[path = "camera_controller.rs"]
mod camera_controller;

use bevy::{
    animation::{AnimationTarget, AnimationTargetId, animated_field},
    asset::RenderAssetUsages,
    camera::primitives::Aabb,
    light::{AmbientLight, DirectionalLightShadowMap},
    picking::Pickable,
    // window::PresentMode,
    prelude::*,
    render::{
        // settings::{WgpuSettings, PowerPreference},
        render_resource::{Extent3d, TextureDimension, TextureFormat},
    },
};
use bevy_ellipsoid_billboard::ELLIPSOID_BILLBOARD_MESH;
use bevy_geodesic_grid::{
    GeodesicGridMaterial, GeodesicGridPlugin,
    geodesic_grid::GeodesicGrid,
    icosahedron::IcosahedronCoordinates,
    surface_structures::{ArcSegment, SphericalCircle},
    tiles::TileSet,
};
use camera_controller::{CameraController, CameraControllerPlugin};
use opensimplex_noise_rs::OpenSimplexNoise;
use std::f32::consts::PI;

#[derive(Component)]
struct Draggable {
    base_polygon: Vec<Vec2>,
    // Solve smallest-circle problem: https://people.inf.ethz.ch/gaertner/subdir/software/miniball.html
    base_center: Vec3,
    base_radius: f32,
    possible_underground: u64,
    is_valid_drop: bool,
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                /*primary_window: Some(Window {
                    present_mode: bevy::window::PresentMode::AutoNoVsync,
                    mode: bevy::window::WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                    ..default()
                }),*/
                // exit_condition: ExitCondition::DontExit,
                ..default()
            }),
            /*.set(RenderPlugin {
                render_creation: WgpuSettings {
                    power_preference: PowerPreference::HighPerformance,
                    ..default()
                }
                .into(),
                ..default()
            })*/
            CameraControllerPlugin,
            GeodesicGridPlugin,
        ))
        .add_systems(Startup, (setup, setup_world))
        .insert_resource(ClearColor(Color::srgb(0.0, 0.0, 0.0)))
        .insert_resource(DirectionalLightShadowMap { size: 4096 })
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 10.0,
            ..AmbientLight::default()
        })
        .run();
}

struct Noise {
    octaves: Vec<OpenSimplexNoise>,
    frequency_factor: f32,
    base_power: f32,
}

impl Noise {
    fn new(seed: i64, octave_count: usize, frequency_factor: f32, base_power: f32) -> Self {
        Self {
            octaves: (0..octave_count)
                .map(|octave_index| OpenSimplexNoise::new(Some(seed + octave_index as i64)))
                .collect::<Vec<_>>(),
            frequency_factor,
            base_power: base_power + 1.0,
        }
    }

    fn sample(&self, cartesian: Vec3) -> f32 {
        self.octaves
            .iter()
            .enumerate()
            .fold(0.0, |acc, (octave_index, noise)| {
                let radius = self.frequency_factor as f64 * (1 << octave_index) as f64;
                acc + noise.eval_3d(
                    cartesian[0] as f64 * radius,
                    cartesian[1] as f64 * radius,
                    cartesian[2] as f64 * radius,
                ) as f32
                    / (self.base_power + octave_index as f32)
            })
    }
}

#[derive(Component)]
struct Lithosphere {
    grid: GeodesicGrid,
    seed: i64,
    water_level: f32,
    temperature: f32,
    percipitation: f32,
    terrain: Vec<u8>,
}

impl Lithosphere {
    fn generate_terrain(&mut self) {
        let elevation_noise = Noise::new(self.seed, 4, 2.0, 0.0);
        let shore_noise = Noise::new(self.seed, 1, 32.0, 4.0);
        let temperature_noise = Noise::new(self.seed + 5, 2, 4.0, 0.0);
        let percipitation_noise = Noise::new(self.seed + 7, 1, 1.0, 0.0);
        let mut terrain = vec![[0u8; 3]; self.grid.width() * self.grid.height()];
        for y in 0..self.grid.height() {
            for x in 0..self.grid.width() {
                let cartesian =
                    IcosahedronCoordinates::from_tile([x, y], &self.grid).into_cartesian();
                let sun_intensity = 1.0 - cartesian[1].abs() as f32;
                let elevation = 1.0 - self.water_level + elevation_noise.sample(cartesian);
                terrain[y * self.grid.width() + x] = [
                    (((elevation + shore_noise.sample(cartesian) - 0.2).max(0.0) * 6.0) as u8)
                        .min(2),
                    (sun_intensity * self.temperature + temperature_noise.sample(cartesian) * 1.0)
                        .min(5.0)
                        .max(0.0) as u8,
                    (self.percipitation - elevation * 3.5
                        + percipitation_noise.sample(cartesian) * 1.0)
                        .min(3.9) as u8,
                ];
            }
        }
        self.terrain = vec![0; self.grid.width() * self.grid.height()];
        for y in 0..self.grid.height() {
            for x in 0..self.grid.width() {
                let (neighbors, _edges) = self.grid.tile_neighbors([x, y]);
                let mut smoothed = [0u8; 3];
                for channel in 0..3 {
                    let neighbor_sum = terrain
                        [neighbors[0][1] * self.grid.width() + neighbors[0][0]][channel]
                        + terrain[neighbors[1][1] * self.grid.width() + neighbors[1][0]][channel]
                        + terrain[neighbors[2][1] * self.grid.width() + neighbors[2][0]][channel];
                    smoothed[channel] = (terrain[y * self.grid.width() + x][channel]
                        - neighbor_sum / 3
                        + neighbor_sum)
                        / 3;
                }
                let [elevation, temperature, percipitation] = smoothed;
                self.terrain[y * self.grid.width() + x] = if elevation < 2 {
                    elevation
                } else if temperature < 2 {
                    2 + temperature
                } else if percipitation < 1 {
                    4 + (temperature - 2) / 2
                } else if percipitation < 2 {
                    6
                } else if temperature < 4 {
                    7 + temperature - 2
                } else {
                    9 + percipitation - 2
                };
            }
        }
    }
}

fn setup(
    asset_server: Res<AssetServer>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut geodesic_materials: ResMut<Assets<GeodesicGridMaterial>>,
    mut animation_clips: ResMut<Assets<AnimationClip>>,
    mut animation_graphs: ResMut<Assets<AnimationGraph>>,
    mut images: ResMut<Assets<Image>>,
) {
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            shadow_depth_bias: 0.01,
            shadow_normal_bias: 2.0,
            ..default()
        },
        Transform::from_xyz(2.0, 0.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let planet_name = Name::new("planet");
    let mut animation_clip = AnimationClip::default();
    animation_clip.add_curve_to_target(
        AnimationTargetId::from_name(&planet_name),
        AnimatableCurve::new(
            animated_field!(Transform::rotation),
            UnevenSampleAutoCurve::new([0.0, 20.0, 40.0, 60.0].into_iter().zip([
                Quat::IDENTITY,
                Quat::from_axis_angle(Vec3::Y, PI * 2.0 / 3.0),
                Quat::from_axis_angle(Vec3::Y, PI * 4.0 / 3.0),
                Quat::IDENTITY,
            ]))
            .unwrap(),
        ),
    );
    let (animation_graph, animation_index) =
        AnimationGraph::from_clip(animation_clips.add(animation_clip));
    let mut player = AnimationPlayer::default();
    player.play(animation_index).repeat();

    let mut lithosphere = Lithosphere {
        grid: GeodesicGrid::new(128),
        seed: 0,
        water_level: 0.5,
        temperature: 6.0,
        percipitation: 5.0,
        terrain: Vec::default(),
    };
    lithosphere.generate_terrain();
    let palette_image = asset_server.load("palette.png");
    let lithosphere_material =
        GeodesicGridMaterial::new(&mut images, &lithosphere.grid, palette_image);
    lithosphere_material.render_tiles(&mut images, lithosphere.terrain.clone(), false);
    let grid = lithosphere.grid;
    let lithosphere_entity = commands
        .spawn((
            lithosphere,
            Mesh3d(ELLIPSOID_BILLBOARD_MESH),
            MeshMaterial3d(geodesic_materials.add(lithosphere_material.clone())),
            Transform::from_xyz(0.0, 0.0, 0.0).with_scale(Vec3::new(1.0, 1.0, 1.0)),
            Aabb {
                center: Vec3A::ZERO,
                half_extents: Vec3A::splat(1.0),
            },
            planet_name.clone(),
            AnimationGraphHandle(animation_graphs.add(animation_graph)),
            player,
        ))
        .observe(building_planet_drag_leave)
        .observe(building_planet_drag_over)
        .with_children(|parent| {
            let mut first_great_circle = ArcSegment::great_circle_from_two_points(
                vec3(2.0, -1.0, 3.0).normalize() * 1.02,
                vec3(3.0, 2.0, -2.0).normalize() * 1.02,
            );
            let mut second_great_circle = ArcSegment::great_circle_from_two_points(
                vec3(3.0, 2.0, -2.0).normalize() * 1.01,
                vec3(0.0, -1.0, -2.0).normalize() * 1.01,
            );
            let small_circle = ArcSegment::small_circle_from_great_circle_crossing(
                &first_great_circle,
                &second_great_circle,
                0.05 * PI,
                false,
            );
            first_great_circle.set_end_cartesian(small_circle.begin_cartesian());
            second_great_circle.set_begin_cartesian(small_circle.end_cartesian());
            let asphalt = materials.add(StandardMaterial {
                base_color: Color::srgb(0.01, 0.01, 0.01),
                perceptual_roughness: 0.95,
                metallic: 0.5,
                ..StandardMaterial::default()
            });
            let road_width = 0.005;
            let lithosphere_radius = 1.0;
            let (first_great_circle_spherical_polygon, first_great_circle_mesh) =
                first_great_circle.spherical_polygon_and_mesh(
                    20,
                    road_width,
                    lithosphere_radius,
                    &grid,
                );
            let (second_great_circle_spherical_polygon, second_great_circle_mesh) =
                second_great_circle.spherical_polygon_and_mesh(
                    20,
                    road_width,
                    lithosphere_radius,
                    &grid,
                );
            let (small_circle_spherical_polygon, small_circle_mesh) =
                small_circle.spherical_polygon_and_mesh(20, road_width, lithosphere_radius, &grid);
            parent.spawn((
                Mesh3d(meshes.add(first_great_circle_mesh)),
                Transform::from_xyz(0.0, 0.0, 0.0),
                MeshMaterial3d(asphalt.clone()),
            ));
            parent.spawn((
                Mesh3d(meshes.add(second_great_circle_mesh)),
                Transform::from_xyz(0.0, 0.0, 0.0),
                MeshMaterial3d(asphalt.clone()),
            ));
            parent.spawn((
                Mesh3d(meshes.add(small_circle_mesh)),
                Transform::from_xyz(0.0, 0.0, 0.0),
                MeshMaterial3d(asphalt),
            ));
            let mut tile_set = TileSet::new(grid);
            grid.tiles_in_spherical_polygon(&first_great_circle_spherical_polygon, &mut tile_set);
            grid.tiles_of_spherical_polygon(&first_great_circle_spherical_polygon, &mut tile_set);
            grid.tiles_in_spherical_polygon(&second_great_circle_spherical_polygon, &mut tile_set);
            grid.tiles_of_spherical_polygon(&second_great_circle_spherical_polygon, &mut tile_set);
            grid.tiles_in_spherical_polygon(&small_circle_spherical_polygon, &mut tile_set);
            grid.tiles_of_spherical_polygon(&small_circle_spherical_polygon, &mut tile_set);
            let mut image = Image::new_fill(
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
            for [x, y] in tile_set.iter() {
                let (neighbor_tiles, _edges) = grid.tile_neighbors([x, y]);
                if let Some(pixel_bytes) = image.pixel_bytes_mut(UVec3::new(x as u32, y as u32, 0))
                {
                    pixel_bytes[0] = 1
                        | ((!tile_set.contains(neighbor_tiles[0]) as u8) << 1)
                        | ((!tile_set.contains(neighbor_tiles[1]) as u8) << 2)
                        | ((!tile_set.contains(neighbor_tiles[2]) as u8) << 3);
                }
            }
            images
                .insert(lithosphere_material.selection_texture.id(), image)
                .unwrap();
            /*let marker = SphericalCircle::from_radius(
                1.0,
                Dir3::new(small_circle.end_cartesian()).unwrap(),
                0.01,
            );
            parent.spawn((
                Mesh3d(meshes.add(Circle::new(marker.radius()).mesh())),
                Transform::from_matrix(
                    marker.polar_orientation()
                        * Mat4::from_translation(vec3(0.0, 0.01, 0.0))
                        * Mat4::from_rotation_x(-0.5 * PI),
                ),
                MeshMaterial3d(materials.add(Color::srgb(0.1, 0.02, 0.02))),
            ));*/
            parent
                .spawn((
                    CameraController::default(),
                    Camera3d::default(),
                    /*Projection::from(OrthographicProjection {
                        scaling_mode: bevy::render::camera::ScalingMode::FixedVertical {
                            viewport_height: 5.0,
                        },
                        ..OrthographicProjection::default_3d()
                    }),*/
                    Transform::from_xyz(0.0, 0.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
                ))
                .with_children(|parent| {
                    parent
                        .spawn((
                            Mesh3d(meshes.add(Cuboid::new(0.25, 0.25, 0.25).mesh())),
                            Transform::from_xyz(-1.0, 0.0, -2.0),
                            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.3, 0.3))),
                        ))
                        .observe(building_pallet_drag_start);
                });
        })
        .id();
    commands.entity(lithosphere_entity).insert(AnimationTarget {
        id: AnimationTargetId::from_name(&planet_name),
        player: lithosphere_entity,
    });
}

fn building_pallet_drag_start(
    pointer_drag_start: On<Pointer<DragStart>>,
    mut commands: Commands,
    aabbs: Query<&Aabb>,
) {
    let Ok(aabb) = aabbs.get(pointer_drag_start.event().entity) else {
        return;
    };
    let base_polygon = vec![
        vec2(aabb.half_extents.x, aabb.half_extents.z),
        vec2(aabb.half_extents.x, -aabb.half_extents.z),
        vec2(-aabb.half_extents.x, -aabb.half_extents.z),
        vec2(-aabb.half_extents.x, aabb.half_extents.z),
    ];
    let base_radius = base_polygon[0].length();
    commands
        .entity(pointer_drag_start.event().entity)
        .clone_and_spawn_with_opt_out(|builder| {
            builder.deny::<ChildOf>();
        })
        .insert((
            Draggable {
                base_polygon,
                base_center: aabb.center.to_vec3() - vec3(0.0, aabb.half_extents.y, 0.0),
                base_radius,
                possible_underground: (1 << 4) | (1 << 5) | (1 << 6),
                is_valid_drop: false,
            },
            Pickable {
                should_block_lower: false,
                is_hoverable: false,
            },
        ));
}

fn building_planet_drag_leave(
    _pointer_drag_leave: On<Pointer<DragLeave>>,
    mut draggable: Single<&mut Draggable>,
) {
    draggable.is_valid_drop = false;
}

fn building_planet_drag_over(
    pointer_drag_over: On<Pointer<DragOver>>,
    lithospheres: Query<(
        &Lithosphere,
        &MeshMaterial3d<GeodesicGridMaterial>,
        &GlobalTransform,
        &Aabb,
    )>,
    mut commands: Commands,
    mut draggable: Single<(&mut Draggable, &mut Transform, Entity)>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<GeodesicGridMaterial>>,
) {
    let Some(world_position) = pointer_drag_over.event().event.hit.position else {
        return;
    };
    let lithosphere_entity = pointer_drag_over.event().entity;
    let Ok((
        lithosphere,
        lithosphere_mesh_material,
        lithosphere_global_transform,
        lithosphere_aabb,
    )) = lithospheres.get(lithosphere_entity)
    else {
        return;
    };
    let lithosphere_radius = lithosphere_aabb.half_extents.x;
    let lithosphere_affine_transform = lithosphere_global_transform.affine();
    let surface_normal = Dir3::new(
        (lithosphere_affine_transform.matrix3.inverse()
            * (Vec3A::from(world_position) - lithosphere_affine_transform.translation))
            .into(),
    )
    .unwrap();
    // surface_normal = IcosahedronCoordinates::from_cartesian(surface_normal).tile_midpoint_corners_and_offset(&lithosphere.grid).0.into_cartesian();
    let draggable_transform = Mat4::from_translation(-surface_normal * draggable.0.base_center.y)
        * SphericalCircle::from_radius(lithosphere_radius, surface_normal, draggable.0.base_radius)
            .polar_orientation();
    commands
        .entity(lithosphere_entity)
        .add_children(&[draggable.2]);
    let mut image = Image::new_fill(
        Extent3d {
            width: lithosphere.grid.width() as u32,
            height: lithosphere.grid.height() as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &[0u8],
        TextureFormat::R8Uint,
        RenderAssetUsages::RENDER_WORLD,
    );
    let spherical_polygon = draggable
        .0
        .base_polygon
        .iter()
        .map(|vertex| {
            let global_position = draggable_transform
                .transform_point(draggable.0.base_center + Vec3::new(vertex.x, 0.0, vertex.y))
                .normalize();
            (
                global_position,
                IcosahedronCoordinates::from_cartesian(global_position)
                    .into_tile(&lithosphere.grid),
            )
        })
        .collect::<Vec<_>>();
    let mut tile_set = TileSet::new(lithosphere.grid);
    lithosphere
        .grid
        .tiles_in_spherical_polygon(&spherical_polygon, &mut tile_set);
    lithosphere
        .grid
        .tiles_of_spherical_polygon(&spherical_polygon, &mut tile_set);
    draggable.0.is_valid_drop = true;
    for [x, y] in tile_set.iter() {
        let (neighbor_tiles, _edges) = lithosphere.grid.tile_neighbors([x, y]);
        if (1 << lithosphere.terrain[y * lithosphere.grid.width() + x])
            & draggable.0.possible_underground
            == 0
        {
            draggable.0.is_valid_drop = false;
        }
        if let Some(pixel_bytes) = image.pixel_bytes_mut(UVec3::new(x as u32, y as u32, 0)) {
            pixel_bytes[0] = 1
                | ((!tile_set.contains(neighbor_tiles[0]) as u8) << 1)
                | ((!tile_set.contains(neighbor_tiles[1]) as u8) << 2)
                | ((!tile_set.contains(neighbor_tiles[2]) as u8) << 3);
        }
    }
    let lithosphere_material = materials.get_mut(lithosphere_mesh_material).unwrap();
    images
        .insert(lithosphere_material.selection_texture.id(), image)
        .unwrap();
    if draggable.0.is_valid_drop {
        *draggable.1 = Transform::from_matrix(draggable_transform);
    }
}

fn setup_world(world: &mut World) {
    world.add_observer(
        |mut pointer_drag_drop: On<Pointer<DragDrop>>,
         mut commands: Commands,
         draggable: Single<(&Draggable, Entity)>| {
            pointer_drag_drop.propagate(false);
            let mut entity_commands = commands.entity(draggable.1);
            if draggable.0.is_valid_drop {
                entity_commands.remove::<Draggable>();
            } else {
                entity_commands.despawn();
            }
        },
    );
}
