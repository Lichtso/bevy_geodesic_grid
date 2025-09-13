//! This example demonstrates the geodesic grid.

#[path = "camera_controller.rs"]
mod camera_controller;

use bevy::{
    animation::{AnimationTarget, AnimationTargetId, animated_field},
    asset::{RenderAssetUsages, uuid_handle},
    camera::primitives::Aabb,
    prelude::*,
    render::{
        // settings::{WgpuSettings, PowerPreference},
        render_resource::{Extent3d, TextureDimension, TextureFormat},
    },
    // window::PresentMode,
};
use bevy_ellipsoid_billboard::EllipsoidBillboardPlugin;
use bevy_geodesic_grid::{
    GeodesicGridMaterial,
    GeodesicGridPlugin,
    geodesic_grid::GeodesicGrid,
    icosahedron::IcosahedronCoordinates,
    // tiles::{Tile, TileSet},
};
use camera_controller::{CameraController, CameraControllerPlugin};
use std::f32::consts::PI;

/*
- Start with cartesian polygon
- Solve smallest-circle problem to find the altitude
- IcosahedronCoordinates::from_cartesian().into_tile()
- GeodesicGrid::tiles_of_spherical_polygon()
- GeodesicGrid::flood_fill()
*/

const SELECTION_TEXTURE: Handle<Image> = uuid_handle!("9e40d4b6-bd1d-438d-8cc4-48dac07a0bf3");

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
            EllipsoidBillboardPlugin,
            GeodesicGridPlugin,
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut geodesic_materials: ResMut<Assets<GeodesicGridMaterial>>,
    mut animation_clips: ResMut<Assets<AnimationClip>>,
    mut animation_graphs: ResMut<Assets<AnimationGraph>>,
    mut images: ResMut<Assets<Image>>,
) {
    commands.spawn((
        Camera3d::default(),
        /*Projection::from(OrthographicProjection {
            scaling_mode: bevy::render::camera::ScalingMode::FixedVertical {
                viewport_height: 5.0,
            },
            ..OrthographicProjection::default_3d()
        }),*/
        Transform::from_xyz(0., 1.5, 6.).looking_at(Vec3::ZERO, Vec3::Y),
        CameraController::default(),
    ));

    if true {
        commands.spawn((
            PointLight {
                shadows_enabled: true,
                shadow_depth_bias: 0.01,
                shadow_normal_bias: 2.0,
                shadow_map_near_z: 1.0,
                // soft_shadows_enabled: true, // "experimental_pbr_pcss"
                ..default()
            },
            Transform::from_xyz(2.0, 4.0, 2.0),
        ));
    } else {
        commands.spawn((
            DirectionalLight {
                shadows_enabled: true,
                shadow_depth_bias: 0.01,
                shadow_normal_bias: 2.0,
                // soft_shadows_enabled: true, // "experimental_pbr_pcss"
                ..default()
            },
            Transform::from_xyz(2.0, 4.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
        ));
    }

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(5.0, 5.0))),
        Transform::default(),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 0.5).mesh())),
        Transform::from_xyz(0.75, 1.0, 0.75),
        MeshMaterial3d(materials.add(Color::srgb(0.5, 0.3, 0.3))),
    ));

    let geodesic_name = Name::new("geodesic");
    let mut animation_clip = AnimationClip::default();
    animation_clip.add_curve_to_target(
        AnimationTargetId::from_name(&geodesic_name),
        AnimatableCurve::new(
            animated_field!(Transform::rotation),
            UnevenSampleAutoCurve::new([0.0, 2.0, 4.0, 6.0].into_iter().zip([
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

    let image = Image::new_fill(
        Extent3d {
            width: 100,
            height: 20,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &[0u8],
        TextureFormat::R8Uint,
        RenderAssetUsages::RENDER_WORLD,
    );
    images.insert(SELECTION_TEXTURE.id(), image).unwrap();

    let geodesic_entity = commands
        .spawn((
            Mesh3d(meshes.add(Plane3d::default().mesh().size(2.0, 2.0))),
            MeshMaterial3d(geodesic_materials.add(GeodesicGridMaterial {
                selection_texture: SELECTION_TEXTURE.into(),
            })),
            Transform::from_xyz(0.0, 1.0, 0.0).with_scale(Vec3::new(1.0, 1.0, 1.0)),
            Aabb {
                center: Vec3A::ZERO,
                half_extents: Vec3A::splat(1.0),
            },
            geodesic_name.clone(),
            AnimationGraphHandle(animation_graphs.add(animation_graph)),
            player,
        ))
        .observe(
            |pointer_move: On<Pointer<Move>>,
             global_transforms: Query<&GlobalTransform>,
             mut images: ResMut<Assets<Image>>,
             mut materials: ResMut<Assets<GeodesicGridMaterial>>| {
                let Some(world_position) = pointer_move.event().event.hit.position else {
                    return;
                };
                let Ok(global_transform) = global_transforms.get(pointer_move.event().entity)
                else {
                    return;
                };
                let transform = global_transform.affine();
                let local_position = Vec3::from(
                    transform.matrix3.inverse()
                        * (Vec3A::from(world_position) - transform.translation),
                );
                let icosahedron_coordinate = IcosahedronCoordinates::from_cartesian(local_position);
                let grid = GeodesicGrid::new(10);
                let tile = icosahedron_coordinate.into_tile(&grid);
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
                if let Some(tiles) = grid.tiles_of_spherical_polygon(&[tile, [13, 17], [23, 12]]) {
                    for [x, y] in tiles.iter() {
                        if let Some(pixel_bytes) =
                            image.pixel_bytes_mut(UVec3::new(*x as u32, *y as u32, 0))
                        {
                            pixel_bytes[0] = 1;
                        }
                    }
                }
                /*if let Some(tiles) = grid.tiles_of_great_circle_arc(tile, [23, 12]) {
                    for [x, y] in tiles.iter() {
                        if let Some(pixel_bytes) =
                            image.pixel_bytes_mut(UVec3::new(*x as u32, *y as u32, 0))
                        {
                            pixel_bytes[0] = 1;
                        }
                    }
                }*/
                /*let (neighbor_tiles, neighbor_edges) = grid.tile_neighbors(tile);
                for i in 0..3 {
                    if let Some(pixel_bytes) = image.pixel_bytes_mut(UVec3::new(
                        neighbor_tiles[i][0] as u32,
                        neighbor_tiles[i][1] as u32,
                        0,
                    )) {
                        pixel_bytes[0] = 1 + neighbor_edges[i] as u8;
                    }
                }*/
                images.insert(SELECTION_TEXTURE.id(), image).unwrap();
                for (_handle, material) in materials.iter_mut() {
                    material.selection_texture = SELECTION_TEXTURE.into();
                }
            },
        )
        .id();
    if false {
        commands.entity(geodesic_entity).insert(AnimationTarget {
            id: AnimationTargetId::from_name(&geodesic_name),
            player: geodesic_entity,
        });
    }
}
