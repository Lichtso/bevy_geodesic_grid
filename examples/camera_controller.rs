//! A freecam-style camera controller plugin.
//! To use in your own application:
//! - Copy the code for the [`CameraControllerPlugin`] and add the plugin to your App.
//! - Attach the [`CameraController`] component to an entity with a [`Camera3d`].
//!
//! Unlike other examples, which demonstrate an application, this demonstrates a plugin library.

use bevy::{
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit},
    prelude::*,
    window::{CursorGrabMode, CursorOptions},
};
use std::{f32::consts::*, ops::Range};

pub struct CameraControllerPlugin;

impl Plugin for CameraControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, run_camera_controller);
    }
}

#[derive(Component)]
pub struct CameraController {
    pub yaw_and_pitch_speed: f32,
    pub pitch_range: Range<f32>,
    pub distance_speed: f32,
    pub distance_range: Range<f32>,
}

impl Default for CameraController {
    fn default() -> Self {
        let pitch_limit = FRAC_PI_2 - 0.01;
        Self {
            yaw_and_pitch_speed: 0.0005,
            pitch_range: -pitch_limit..pitch_limit,
            distance_speed: 0.1,
            distance_range: 1.5..100.0,
        }
    }
}

fn run_camera_controller(
    mut windows: Query<(&Window, &mut CursorOptions)>,
    mut camera_and_settings: Single<(&mut Transform, &CameraController), With<Camera>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    accumulated_mouse_scroll: Res<AccumulatedMouseScroll>,
) {
    let (ref mut camera, ref camera_settings) = *camera_and_settings;
    let target = Vec3::ZERO;
    let mut distance = (target - camera.translation).length();
    let delta = mouse_motion.delta;
    let (mut yaw, mut pitch, _roll) = camera.rotation.to_euler(EulerRot::YXZ);
    if mouse_buttons.pressed(MouseButton::Middle) {
        let speed = distance.ln() * camera_settings.yaw_and_pitch_speed;
        pitch = (pitch - delta.y * speed).clamp(
            camera_settings.pitch_range.start,
            camera_settings.pitch_range.end,
        );
        yaw -= delta.x * speed;
        for (window, mut cursor_options) in &mut windows {
            if !window.focused {
                continue;
            }
            cursor_options.grab_mode = CursorGrabMode::Locked;
            cursor_options.visible = false;
        }
    } else {
        for (_, mut cursor_options) in &mut windows {
            cursor_options.grab_mode = CursorGrabMode::None;
            cursor_options.visible = true;
        }
    }
    camera.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
    let scroll_amount = match accumulated_mouse_scroll.unit {
        MouseScrollUnit::Line => accumulated_mouse_scroll.delta.y,
        MouseScrollUnit::Pixel => accumulated_mouse_scroll.delta.y / 16.0,
    };
    distance = (distance.ln() + scroll_amount * camera_settings.distance_speed)
        .exp()
        .clamp(
            camera_settings.distance_range.start,
            camera_settings.distance_range.end,
        );
    camera.translation = target - camera.forward() * distance;
}
