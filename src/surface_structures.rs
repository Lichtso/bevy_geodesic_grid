use crate::{geodesic_grid::GeodesicGrid, icosahedron::IcosahedronCoordinates, tiles::Tile};
use bevy::{asset::RenderAssetUsages, mesh::PrimitiveTopology, prelude::*};
use std::f32::consts::PI;

#[derive(Debug)]
pub struct SphericalCircle {
    normal: Dir3,
    altitude: f32,
    radius: f32,
}

impl SphericalCircle {
    pub fn from_radius(sphere_radius: f32, normal: Dir3, radius: f32) -> Self {
        debug_assert!(radius <= sphere_radius);
        Self {
            normal,
            altitude: (sphere_radius * sphere_radius - radius * radius).sqrt(),
            radius,
        }
    }

    pub fn from_altitude(sphere_radius: f32, normal: Dir3, altitude: f32) -> Self {
        debug_assert!(altitude < sphere_radius);
        Self {
            normal,
            altitude,
            radius: (sphere_radius * sphere_radius - altitude * altitude).sqrt(),
        }
    }

    pub fn from_cone(sphere_radius: f32, cone_axis: Dir3, half_angle: f32) -> Self {
        Self::from_radius(sphere_radius, cone_axis, sphere_radius * half_angle.sin())
    }

    pub fn normal(&self) -> Dir3 {
        self.normal
    }

    pub fn altitude(&self) -> f32 {
        self.altitude
    }

    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub fn center(&self) -> Vec3 {
        self.normal * self.altitude
    }

    pub fn polar_orientation(&self) -> Mat4 {
        Mat4::from_rotation_translation(
            Quat::from_rotation_arc(Vec3::Y, self.normal.into())
                * Quat::from_axis_angle(Vec3::Y, self.normal.x.atan2(self.normal.z)),
            self.center(),
        )
    }
}

fn smooth_step(x: f32) -> f32 {
    // x = x * x * (3.0 - 2.0 * x)
    x * x * x * (x * (6.0 * x - 15.0) + 10.0)
}

fn signed_slerp(from: Quat, to: Quat, parameter: f32) -> Quat {
    // from.slerp(to, parameter)
    let theta = from.dot(to).acos();
    (from * ((1.0 - parameter) * theta).sin() + to * (parameter * theta).sin())
        * (1.0 / theta.sin())
}

#[derive(Debug)]
pub struct ArcSegment {
    begin_altitude: f32,
    end_altitude: f32,
    begin_cartesian: Dir3,
    end_cartesian: Dir3,
    circle_normal: Dir3,
}

impl ArcSegment {
    pub fn great_circle_from_two_points(begin_cartesian: Vec3, end_cartesian: Vec3) -> Self {
        Self {
            /*circle: SphericalCircle::from_radius(
                1.0,
                Dir3::new(end_cartesian.cross(begin_cartesian)).unwrap(),
                1.0,
            ),*/
            begin_altitude: begin_cartesian.length(),
            end_altitude: end_cartesian.length(),
            begin_cartesian: Dir3::new(begin_cartesian).unwrap(),
            end_cartesian: Dir3::new(end_cartesian).unwrap(),
            circle_normal: Dir3::new(end_cartesian.cross(begin_cartesian)).unwrap(),
        }
    }

    pub fn great_circle_from_antipodes(great_circle_normal: Dir3, begin_cartesian: Vec3) -> Self {
        Self {
            // circle: SphericalCircle::from_radius(1.0, great_circle_normal, 1.0),
            begin_altitude: begin_cartesian.length(),
            end_altitude: begin_cartesian.length(),
            begin_cartesian: Dir3::new(begin_cartesian).unwrap(),
            end_cartesian: -Dir3::new(begin_cartesian).unwrap(),
            circle_normal: great_circle_normal,
        }
    }

    pub fn small_circle_from_great_circle_crossing(
        first_great_circle_arc: &Self,
        second_great_circle_arc: &Self,
        cone_half_angle: f32,
        counter_clockwise: bool,
    ) -> Self {
        let first_normal = -Vec3::from(first_great_circle_arc.circle_normal);
        let second_normal = Vec3::from(second_great_circle_arc.circle_normal);
        let bisector = Dir3::new(second_normal - first_normal).unwrap();
        let extremum_axis = Dir3::new(first_normal.cross(second_normal)).unwrap();
        let dihedral_angle = (-first_normal).angle_between(second_normal);
        debug_assert!(dihedral_angle < 2.0 * (PI * 0.5 - cone_half_angle));
        let cos_alpha = cone_half_angle.sin()
            / if counter_clockwise {
                first_normal
            } else {
                second_normal
            }
            .dot(bisector.into());
        let sin_alpha = (1.0 - cos_alpha * cos_alpha).sqrt();
        let cone_axis = Dir3::new(bisector * cos_alpha + extremum_axis * sin_alpha).unwrap();
        let mut begin_cartesian =
            Dir3::new(cone_axis.cross(first_normal).cross(-first_normal)).unwrap();
        let mut end_cartesian =
            Dir3::new(cone_axis.cross(second_normal).cross(-second_normal)).unwrap();
        if counter_clockwise {
            std::mem::swap(&mut begin_cartesian, &mut end_cartesian);
        }
        // let cone_half_angle = Vec3::from(begin_cartesian).angle_between(cone_axis.into());
        Self {
            /*circle: SphericalCircle::from_cone(
                1.0,
                cone_axis,
                half_angle,
            ),*/
            begin_altitude: first_great_circle_arc
                .interpolate_altitude(first_great_circle_arc.parameter_at(begin_cartesian)),
            end_altitude: second_great_circle_arc
                .interpolate_altitude(second_great_circle_arc.parameter_at(end_cartesian)),
            begin_cartesian,
            end_cartesian,
            circle_normal: cone_axis,
        }
    }

    pub fn begin_cartesian(&self) -> (f32, Dir3) {
        (self.begin_altitude, self.begin_cartesian)
    }

    pub fn end_cartesian(&self) -> (f32, Dir3) {
        (self.end_altitude, self.end_cartesian)
    }

    pub fn set_begin_cartesian(&mut self, begin_cartesian: (f32, Dir3)) {
        (self.begin_altitude, self.begin_cartesian) = begin_cartesian;
    }

    pub fn set_end_cartesian(&mut self, end_cartesian: (f32, Dir3)) {
        (self.end_altitude, self.end_cartesian) = end_cartesian;
    }

    fn parameter_at(&self, cartesian: Dir3) -> f32 {
        Vec3::from(self.begin_cartesian).angle_between(cartesian.into())
            / Vec3::from(self.begin_cartesian).angle_between(self.end_cartesian.into())
    }

    fn interpolate_altitude(&self, parameter: f32) -> f32 {
        self.begin_altitude + (self.end_altitude - self.begin_altitude) * smooth_step(parameter)
    }

    pub fn spherical_polygon_and_mesh(
        &self,
        resolution: usize,
        width: f32,
        sphere_radius: f32,
        grid: &GeodesicGrid,
    ) -> (Vec<(Vec3, Tile)>, Mesh) {
        let circle_normal = Vec3::from(self.circle_normal);
        let begin_y_axis = Vec3::from(self.begin_cartesian);
        let end_y_axis = Vec3::from(self.end_cartesian);
        let begin_vertex = begin_y_axis - circle_normal * circle_normal.dot(begin_y_axis);
        let end_vertex = end_y_axis - circle_normal * circle_normal.dot(end_y_axis);
        let arc_length = begin_vertex.angle_between(end_vertex);
        let span = if begin_vertex.cross(end_vertex).dot(circle_normal) > 0.0 {
            Quat::from_axis_angle(-circle_normal, 2.0 * PI - arc_length)
        } else {
            Quat::from_axis_angle(-circle_normal, arc_length)
        };
        let mut vertices = Vec::with_capacity(resolution * 2);
        let mut normals = Vec::with_capacity(resolution * 2);
        let mut spherical_polygon = Vec::with_capacity(resolution * 2);
        for segment_index in 0..resolution {
            let parameter = segment_index as f32 / (resolution - 1) as f32;
            let y_axis = signed_slerp(Quat::IDENTITY, span, parameter) * begin_y_axis;
            let z_axis = circle_normal.cross(y_axis).normalize();
            let x_axis = y_axis.cross(z_axis).normalize();
            let frame = Mat3 {
                x_axis,
                y_axis,
                z_axis,
            };
            for side_index in 0..2 {
                let side = (side_index as isize * 2 - 1) as f32;
                let cartesian =
                    frame * vec3(side * width, self.interpolate_altitude(parameter), 0.0);
                vertices.push(cartesian);
                normals.push(cartesian.normalize());
                spherical_polygon.push(frame * vec3(side * width, sphere_radius, 0.0).normalize());
            }
        }
        let spherical_polygon = spherical_polygon
            .iter()
            .step_by(2)
            .rev()
            .chain(spherical_polygon.iter().skip(1).step_by(2))
            .map(|cartesian| {
                let icosahedron_coordinate = IcosahedronCoordinates::from_cartesian(*cartesian);
                let tile = IcosahedronCoordinates::into_tile(&icosahedron_coordinate, grid);
                (*cartesian, tile)
            })
            .collect::<Vec<_>>();
        let mesh = Mesh::new(
            PrimitiveTopology::TriangleStrip,
            RenderAssetUsages::RENDER_WORLD,
        )
        // .with_inserted_indices(Indices::U16(indices))
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        (spherical_polygon, mesh)
    }
}
