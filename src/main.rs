use std::{f32::consts::PI, iter::zip};

use macroquad::prelude::*;

struct Vert {
    w: f32,
    x: Vec3,
    v: Vec3,
}

#[derive(PartialEq)]
enum CstrKind {
    Distance(f32),
    Floor(f32),
    Bend(f32),
    Volume,
}

#[derive(PartialEq)]
enum CstrEqType {
    Equality,
    Inequality,
}

struct Cstr {
    inds: Vec<usize>,
    k: f32,
    kind: CstrKind,
    eq_type: CstrEqType,
}

fn dihedral_angle(p1: Vec3, p2: Vec3, p3: Vec3, p4: Vec3) -> f32 {
    let e = p2 - p1;
    let n1 = e.cross(p3 - p1).normalize();
    let n2 = (p4 - p1).cross(e).normalize();

    let sin = e.normalize().dot(n1.cross(n2));
    let cos = n1.dot(n2);

    sin.atan2(cos)
}

fn wrap_angle(a: f32) -> f32 {
    (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

const EPS: f32 = 0.001;
impl Cstr {
    fn project(&self, p: &mut [Vec3], verts: &[Vert]) {
        let c = match self.kind {
            CstrKind::Distance(d) => self._c_distance(d, p),
            CstrKind::Volume => unimplemented!(),
            CstrKind::Floor(h) => self._c_floor(h, p),
            CstrKind::Bend(phi) => self._c_bend(phi, p),
        };

        match self.eq_type {
            CstrEqType::Equality => {
                // println!("{}", c);
                if c.abs() < EPS {
                    return;
                }
            }
            CstrEqType::Inequality => {
                if c >= 0.0 {
                    return;
                }
            }
        }

        let grad_p_c = match self.kind {
            CstrKind::Distance(_) => self._grad_distance(p),
            CstrKind::Volume => unimplemented!(),
            CstrKind::Floor(_) => self._grad_floor(),
            CstrKind::Bend(phi) => self._grad_bend(phi, p),
        };

        let mut s_denom = 0.;
        for (i, ind) in self.inds.iter().enumerate() {
            s_denom += verts[*ind].w * grad_p_c[i].length().powi(2)
        }
        let s = c / s_denom;

        let k_ = 1.0 - (1.0 - self.k).powf(1.0 / SOLVER_ITERATIONS as f32);
        if matches!(self.kind, CstrKind::Distance(_)) {
            // println!("{}", k_);
        }

        for (i, ind) in self.inds.iter().enumerate() {
            let dp = -s * verts[*ind].w * grad_p_c[i];
            // println!("dp = {}", dp);
            p[*ind] += dp * k_;
        }
    }

    fn _c_bend(&self, phi: f32, p: &[Vec3]) -> f32 {
        assert_eq!(self.inds.len(), 4);
    
        let p1 = p[self.inds[0]];
        let p2 = p[self.inds[1]];
        let p3 = p[self.inds[2]];
        let p4 = p[self.inds[3]];
    
        wrap_angle(dihedral_angle(p1, p2, p3, p4) - phi)
    }

    // ai slop
    fn _grad_bend(&self, phi: f32, p: &[Vec3]) -> Vec<Vec3> {
        assert_eq!(self.inds.len(), 4);

        let eps = 1e-4;
        let mut grad = vec![Vec3::ZERO; 4];

        for i in 0..4 {
            for axis in 0..3 {
                let mut p_plus = p.to_vec();
                let mut p_minus = p.to_vec();

                p_plus[self.inds[i]][axis] += eps;
                p_minus[self.inds[i]][axis] -= eps;

                let c_plus = self._c_bend(phi, &p_plus);
                let c_minus = self._c_bend(phi, &p_minus);

                grad[i][axis] = wrap_angle(c_plus - c_minus) / (2.0 * eps);;
            }
        }

        grad
    }

    fn _c_floor(&self, h: f32, p: &[Vec3]) -> f32 {
        assert!(self.inds.len() == 1);
        p[self.inds[0]].y - h
    }
    fn _c_distance(&self, d: f32, p: &[Vec3]) -> f32 {
        assert!(self.inds.len() == 2);
        let a = p[self.inds[0]];
        let b = p[self.inds[1]];
        let dist = (a - b).length();
        dist - d
    }
    fn _grad_floor(&self) -> Vec<Vec3> {
        assert!(self.inds.len() == 1);
        vec![Vec3::Y]
    }
    fn _grad_distance(&self, p: &[Vec3]) -> Vec<Vec3> {
        assert!(self.inds.len() == 2);
        let p1 = p[self.inds[0]];
        let p2 = p[self.inds[1]];
        let n = (p1 - p2) / (p1 - p2).length();
        let gp1c = n;
        let gp2c = -n;
        vec![gp1c, gp2c]
    }
}

struct Body {
    verts: Vec<Vert>,
    cstrs: Vec<Cstr>,
}

const G: f32 = 9.8;
const K_DAMP: f32 = 0.5;
const SOLVER_ITERATIONS: usize = 2;
impl Body {
    fn step(&mut self, dt: f32) {
        let mut p = vec![Vec3::ZERO; self.verts.len()];
        for (vert, p_i) in zip(&mut self.verts, &mut p) {
            // apply gravity
            // possible to add other external forces in the future
            vert.v += -dt * Vec3::Y * G;
            // damping
            vert.v *= 0.99; //lol?
            // apply velocities
            *p_i = vert.x + dt * vert.v;
        }

        // fun starts here
        for _ in 0..SOLVER_ITERATIONS {
            for cstr in &self.cstrs {
                cstr.project(&mut p, &self.verts);
            }
            // println!("---------")
        }

        for (vert, p_i) in zip(&mut self.verts, &mut p) {
            vert.v = (*p_i - vert.x) / dt;
            vert.x = *p_i;
            // println!("{}", vert.x)
        }
    }
}

fn pyramid(half: f32, height: f32) -> Body {
    let pos = vec![
        Vec3::new(-half, 0.0, -half),
        Vec3::new(half, 0.0, -half),
        Vec3::new(half, 0.0, half),
        Vec3::new(-half, 0.0, half),
        Vec3::new(0.0, height, 0.0),
    ];

    // 4 base edges, 4 side edges, 2 base diagonals
    let edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (0, 4),
        (1, 4),
        (2, 4),
        (3, 4),
        (0, 2),
        (1, 3),
    ];

    let mut cstrs: Vec<Cstr> = edges
        .iter()
        .map(|&(i, j)| Cstr {
            inds: vec![i, j],
            k: 1.0,
            kind: CstrKind::Distance((pos[i] - pos[j]).length()),
            eq_type: CstrEqType::Equality,
        })
        .collect();

    let mut floor_cstrs: Vec<Cstr> = (0..pos.len())
        .map(|i| Cstr {
            inds: vec![i],
            k: 1.0,
            kind: CstrKind::Floor(-3.0),
            eq_type: CstrEqType::Inequality,
        })
        .collect();

    cstrs.append(&mut floor_cstrs);

    let verts: Vec<Vert> = pos
        .into_iter()
        .map(|pos| Vert {
            w: 1.0,
            x: pos,
            v: Vec3::ZERO,
        })
        .collect();

    Body { verts, cstrs }
}

fn donut(big_r: f32, small_r: f32, n: usize, m: usize) -> Body {
    // n segments around the ring, m segments around the tube
    let mut pos = Vec::with_capacity(n * m);
    for i in 0..n {
        let u = std::f32::consts::TAU * i as f32 / n as f32;
        for j in 0..m {
            let v = std::f32::consts::TAU * j as f32 / m as f32;
            let r = big_r + small_r * v.cos();
            pos.push(Vec3::new(r * u.cos(), small_r * v.sin(), r * u.sin()));
        }
    }

    let idx = |i: usize, j: usize| (i % n) * m + (j % m);
    let mut edges = Vec::new();
    for i in 0..n {
        for j in 0..m {
            let a = idx(i, j);
            edges.push((a, idx(i, j + 1))); // around the tube
            edges.push((a, idx(i + 1, j))); // around the ring
            edges.push((a, idx(i + 1, j + 1))); // diagonal, resists shearing
        }
    }

    let mut cstrs: Vec<Cstr> = edges
        .iter()
        .map(|&(a, b)| Cstr {
            inds: vec![a, b],
            k: 0.5,
            kind: CstrKind::Distance((pos[a] - pos[b]).length()),
            eq_type: CstrEqType::Equality,
        })
        .collect();

    let mut floor_cstrs: Vec<Cstr> = (0..pos.len())
        .map(|i| Cstr {
            inds: vec![i],
            k: 1.0,
            kind: CstrKind::Floor(-3.0),
            eq_type: CstrEqType::Inequality,
        })
        .collect();

    cstrs.append(&mut floor_cstrs);
    let mut bend_cstrs = Vec::new();
    
    let mut add_bend = |inds: [usize; 4]| {
        let phi = dihedral_angle(
            pos[inds[0]],
            pos[inds[1]],
            pos[inds[2]],
            pos[inds[3]],
        );
    
        bend_cstrs.push(Cstr {
            inds: inds.to_vec(),
            k: 0.3,
            kind: CstrKind::Bend(phi),
            eq_type: CstrEqType::Equality,
        });
    };
    
    for i in 0..n {
        for j in 0..m {
            let a = idx(i, j);
            let b = idx(i + 1, j);
            let c = idx(i, j + 1);
            let d = idx(i + 1, j + 1);
    
            // Diagonal
            add_bend([a, d, b, c]);
    
            // Ring direction
            add_bend([a, b, d, idx(i, j.wrapping_sub(1))]);
    
            // Tube direction
            add_bend([a, c, d, idx(i.wrapping_sub(1), j)]);
        }
    }
    
    cstrs.append(&mut bend_cstrs);
    
    let verts: Vec<Vert> = pos
        .into_iter()
        .map(|pos| Vert {
            w: 1.0,
            x: pos,
            v: Vec3::ZERO,
        })
        .collect();

    Body { verts, cstrs }
}

fn draw_body(b: &Body) {
    for vert in &b.verts {
        draw_sphere(vert.x, 0.01, None, RED);
    }
    for cstr in &b.cstrs {
        if cstr.inds.len() == 2 && matches!(cstr.kind, CstrKind::Distance(_)) {
            let v1 = b.verts[cstr.inds[0]].x;
            let v2 = b.verts[cstr.inds[1]].x;

            draw_line_3d(v1, v2, BLUE);
        }
    }
}

fn rotate(body: &mut Body, rotation: Quat) {
    for vert in &mut body.verts {
        vert.x = rotation * vert.x;
        vert.v = rotation * vert.v;
    }
}

const MOVE_SPEED: f32 = 0.1;
const LOOK_SPEED: f32 = 0.1;
const R: f32 = 10.;

#[macroquad::main("MyGame")]
async fn main() {
    let mut b = donut(1.0, 0.2, 20, 10);
    let q = Quat::from_rotation_z(std::f32::consts::FRAC_PI_4)
        * Quat::from_rotation_x(std::f32::consts::FRAC_PI_4);
    rotate(&mut b, q);
    let world_up = vec3(0.0, 1.0, 0.0);
    let mut yaw: f32 = 1.18;
    let mut pitch: f32 = 0.0;

    let mut last_mouse_position: Vec2 = mouse_position().into();

    let mut grabbed = true;
    set_cursor_grab(grabbed);
    show_mouse(false);
    loop {
        let delta = get_frame_time();
        // println!("dt = {}", delta);
        if is_key_pressed(KeyCode::Escape) {
            break;
        }
        if is_key_pressed(KeyCode::Tab) {
            grabbed = !grabbed;
            set_cursor_grab(grabbed);
            show_mouse(!grabbed);
        }

        let mouse_position: Vec2 = mouse_position().into();
        let mouse_delta = mouse_position - last_mouse_position;

        last_mouse_position = mouse_position;

        let mut dx = 0.;
        let mut dy = 0.;
        let mut dz = 0.;

        if grabbed {
            yaw += mouse_delta.x * delta * -LOOK_SPEED;
            pitch += mouse_delta.y * delta * LOOK_SPEED;

            pitch = pitch.clamp(-PI / 2., PI / 2.);

            yaw = yaw.rem_euclid(2.0 * PI);

            dx = R * pitch.cos() * yaw.sin();
            dy = R * pitch.sin();
            dz = R * pitch.cos() * yaw.cos();
        }

        let position = vec3(dx, dy, dz);

        b.step(delta);
        clear_background(LIGHTGRAY);
        set_camera(&Camera3D {
            position,
            up: world_up,
            target: vec3(0., 0., 0.),
            ..Default::default()
        });

        draw_body(&b);

        set_default_camera();

        draw_fps();
        next_frame().await
    }
}
