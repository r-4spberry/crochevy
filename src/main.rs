use std::f32::consts::PI;

use macroquad::prelude::*;



struct Vert {
    m: f32,
    pos: Vec3,
    v: Vec3,
}

#[derive(PartialEq)]
enum CstrKind {
    Distance(f32),
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

impl Cstr {
    fn c(&self, verts: Vec<Vert>) -> f32 {
        match self.kind {
            CstrKind::Distance(d) => self._c_distance(d, verts),
            CstrKind::Volume => unimplemented!(),
        }
    }
    fn _c_distance(&self, d: f32, verts: Vec<Vert>) -> f32 {
        assert!(self.inds.len() == 2);
        let a = verts[self.inds[0]].pos;
        let b = verts[self.inds[1]].pos;
        let dist = (a - b).length();
        dist - d
    }
}

struct Body {
    verts: Vec<Vert>,
    cstrs: Vec<Cstr>,
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

    let cstrs: Vec<Cstr> = edges
        .iter()
        .map(|&(i, j)| Cstr {
            inds: vec![i, j],
            k: 0.8,
            kind: CstrKind::Distance((pos[i] - pos[j]).length()),
            eq_type: CstrEqType::Equality,
        })
        .collect();

    let verts: Vec<Vert> = pos
        .into_iter()
        .map(|pos| Vert {
            m: 1.0,
            pos,
            v: Vec3::ZERO,
        })
        .collect();

    Body { verts, cstrs }
}

impl Body {
    fn step(&mut self) {}
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

    let cstrs: Vec<Cstr> = edges
        .iter()
        .map(|&(a, b)| Cstr {
            inds: vec![a, b],
            k: 0.8,
            kind: CstrKind::Distance((pos[a] - pos[b]).length()),
            eq_type: CstrEqType::Equality,
        })
        .collect();

    let verts: Vec<Vert> = pos
        .into_iter()
        .map(|pos| Vert {
            m: 1.0,
            pos,
            v: Vec3::ZERO,
        })
        .collect();

    Body { verts, cstrs }
}

fn draw_body(b: &Body) {
    for vert in &b.verts {
        draw_sphere(vert.pos, 0.1, None, RED);
    }
    for cstr in &b.cstrs {
        if cstr.inds.len() == 2 && matches!(cstr.kind, CstrKind::Distance(_)) {
            let v1 = b.verts[cstr.inds[0]].pos;
            let v2 = b.verts[cstr.inds[1]].pos;

            draw_line_3d(v1, v2, BLUE);
        }
    }
}

const MOVE_SPEED: f32 = 0.1;
const LOOK_SPEED: f32 = 0.1;
const R: f32 = 10.;

#[macroquad::main("MyGame")]
async fn main() {
    let b = pyramid(1., 2.);

    let world_up = vec3(0.0, 1.0, 0.0);
    let mut yaw: f32 = 1.18;
    let mut pitch: f32 = 0.0;

    let mut position = vec3(0.0, 1.0, 0.0);
    let mut last_mouse_position: Vec2 = mouse_position().into();

    let mut grabbed = true;
    set_cursor_grab(grabbed);
    show_mouse(false);
    loop {
        let delta = get_frame_time();

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

            yaw = yaw.rem_euclid(PI);

            dx = R * pitch.cos() * yaw.sin();
            dy = R * pitch.sin();
            dz = R * pitch.cos() * yaw.cos();
        }

        position = vec3(dx, dy, dz);
        clear_background(LIGHTGRAY);
        set_camera(&Camera3D {
            position: position,
            up: world_up,
            target: vec3(0., 0., 0.),
            ..Default::default()
        });

        draw_body(&b);
        next_frame().await
    }
}
