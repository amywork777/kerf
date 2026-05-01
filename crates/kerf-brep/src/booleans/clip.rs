//! Clip a 3D line to a planar convex polygon, returning the clipped parameter range.

use kerf_geom::{Frame, Line, Point3, Tolerance};

#[derive(Clone, Debug, PartialEq)]
pub enum ClipResult {
    Empty,
    Range(f64, f64),
}

pub fn clip_line_to_convex_polygon(
    line: &Line,
    poly: &[Point3],
    frame: &Frame,
    tol: &Tolerance,
) -> ClipResult {
    if poly.len() < 3 {
        return ClipResult::Empty;
    }

    let poly2d: Vec<(f64, f64)> = poly
        .iter()
        .map(|p| {
            let (u, v, _) = frame.local_of(*p);
            (u, v)
        })
        .collect();

    let (ox, oy, _) = frame.local_of(line.origin);
    let dx = line.direction.dot(&frame.x);
    let dy = line.direction.dot(&frame.y);

    let mut ts: Vec<f64> = Vec::new();
    let n = poly2d.len();
    for i in 0..n {
        let (ax, ay) = poly2d[i];
        let (bx, by) = poly2d[(i + 1) % n];
        let ex = bx - ax;
        let ey = by - ay;
        let det = dx * (-ey) - (-ex) * dy;
        if det.abs() < tol.angle_eq {
            continue;
        }
        let rhs1 = ax - ox;
        let rhs2 = ay - oy;
        let t = (rhs1 * (-ey) - (-ex) * rhs2) / det;
        let s = (dx * rhs2 - dy * rhs1) / det;
        if s >= -tol.point_eq && s <= 1.0 + tol.point_eq {
            ts.push(t);
        }
    }

    if ts.is_empty() {
        return ClipResult::Empty;
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < tol.point_eq);

    if ts.len() == 1 {
        return ClipResult::Range(ts[0], ts[0]);
    }

    let t_min = *ts.first().unwrap();
    let t_max = *ts.last().unwrap();
    if (t_max - t_min).abs() < tol.point_eq {
        ClipResult::Range(t_min, t_min)
    } else {
        ClipResult::Range(t_min, t_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::Vec3;

    fn unit_square_xy() -> (Vec<Point3>, Frame) {
        let poly = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        (poly, Frame::world(Point3::origin()))
    }

    #[test]
    fn line_through_diameter_clips_to_unit_range() {
        let (poly, frame) = unit_square_xy();
        let line = Line::from_origin_dir(Point3::new(-1.0, 0.5, 0.0), Vec3::x()).unwrap();
        let result = clip_line_to_convex_polygon(&line, &poly, &frame, &Tolerance::default());
        match result {
            ClipResult::Range(t_min, t_max) => {
                assert!((t_min - 1.0).abs() < 1e-9, "t_min = {t_min}");
                assert!((t_max - 2.0).abs() < 1e-9, "t_max = {t_max}");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn line_outside_polygon_is_empty() {
        let (poly, frame) = unit_square_xy();
        let line = Line::from_origin_dir(Point3::new(-1.0, 5.0, 0.0), Vec3::x()).unwrap();
        let result = clip_line_to_convex_polygon(&line, &poly, &frame, &Tolerance::default());
        assert_eq!(result, ClipResult::Empty);
    }

    #[test]
    fn line_through_corner_yields_short_range() {
        let (poly, frame) = unit_square_xy();
        let line =
            Line::from_origin_dir(Point3::new(-1.0, -1.0, 0.0), Vec3::new(1.0, 1.0, 0.0)).unwrap();
        let result = clip_line_to_convex_polygon(&line, &poly, &frame, &Tolerance::default());
        match result {
            ClipResult::Range(t_min, t_max) => {
                let s2 = std::f64::consts::SQRT_2;
                assert!((t_min - s2).abs() < 1e-9, "t_min = {t_min}, expected {s2}");
                assert!(
                    (t_max - 2.0 * s2).abs() < 1e-9,
                    "t_max = {t_max}, expected {}",
                    2.0 * s2
                );
            }
            other => panic!("{other:?}"),
        }
    }
}
