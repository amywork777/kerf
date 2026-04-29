//! Polynomial root finders used by intersection routines.

/// Real roots of `a t² + b t + c = 0` within numerical tolerance.
/// Returns roots sorted ascending. Handles the linear and degenerate cases.
pub fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
    if a.abs() < 1e-15 {
        if b.abs() < 1e-15 {
            return Vec::new();
        }
        return vec![-c / b];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return Vec::new();
    }
    if disc == 0.0 {
        return vec![-b / (2.0 * a)];
    }
    let sd = disc.sqrt();
    let q = -0.5 * (b + b.signum() * sd);
    let t1 = q / a;
    let t2 = c / q;
    let mut out = vec![t1, t2];
    out.sort_by(|x, y| x.partial_cmp(y).unwrap());
    out
}

/// Real roots of `a t³ + b t² + c t + d = 0`. Cardano's formula.
pub fn solve_cubic(a: f64, b: f64, c: f64, d: f64) -> Vec<f64> {
    if a.abs() < 1e-15 {
        return solve_quadratic(b, c, d);
    }
    let p = b / a;
    let q = c / a;
    let r = d / a;
    let p3 = p / 3.0;
    let alpha = q - p * p / 3.0;
    let beta  = 2.0 * p * p * p / 27.0 - p * q / 3.0 + r;
    let disc = -4.0 * alpha.powi(3) - 27.0 * beta * beta;
    let mut roots = Vec::new();
    if disc > 0.0 {
        let m = 2.0 * (-alpha / 3.0).sqrt();
        let theta = (3.0 * beta / (alpha * m)).acos() / 3.0;
        for k in 0..3 {
            let t = m * (theta - 2.0 * std::f64::consts::PI * k as f64 / 3.0).cos() - p3;
            roots.push(t);
        }
    } else {
        let half_b = beta / 2.0;
        let disc_in = half_b * half_b + alpha.powi(3) / 27.0;
        let s_disc = disc_in.sqrt();
        let u = cbrt(-half_b + s_disc);
        let v = cbrt(-half_b - s_disc);
        roots.push(u + v - p3);
    }
    roots.sort_by(|x, y| x.partial_cmp(y).unwrap());
    roots
}

/// Real roots of `a t⁴ + b t³ + c t² + d t + e = 0`. Ferrari's method.
pub fn solve_quartic(a: f64, b: f64, c: f64, d: f64, e: f64) -> Vec<f64> {
    if a.abs() < 1e-15 {
        return solve_cubic(b, c, d, e);
    }
    let p = b / a;
    let q = c / a;
    let r = d / a;
    let s = e / a;
    let p4 = p / 4.0;
    let alpha = q - 3.0 * p * p / 8.0;
    let beta  = r - p * q / 2.0 + p * p * p / 8.0;
    let gamma = s - p * r / 4.0 + p * p * q / 16.0 - 3.0 * p.powi(4) / 256.0;

    if beta.abs() < 1e-12 {
        let mut out = Vec::new();
        for &u2 in &solve_quadratic(1.0, alpha, gamma) {
            if u2 < 0.0 { continue; }
            let u = u2.sqrt();
            out.push(u - p4);
            out.push(-u - p4);
        }
        out.sort_by(|x, y| x.partial_cmp(y).unwrap());
        return out;
    }

    let cubic_roots = solve_cubic(8.0, 8.0 * alpha, 2.0 * alpha * alpha - 8.0 * gamma, -beta * beta);
    let y = cubic_roots.into_iter().fold(f64::NEG_INFINITY, f64::max);
    let two_y = 2.0 * y;
    if two_y < 0.0 { return Vec::new(); }
    let sqrt_2y = two_y.sqrt();
    if sqrt_2y.abs() < 1e-15 { return Vec::new(); }

    let m1 = alpha + 2.0 * y - beta / sqrt_2y;
    let m2 = alpha + 2.0 * y + beta / sqrt_2y;
    let mut out: Vec<f64> = solve_quadratic(1.0, sqrt_2y, m1 / 2.0)
        .into_iter()
        .chain(solve_quadratic(1.0, -sqrt_2y, m2 / 2.0))
        .map(|u| u - p4)
        .collect();
    out.sort_by(|x, y| x.partial_cmp(y).unwrap());
    out
}

fn cbrt(x: f64) -> f64 {
    x.signum() * x.abs().powf(1.0 / 3.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn quadratic_two_roots() {
        let r = solve_quadratic(1.0, -3.0, 2.0);
        assert_eq!(r.len(), 2);
        assert_relative_eq!(r[0], 1.0, epsilon = 1e-12);
        assert_relative_eq!(r[1], 2.0, epsilon = 1e-12);
    }

    #[test]
    fn quadratic_no_real_roots() {
        assert!(solve_quadratic(1.0, 0.0, 1.0).is_empty());
    }

    #[test]
    fn quadratic_linear_fallback() {
        let r = solve_quadratic(0.0, 2.0, -4.0);
        assert_eq!(r.len(), 1);
        assert_relative_eq!(r[0], 2.0, epsilon = 1e-12);
    }

    #[test]
    fn cubic_three_roots() {
        let r = solve_cubic(1.0, -6.0, 11.0, -6.0);
        assert_eq!(r.len(), 3);
        for (got, want) in r.iter().zip([1.0, 2.0, 3.0].iter()) {
            assert_relative_eq!(*got, *want, epsilon = 1e-9);
        }
    }

    #[test]
    fn quartic_four_roots() {
        let r = solve_quartic(1.0, 0.0, -5.0, 0.0, 4.0);
        assert_eq!(r.len(), 4);
        for (got, want) in r.iter().zip([-2.0, -1.0, 1.0, 2.0].iter()) {
            assert_relative_eq!(*got, *want, epsilon = 1e-9);
        }
    }
}
