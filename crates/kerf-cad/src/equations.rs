//! Equation resolver: topological evaluation of cross-parameter constraints.
//!
//! An "equation" maps a target parameter name to an arithmetic expression
//! string (the same syntax understood by [`crate::scalar`]).  At evaluation
//! time the resolver:
//!
//! 1. Builds a dependency graph by extracting all `$name` references from
//!    each expression.
//! 2. Topologically sorts the equations.
//! 3. Evaluates them in order, accumulating a merged parameter map that
//!    starts from the model's base `parameters` and overwrites entries as
//!    each equation is resolved.
//!
//! The result is a `HashMap<String, f64>` that features should use in place
//! of the raw `model.parameters` map during evaluation.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use crate::model::ModelError;
use crate::scalar::Scalar;

// ---------------------------------------------------------------------------
// Dependency extraction
// ---------------------------------------------------------------------------

/// Extract the set of `$name` references from an expression string.
/// Only returns the bare names (no leading `$`).
fn expr_deps(expr: &str) -> HashSet<String> {
    let bytes = expr.as_bytes();
    let mut deps = HashSet::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'$' {
            let start = i + 1;
            let mut j = start;
            while j < bytes.len() && (bytes[j].is_ascii_alphanumeric() || bytes[j] == b'_') {
                j += 1;
            }
            if j > start {
                if let Ok(name) = std::str::from_utf8(&bytes[start..j]) {
                    deps.insert(name.to_string());
                }
            }
            i = j;
        } else {
            i += 1;
        }
    }
    deps
}

// ---------------------------------------------------------------------------
// Topological sort (Kahn's algorithm)
// ---------------------------------------------------------------------------

/// Topologically sort equation names so each equation is evaluated after all
/// equations it references. Returns `Err(ModelError::Cycle(name))` if a cycle
/// is detected.
///
/// `equations` — the full equations map.
/// `base_params` — the model's base parameters (names in this set are leaves;
///   they don't generate a node in the dependency graph but are valid targets
///   for `$ref` inside expressions).
fn toposort(
    equations: &BTreeMap<String, String>,
    base_params: &HashMap<String, f64>,
) -> Result<Vec<String>, ModelError> {
    // Build adjacency using owned Strings to avoid lifetime headaches.
    // in_degree[name] = number of equation-to-equation deps that must resolve first.
    // dependents[dep_name] = list of equation names that depend on dep_name.
    let mut in_degree: HashMap<String, usize> = HashMap::new();
    let mut dependents: HashMap<String, Vec<String>> = HashMap::new();

    for name in equations.keys() {
        in_degree.entry(name.clone()).or_insert(0);
    }

    for (name, expr) in equations.iter() {
        let deps = expr_deps(expr);
        for dep in deps {
            if equations.contains_key(&dep) {
                // name depends on dep (an equation); dep must come first
                dependents
                    .entry(dep.clone())
                    .or_default()
                    .push(name.clone());
                *in_degree.entry(name.clone()).or_insert(0) += 1;
            } else if !base_params.contains_key(&dep) {
                // Neither an equation nor a base param → unknown
                return Err(ModelError::UnknownId(dep));
            }
        }
    }

    // Kahn's: start with all zero-in-degree nodes (sorted for determinism)
    let mut queue: VecDeque<String> = {
        let mut zeros: Vec<String> = in_degree
            .iter()
            .filter_map(|(k, &v)| if v == 0 { Some(k.clone()) } else { None })
            .collect();
        zeros.sort_unstable();
        VecDeque::from(zeros)
    };

    let mut order: Vec<String> = Vec::with_capacity(equations.len());

    while let Some(node) = queue.pop_front() {
        order.push(node.clone());
        if let Some(nexts) = dependents.get(&node) {
            let mut nexts_sorted = nexts.clone();
            nexts_sorted.sort_unstable();
            for next in nexts_sorted {
                let deg = in_degree.get_mut(&next).expect("node in graph");
                *deg -= 1;
                if *deg == 0 {
                    queue.push_back(next);
                }
            }
        }
    }

    if order.len() < equations.len() {
        // Find a cycle participant for the error message.
        // Any remaining node with in_degree > 0 is in a cycle.
        let cycle_node = in_degree
            .iter()
            .filter(|(_, v)| **v > 0)
            .map(|(k, _)| k.as_str())
            .min() // deterministic: pick lexicographically smallest
            .unwrap_or("unknown");
        return Err(ModelError::Cycle(cycle_node.to_string()));
    }

    Ok(order)
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Resolve all equations in `equations` against `base_params`.
///
/// Returns a merged parameter map: every entry from `base_params` is present,
/// and each equation result overwrites (or adds) its name. Features should
/// use this map instead of `model.parameters` during evaluation.
///
/// Errors:
/// - `ModelError::Cycle` — circular equation dependency
/// - `ModelError::UnknownId` — a `$name` reference that is neither an
///   equation name nor a base parameter name
/// - `ModelError::ExprError` — expression evaluation failed (parse or
///   arithmetic error)
pub fn resolve_equations(
    equations: &BTreeMap<String, String>,
    base_params: &HashMap<String, f64>,
) -> Result<HashMap<String, f64>, ModelError> {
    if equations.is_empty() {
        return Ok(base_params.clone());
    }

    // Validate all references first (catches unknown refs even in cycles).
    // The toposort also does this, but let's be explicit and surface unknown
    // refs before cycle errors when both apply.
    for (_name, expr) in equations.iter() {
        for dep in expr_deps(expr) {
            if !equations.contains_key(&dep) && !base_params.contains_key(&dep) {
                return Err(ModelError::UnknownId(dep));
            }
        }
    }

    let order = toposort(equations, base_params)?;

    // Start from a clone of the base params; overwrite/add equation results.
    let mut resolved: HashMap<String, f64> = base_params.clone();

    for name in &order {
        let expr = equations.get(name).expect("name comes from equations map");
        let value = Scalar::Expr(expr.clone())
            .resolve(&resolved)
            .map_err(|msg| ModelError::ExprError(name.clone(), msg))?;
        resolved.insert(name.clone(), value);
    }

    Ok(resolved)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn params(pairs: &[(&str, f64)]) -> HashMap<String, f64> {
        pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    fn eqs(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    // --- test 1: single equation ---
    #[test]
    fn single_equation_radius_from_diameter() {
        // radius = $diameter / 2, diameter=10 → radius=5
        let base = params(&[("diameter", 10.0)]);
        let eq = eqs(&[("radius", "$diameter / 2")]);
        let out = resolve_equations(&eq, &base).unwrap();
        assert!((out["radius"] - 5.0).abs() < 1e-12);
        assert!((out["diameter"] - 10.0).abs() < 1e-12);
    }

    // --- test 2: chained equations ---
    #[test]
    fn chained_equations() {
        // width = $depth * 2, height = $width + 1, depth=3 → height=7
        let base = params(&[("depth", 3.0)]);
        let eq = eqs(&[("width", "$depth * 2"), ("height", "$width + 1")]);
        let out = resolve_equations(&eq, &base).unwrap();
        assert!((out["width"] - 6.0).abs() < 1e-12);
        assert!((out["height"] - 7.0).abs() < 1e-12);
    }

    // --- test 3: cycle detection ---
    #[test]
    fn cycle_two_equations() {
        let base = params(&[]);
        let eq = eqs(&[("a", "$b"), ("b", "$a")]);
        match resolve_equations(&eq, &base) {
            Err(ModelError::Cycle(_)) => {}
            other => panic!("expected Cycle, got {other:?}"),
        }
    }

    // --- test 4: self-reference cycle ---
    #[test]
    fn self_reference_cycle() {
        let base = params(&[]);
        let eq = eqs(&[("a", "$a + 1")]);
        match resolve_equations(&eq, &base) {
            Err(ModelError::Cycle(_)) => {}
            other => panic!("expected Cycle, got {other:?}"),
        }
    }

    // --- test 5: unknown reference ---
    #[test]
    fn unknown_ref_errors() {
        let base = params(&[]);
        let eq = eqs(&[("x", "$nonexistent + 1")]);
        match resolve_equations(&eq, &base) {
            Err(ModelError::UnknownId(name)) => assert_eq!(name, "nonexistent"),
            other => panic!("expected UnknownId, got {other:?}"),
        }
    }

    // --- test 6 (model-level): JSON round-trip with equations ---
    // (in model.rs tests; referenced here for completeness — see equations_json_round_trip)

    // --- dep extraction unit test ---
    #[test]
    fn expr_deps_basic() {
        let d = expr_deps("$x + $y * 2");
        assert!(d.contains("x"));
        assert!(d.contains("y"));
        assert_eq!(d.len(), 2);
    }

    #[test]
    fn expr_deps_no_refs() {
        assert!(expr_deps("1 + 2 * 3").is_empty());
    }
}
