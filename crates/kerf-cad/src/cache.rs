//! Per-Model evaluation cache.
//!
//! Recomputing a feature graph from JSON is expensive — every parameter
//! tweak in the viewer would otherwise rebuild every feature in the DAG,
//! including booleans that take tens of milliseconds each. This cache
//! skips re-evaluation when a feature's inputs and parameters are unchanged
//! from a previous call.
//!
//! ## Caching strategy: recipe fingerprints
//!
//! Solids don't implement `Hash` (their `SecondaryMap` storage isn't
//! hash-stable), so we never hash a Solid's topology. Instead we hash the
//! **recipe** that produced it — `(feature_id, feature_serde_repr,
//! resolved_params, input_fingerprints)`. Two evaluations with the same
//! recipe must produce the same Solid (the evaluator is deterministic), so
//! a recipe match is a safe cache hit.
//!
//! The fingerprint is a `u64` from `DefaultHasher`. Collisions are
//! astronomically unlikely for the scale of recipes a single viewer
//! session sees, but they would be silently wrong if they occurred. If
//! that ever becomes a concern, the fingerprint can be widened to a
//! `[u64; 2]` SipHash without changing the cache API.
//!
//! ## Invalidation
//!
//! Parameter changes propagate naturally: when `params` differ, the
//! fingerprint of every feature that references those params changes,
//! and so does the fingerprint of every feature downstream of those.
//! Features unrelated to the changed params keep the same fingerprint
//! and stay cached.
//!
//! ## What's cached
//!
//! - Successful evaluations only. Errors are not cached — re-running
//!   yields a fresh error object each time, which is fine because errors
//!   are rare and not on the hot path.
//! - Whole solids, not partial work. A boolean's fold accumulator isn't
//!   cached on its own — only the boolean's final result.

use std::collections::HashMap;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use kerf_brep::Solid;

use crate::feature::Feature;

/// 64-bit recipe fingerprint. Two evaluations with the same fingerprint
/// produce identical Solids (modulo collisions, which we accept).
pub type Fingerprint = u64;

/// Internal entry: the cached Solid plus the fingerprint that identifies
/// the recipe. Storing the fingerprint alongside lets a feature downstream
/// look up "what recipe was the input of `acc` produced from?" without
/// re-walking the DAG.
#[derive(Clone, Debug)]
struct Entry {
    fingerprint: Fingerprint,
    solid: Solid,
}

/// Per-Model evaluation cache. One cache instance corresponds to one
/// long-lived Model — typically the model loaded in the browser viewer.
///
/// The cache is by **feature id**, not by fingerprint, because the public
/// API exposes feature ids (`evaluate("body")`) and that's how the viewer
/// re-requests results when a parameter slider moves. Internally each id
/// also carries its last-seen fingerprint, which is the actual cache-hit
/// test: a fingerprint match means "same recipe → same solid", a miss
/// means "recipe changed (parameter, input, or feature itself) → recompute".
#[derive(Clone, Debug, Default)]
pub struct EvalCache {
    by_id: HashMap<String, Entry>,
}

impl EvalCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// Number of cached features. Useful for tests and debug overlays.
    pub fn len(&self) -> usize {
        self.by_id.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_id.is_empty()
    }

    /// Drop every cached solid. Call after editing the model graph
    /// (adding/removing features) when you don't want the staleness
    /// risk of trying to invalidate piecemeal.
    pub fn clear(&mut self) {
        self.by_id.clear();
    }

    /// Drop the cached solid for one feature. Used when callers know a
    /// specific feature has been edited.
    pub fn invalidate(&mut self, id: &str) {
        self.by_id.remove(id);
    }

    /// Look up by feature id and current fingerprint. Returns Some only
    /// if both the id is present AND its stored fingerprint matches the
    /// caller's. A miss can mean "never computed" or "recipe changed";
    /// callers don't need to distinguish.
    pub fn get(&self, id: &str, fingerprint: Fingerprint) -> Option<&Solid> {
        let entry = self.by_id.get(id)?;
        if entry.fingerprint == fingerprint {
            Some(&entry.solid)
        } else {
            None
        }
    }

    /// Get the fingerprint stored for a feature id, regardless of whether
    /// it matches anything. Used during DAG walks to construct downstream
    /// fingerprints from upstream ones.
    pub fn fingerprint_of(&self, id: &str) -> Option<Fingerprint> {
        self.by_id.get(id).map(|e| e.fingerprint)
    }

    /// Insert or update an entry.
    pub fn put(&mut self, id: String, fingerprint: Fingerprint, solid: Solid) {
        self.by_id.insert(id, Entry { fingerprint, solid });
    }
}

/// Compute the fingerprint of a feature's recipe.
///
/// `feature` is the feature itself (its serde repr captures every field
/// that affects the result). `params` is the model's full parameter map
/// — we hash all of it rather than try to figure out which keys this
/// feature references, because a) figuring that out requires walking
/// expression strings, and b) hashing a small map is cheap. This means
/// changing an unrelated parameter still invalidates this feature's
/// fingerprint, which is over-conservative but safe.
///
/// `input_fingerprints` are the fingerprints of upstream features in the
/// order `feature.inputs()` returns them. The order matters: it's how
/// boolean folding distinguishes `Difference [a, b]` from `Difference [b, a]`.
pub fn feature_fingerprint(
    feature: &Feature,
    params: &HashMap<String, f64>,
    input_fingerprints: &[Fingerprint],
) -> Fingerprint {
    let mut hasher = DefaultHasher::new();
    // Hash the feature's serde JSON representation. JSON is deterministic
    // for our struct shapes (enum tag + named fields, never HashMap fields
    // inside Feature itself), so the same feature always produces the same
    // bytes. We could hash the in-memory Feature directly if it implemented
    // Hash, but it doesn't (Scalar::Lit holds an f64 which has no Hash impl
    // because of NaN), so JSON is the simplest stable serialization.
    let json = serde_json::to_string(feature).unwrap_or_default();
    json.hash(&mut hasher);

    // Hash params in a deterministic order. HashMap iteration order
    // varies across runs, so collect-and-sort by key.
    let mut keys: Vec<&String> = params.keys().collect();
    keys.sort();
    for k in keys {
        k.hash(&mut hasher);
        // f64 has no Hash; hash its bit pattern. -0.0 and +0.0 hash
        // differently, but they also produce identical evaluations,
        // so a spurious miss here costs us a recompute, never a wrong
        // result. NaN in params is undefined behavior already.
        params[k].to_bits().hash(&mut hasher);
    }

    for fp in input_fingerprints {
        fp.hash(&mut hasher);
    }
    hasher.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scalar::Scalar;

    fn box_feature(id: &str, x: f64) -> Feature {
        Feature::Box {
            id: id.into(),
            extents: [Scalar::lit(x), Scalar::lit(1.0), Scalar::lit(1.0)],
        }
    }

    #[test]
    fn same_recipe_same_fingerprint() {
        let f = box_feature("a", 2.0);
        let p = HashMap::new();
        assert_eq!(
            feature_fingerprint(&f, &p, &[]),
            feature_fingerprint(&f, &p, &[]),
        );
    }

    #[test]
    fn different_extents_different_fingerprint() {
        let p = HashMap::new();
        let a = feature_fingerprint(&box_feature("x", 2.0), &p, &[]);
        let b = feature_fingerprint(&box_feature("x", 3.0), &p, &[]);
        assert_ne!(a, b);
    }

    #[test]
    fn different_param_value_different_fingerprint() {
        let f = Feature::Box {
            id: "b".into(),
            extents: [Scalar::param("w"), Scalar::lit(1.0), Scalar::lit(1.0)],
        };
        let mut p1 = HashMap::new();
        p1.insert("w".to_string(), 1.0);
        let mut p2 = HashMap::new();
        p2.insert("w".to_string(), 2.0);
        assert_ne!(
            feature_fingerprint(&f, &p1, &[]),
            feature_fingerprint(&f, &p2, &[]),
        );
    }

    #[test]
    fn different_inputs_different_fingerprint() {
        let f = Feature::Translate {
            id: "t".into(),
            input: "src".into(),
            offset: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
        };
        let p = HashMap::new();
        assert_ne!(
            feature_fingerprint(&f, &p, &[1]),
            feature_fingerprint(&f, &p, &[2]),
        );
    }

    #[test]
    fn cache_get_miss_for_different_fingerprint() {
        let mut c = EvalCache::new();
        c.put("a".into(), 42, Solid::new());
        assert!(c.get("a", 42).is_some());
        assert!(c.get("a", 43).is_none());
        assert!(c.get("missing", 42).is_none());
    }

    #[test]
    fn cache_invalidate_drops_entry() {
        let mut c = EvalCache::new();
        c.put("a".into(), 1, Solid::new());
        c.put("b".into(), 2, Solid::new());
        c.invalidate("a");
        assert!(c.get("a", 1).is_none());
        assert!(c.get("b", 2).is_some());
    }

    #[test]
    fn cache_clear_drops_everything() {
        let mut c = EvalCache::new();
        c.put("a".into(), 1, Solid::new());
        c.put("b".into(), 2, Solid::new());
        c.clear();
        assert_eq!(c.len(), 0);
    }
}
