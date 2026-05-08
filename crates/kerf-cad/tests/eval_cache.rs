//! Tests for the per-Model evaluation cache (M-perf).
//!
//! These exercise the cache from the public `evaluate_cached` entry point
//! rather than poking the cache directly — that's what the WASM crate (and
//! any other production caller) actually uses.

use kerf_brep::solid_volume;
use kerf_cad::{EvalCache, Feature, Model, Scalar};

/// Helper: build a model with one Box that depends on parameter `w`.
fn box_model_param(w: f64) -> Model {
    Model::new()
        .with_parameter("w", w)
        .with_parameter("h", 1.0)
        .with_parameter("d", 1.0)
        .add(Feature::Box {
            id: "body".into(),
            extents: [
                Scalar::param("w"),
                Scalar::param("h"),
                Scalar::param("d"),
            ],
        })
}

/// Helper: build a model with two boxes and a Difference between them. The
/// "boolean" subtraction is the expensive node we most want to cache hit.
fn diff_model(big_w: f64, hole_r: f64) -> Model {
    Model::new()
        .with_parameter("big_w", big_w)
        .with_parameter("hole_r", hole_r)
        .add(Feature::Box {
            id: "body".into(),
            extents: [Scalar::param("big_w"), Scalar::lit(2.0), Scalar::lit(2.0)],
        })
        .add(Feature::Cylinder {
            id: "hole".into(),
            radius: Scalar::param("hole_r"),
            height: Scalar::lit(10.0),
            segments: 16,
        })
        .add(Feature::Difference {
            id: "result".into(),
            inputs: vec!["body".into(), "hole".into()],
        })
}

#[test]
fn eval_cache_hits_when_no_change() {
    let m = box_model_param(2.0);
    let mut cache = EvalCache::new();

    // Cold: cache empty, compute everything.
    let s1 = m.evaluate_cached("body", &mut cache).expect("eval 1");
    assert_eq!(cache.len(), 1, "one entry after cold eval");
    let v1 = solid_volume(&s1);

    // Warm: same recipe, expect a cache hit (cache size unchanged) and an
    // identical solid (same volume, same shell count).
    let entries_before = cache.len();
    let s2 = m.evaluate_cached("body", &mut cache).expect("eval 2");
    let entries_after = cache.len();
    let v2 = solid_volume(&s2);

    assert_eq!(entries_before, entries_after, "no new entries on hit");
    assert!((v1 - v2).abs() < 1e-12, "same volume on hit");
    assert_eq!(s1.shell_count(), s2.shell_count());
    assert_eq!(s1.face_count(), s2.face_count());
}

#[test]
fn eval_cache_invalidates_on_param_change() {
    // Same model graph, different parameter value. The fingerprint changes
    // for `body` because it references `w`, so the cached solid must NOT
    // be returned.
    let m1 = box_model_param(2.0);
    let m2 = box_model_param(3.0);
    let mut cache = EvalCache::new();

    let s1 = m1.evaluate_cached("body", &mut cache).expect("eval 1");
    let s2 = m2.evaluate_cached("body", &mut cache).expect("eval 2");

    let v1 = solid_volume(&s1);
    let v2 = solid_volume(&s2);

    // body extents = (w, 1, 1), so volumes are 2 and 3. If the cache
    // returned the stale solid, both would be 2.
    assert!((v1 - 2.0).abs() < 1e-9, "v1 = {v1}");
    assert!((v2 - 3.0).abs() < 1e-9, "v2 = {v2}");
}

#[test]
fn eval_cache_recomputes_on_input_change() {
    // A boolean's input changes (because an upstream parameter changed).
    // The boolean's own JSON is identical between the two models, but its
    // input fingerprint differs, so the boolean's fingerprint differs and
    // the cache must NOT hit.
    let m1 = diff_model(4.0, 0.5);
    let m2 = diff_model(4.0, 0.7); // bigger hole → smaller result volume

    let mut cache = EvalCache::new();
    let r1 = m1.evaluate_cached("result", &mut cache).expect("eval 1");
    let r2 = m2.evaluate_cached("result", &mut cache).expect("eval 2");

    let v1 = solid_volume(&r1);
    let v2 = solid_volume(&r2);

    // Bigger hole removes more material → r2 has strictly smaller volume.
    // If the cache wrongly hit, v1 == v2.
    assert!(v2 < v1, "expected v2 < v1 (v1={v1}, v2={v2})");
}

#[test]
fn eval_cache_only_one_entry_per_feature() {
    // When the cache is reused across two evaluations, it should grow the
    // entry-set monotonically (re-using slots for the same feature id, not
    // doubling up). This catches a class of bugs where the cache key
    // accidentally includes per-call state and fills up over time.
    let m1 = diff_model(4.0, 0.5);
    let m2 = diff_model(4.0, 0.7);
    let mut cache = EvalCache::new();

    let _ = m1.evaluate_cached("result", &mut cache).expect("warm");
    assert_eq!(cache.len(), 3, "body + hole + result cached after first eval");

    // Note: the fingerprint algorithm hashes the WHOLE parameter map for
    // simplicity (we don't statically know which params each feature
    // references because expressions can be arbitrary). So changing
    // `hole_r` does invalidate body's fingerprint — body recomputes once.
    // After that, both old and new fingerprints belong to body, but the
    // cache only stores ONE entry per id (the latest). We verify that:
    let _ = m2.evaluate_cached("result", &mut cache).expect("warm 2");
    assert_eq!(
        cache.len(),
        3,
        "still 3 entries (body, hole, result) — same ids overwrite",
    );
}

#[test]
fn eval_cache_clear_drops_everything() {
    let m = box_model_param(1.5);
    let mut cache = EvalCache::new();
    m.evaluate_cached("body", &mut cache).expect("eval");
    assert_eq!(cache.len(), 1);

    cache.clear();
    assert!(cache.is_empty());

    // Subsequent eval still works; cache repopulates.
    m.evaluate_cached("body", &mut cache).expect("eval after clear");
    assert_eq!(cache.len(), 1);
}

#[test]
fn cached_eval_matches_uncached_eval() {
    // Sanity: evaluate_cached and evaluate must produce equivalent solids.
    // This catches bugs where the cache path skipped face_owner_tag
    // assignment or otherwise diverged.
    let m = diff_model(4.0, 0.6);

    let plain = m.evaluate("result").expect("plain eval");
    let mut cache = EvalCache::new();
    let cached = m.evaluate_cached("result", &mut cache).expect("cached eval");

    assert_eq!(plain.vertex_count(), cached.vertex_count());
    assert_eq!(plain.edge_count(), cached.edge_count());
    assert_eq!(plain.face_count(), cached.face_count());
    let v_plain = solid_volume(&plain);
    let v_cached = solid_volume(&cached);
    assert!(
        (v_plain - v_cached).abs() < 1e-9,
        "volumes differ: plain={v_plain} cached={v_cached}"
    );
}

#[test]
fn eval_cache_face_owner_tags_propagate_through_hits() {
    // The cache stores the tagged solid. On a hit, those tags must be
    // intact on the returned clone — picking would break otherwise.
    let m = box_model_param(2.0);
    let mut cache = EvalCache::new();

    let cold = m.evaluate_cached("body", &mut cache).expect("cold");
    let hot = m.evaluate_cached("body", &mut cache).expect("hot");

    let cold_tags: Vec<String> = cold
        .topo
        .face_ids()
        .map(|fid| cold.face_owner_tag.get(fid).cloned().unwrap_or_default())
        .collect();
    let hot_tags: Vec<String> = hot
        .topo
        .face_ids()
        .map(|fid| hot.face_owner_tag.get(fid).cloned().unwrap_or_default())
        .collect();

    assert_eq!(cold_tags, hot_tags, "owner tags identical across hits");
    assert!(
        cold_tags.iter().all(|t| t == "body"),
        "all faces tagged with feature id"
    );
}
