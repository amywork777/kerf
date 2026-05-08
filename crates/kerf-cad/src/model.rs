//! Model: a DAG of features keyed by id.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::feature::Feature;

#[derive(Debug, Error)]
pub enum ModelError {
    #[error("duplicate feature id: {0}")]
    DuplicateId(String),
    #[error("unknown feature id: {0}")]
    UnknownId(String),
    #[error("cycle detected involving id: {0}")]
    Cycle(String),
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
}

/// A declarative CAD model. Insertion order is preserved (so JSON round-trips
/// keep their authored order) but lookup is by id.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Model {
    /// Named numeric parameters referenced by `Scalar::Param("$name")` in
    /// feature fields. Empty by default.
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub parameters: HashMap<String, f64>,
    /// Features keyed by id, kept in insertion order so the JSON/file form
    /// matches what the user wrote.
    pub(crate) features: Vec<Feature>,
    /// Cached id -> index lookup. Recomputed after deserialize.
    #[serde(skip)]
    index: HashMap<String, usize>,
}

impl Model {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a feature. Panics if the id is already present (use `try_add` for
    /// the non-panicking variant). Returns self for fluent chaining.
    pub fn add(mut self, f: Feature) -> Self {
        self = self.try_add(f).expect("duplicate feature id");
        self
    }

    pub fn with_parameter(mut self, name: impl Into<String>, value: f64) -> Self {
        self.parameters.insert(name.into(), value);
        self
    }

    pub fn try_add(mut self, f: Feature) -> Result<Self, ModelError> {
        let id = f.id().to_string();
        if self.index.contains_key(&id) {
            return Err(ModelError::DuplicateId(id));
        }
        self.index.insert(id, self.features.len());
        self.features.push(f);
        Ok(self)
    }

    pub fn feature(&self, id: &str) -> Option<&Feature> {
        self.index.get(id).map(|i| &self.features[*i])
    }

    pub fn ids(&self) -> impl Iterator<Item = &str> {
        self.features.iter().map(|f| f.id())
    }

    pub fn len(&self) -> usize {
        self.features.len()
    }

    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    /// Rebuild the id -> index map. Called after deserialize and any time the
    /// caller mutates `features` directly.
    pub fn reindex(&mut self) -> Result<(), ModelError> {
        self.index.clear();
        for (i, f) in self.features.iter().enumerate() {
            let id = f.id().to_string();
            if self.index.insert(id.clone(), i).is_some() {
                return Err(ModelError::DuplicateId(id));
            }
        }
        Ok(())
    }

    pub fn to_json_string(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }

    pub fn from_json_str(s: &str) -> Result<Self, ModelError> {
        let mut m: Model = serde_json::from_str(s).map_err(ModelError::Json)?;
        m.reindex()?;
        Ok(m)
    }

    pub fn write_json(&self, w: &mut impl Write) -> Result<(), ModelError> {
        serde_json::to_writer_pretty(w, self).map_err(ModelError::Json)
    }

    pub fn read_json(r: &mut impl Read) -> Result<Self, ModelError> {
        let mut m: Model = serde_json::from_reader(r).map_err(ModelError::Json)?;
        m.reindex()?;
        Ok(m)
    }

    pub fn write_json_path(&self, path: impl AsRef<Path>) -> Result<(), ModelError> {
        let f = File::create(path.as_ref()).map_err(ModelError::Io)?;
        let mut w = BufWriter::new(f);
        self.write_json(&mut w)
    }

    pub fn read_json_path(path: impl AsRef<Path>) -> Result<Self, ModelError> {
        let f = File::open(path.as_ref()).map_err(ModelError::Io)?;
        let mut r = BufReader::new(f);
        Self::read_json(&mut r)
    }
}
