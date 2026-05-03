//! Scalar values: literal numbers, parameter lookups, or expressions.
//!
//! In JSON, this serializes as either a number (`1.5`) or a string. A string
//! starting with `$` followed by an identifier (and nothing else) is treated
//! as a bare parameter lookup; any other string is parsed as an expression.
//! Expressions support `+ - * /`, parentheses, `$name` parameter references,
//! decimal numbers, and a handful of math builtins (`sin`, `cos`, `tan`,
//! `sqrt`, `abs`, `min`, `max`).

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Scalar {
    /// A concrete numeric value.
    Lit(f64),
    /// A string — parsed at evaluation time as either a single `$name`
    /// reference or an arithmetic expression.
    Expr(String),
}

impl Scalar {
    pub fn lit(x: f64) -> Self {
        Scalar::Lit(x)
    }

    /// Construct a bare parameter reference (`$name`).
    pub fn param(name: impl Into<String>) -> Self {
        let mut s = name.into();
        if !s.starts_with('$') {
            s.insert(0, '$');
        }
        Scalar::Expr(s)
    }

    /// Construct an arbitrary expression. The string is parsed at evaluation
    /// time, so syntax errors surface only when the model is evaluated.
    pub fn expr(s: impl Into<String>) -> Self {
        Scalar::Expr(s.into())
    }

    /// Resolve this scalar to an f64 against `params`. Literals pass through;
    /// expressions are tokenized + parsed + evaluated.
    pub fn resolve(&self, params: &HashMap<String, f64>) -> Result<f64, String> {
        match self {
            Scalar::Lit(x) => Ok(*x),
            Scalar::Expr(s) => evaluate_expression(s, params),
        }
    }
}

impl From<f64> for Scalar {
    fn from(x: f64) -> Self {
        Scalar::Lit(x)
    }
}

/// Convert a fixed-size array of f64s into the same-size array of `Scalar::Lit`.
pub fn lits<const N: usize>(arr: [f64; N]) -> [Scalar; N] {
    arr.map(Scalar::Lit)
}

pub(crate) fn resolve_arr<const N: usize>(
    arr: &[Scalar; N],
    params: &HashMap<String, f64>,
) -> Result<[f64; N], String> {
    let mut out = [0.0; N];
    for (i, s) in arr.iter().enumerate() {
        out[i] = s.resolve(params)?;
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Expression evaluation: tokenizer + recursive-descent parser + evaluator.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Num(f64),
    Var(String),  // bare name (no leading $)
    Func(String),
    LParen,
    RParen,
    Comma,
    Plus,
    Minus,
    Star,
    Slash,
    End,
}

fn tokenize(src: &str) -> Result<Vec<Tok>, String> {
    let mut tokens = Vec::new();
    let bytes = src.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        match c {
            b' ' | b'\t' | b'\n' => {
                i += 1;
            }
            b'(' => { tokens.push(Tok::LParen); i += 1; }
            b')' => { tokens.push(Tok::RParen); i += 1; }
            b',' => { tokens.push(Tok::Comma); i += 1; }
            b'+' => { tokens.push(Tok::Plus); i += 1; }
            b'-' => { tokens.push(Tok::Minus); i += 1; }
            b'*' => { tokens.push(Tok::Star); i += 1; }
            b'/' => { tokens.push(Tok::Slash); i += 1; }
            b'$' => {
                let start = i + 1;
                let mut j = start;
                while j < bytes.len()
                    && (bytes[j].is_ascii_alphanumeric() || bytes[j] == b'_')
                {
                    j += 1;
                }
                if j == start {
                    return Err(format!("expected name after '$' at byte {i}"));
                }
                let name = std::str::from_utf8(&bytes[start..j])
                    .map_err(|e| format!("bad utf8 in name: {e}"))?
                    .to_string();
                tokens.push(Tok::Var(name));
                i = j;
            }
            b'0'..=b'9' | b'.' => {
                let start = i;
                let mut j = i;
                let mut saw_dot = false;
                let mut saw_e = false;
                while j < bytes.len() {
                    let ch = bytes[j];
                    if ch.is_ascii_digit() {
                        j += 1;
                    } else if ch == b'.' && !saw_dot && !saw_e {
                        saw_dot = true;
                        j += 1;
                    } else if (ch == b'e' || ch == b'E') && !saw_e {
                        saw_e = true;
                        j += 1;
                        if j < bytes.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
                            j += 1;
                        }
                    } else {
                        break;
                    }
                }
                let s = std::str::from_utf8(&bytes[start..j])
                    .map_err(|e| format!("bad utf8 in number: {e}"))?;
                let v = s.parse::<f64>().map_err(|e| format!("bad number '{s}': {e}"))?;
                tokens.push(Tok::Num(v));
                i = j;
            }
            c if c.is_ascii_alphabetic() || c == b'_' => {
                let start = i;
                let mut j = i;
                while j < bytes.len()
                    && (bytes[j].is_ascii_alphanumeric() || bytes[j] == b'_')
                {
                    j += 1;
                }
                let name = std::str::from_utf8(&bytes[start..j])
                    .map_err(|e| format!("bad utf8: {e}"))?
                    .to_string();
                // If followed by '(' it's a function call; otherwise treat as
                // an unknown bare identifier (which surfaces as a parameter
                // lookup error at evaluation time, telling the user they
                // probably meant `$name`).
                let mut k = j;
                while k < bytes.len() && (bytes[k] == b' ' || bytes[k] == b'\t') {
                    k += 1;
                }
                if k < bytes.len() && bytes[k] == b'(' {
                    tokens.push(Tok::Func(name));
                } else {
                    return Err(format!(
                        "bare identifier '{name}' at byte {i} — \
                         did you mean ${name}?  (function calls need a '(' next)"
                    ));
                }
                i = j;
            }
            _ => return Err(format!("unexpected character '{}' at byte {i}", c as char)),
        }
    }
    tokens.push(Tok::End);
    Ok(tokens)
}

struct Parser<'a> {
    tokens: &'a [Tok],
    pos: usize,
    params: &'a HashMap<String, f64>,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> &Tok {
        &self.tokens[self.pos]
    }
    fn advance(&mut self) -> Tok {
        let t = self.tokens[self.pos].clone();
        self.pos += 1;
        t
    }
    fn expect(&mut self, want: &Tok) -> Result<(), String> {
        if std::mem::discriminant(self.peek()) == std::mem::discriminant(want) {
            self.advance();
            Ok(())
        } else {
            Err(format!("expected {want:?}, got {:?}", self.peek()))
        }
    }
    // expr := term (('+'|'-') term)*
    fn parse_expr(&mut self) -> Result<f64, String> {
        let mut acc = self.parse_term()?;
        loop {
            match self.peek() {
                Tok::Plus => { self.advance(); acc += self.parse_term()?; }
                Tok::Minus => { self.advance(); acc -= self.parse_term()?; }
                _ => return Ok(acc),
            }
        }
    }
    // term := unary (('*'|'/') unary)*
    fn parse_term(&mut self) -> Result<f64, String> {
        let mut acc = self.parse_unary()?;
        loop {
            match self.peek() {
                Tok::Star => { self.advance(); acc *= self.parse_unary()?; }
                Tok::Slash => {
                    self.advance();
                    let denom = self.parse_unary()?;
                    if denom == 0.0 {
                        return Err("division by zero".into());
                    }
                    acc /= denom;
                }
                _ => return Ok(acc),
            }
        }
    }
    // unary := '-' unary | '+' unary | atom
    fn parse_unary(&mut self) -> Result<f64, String> {
        match self.peek() {
            Tok::Minus => { self.advance(); Ok(-self.parse_unary()?) }
            Tok::Plus => { self.advance(); self.parse_unary() }
            _ => self.parse_atom(),
        }
    }
    // atom := Num | Var | Func '(' args ')' | '(' expr ')'
    fn parse_atom(&mut self) -> Result<f64, String> {
        match self.peek().clone() {
            Tok::Num(n) => { self.advance(); Ok(n) }
            Tok::Var(name) => {
                self.advance();
                self.params
                    .get(&name)
                    .copied()
                    .ok_or_else(|| format!("unknown parameter '${name}'"))
            }
            Tok::Func(name) => {
                self.advance();
                self.expect(&Tok::LParen)?;
                let mut args = vec![self.parse_expr()?];
                while matches!(self.peek(), Tok::Comma) {
                    self.advance();
                    args.push(self.parse_expr()?);
                }
                self.expect(&Tok::RParen)?;
                eval_func(&name, &args)
            }
            Tok::LParen => {
                self.advance();
                let v = self.parse_expr()?;
                self.expect(&Tok::RParen)?;
                Ok(v)
            }
            other => Err(format!("unexpected {other:?}")),
        }
    }
}

fn eval_func(name: &str, args: &[f64]) -> Result<f64, String> {
    match (name, args.len()) {
        ("sin", 1) => Ok(args[0].sin()),
        ("cos", 1) => Ok(args[0].cos()),
        ("tan", 1) => Ok(args[0].tan()),
        ("sqrt", 1) => {
            if args[0] < 0.0 {
                Err(format!("sqrt of negative: {}", args[0]))
            } else {
                Ok(args[0].sqrt())
            }
        }
        ("abs", 1) => Ok(args[0].abs()),
        ("min", 2) => Ok(args[0].min(args[1])),
        ("max", 2) => Ok(args[0].max(args[1])),
        ("floor", 1) => Ok(args[0].floor()),
        ("ceil", 1) => Ok(args[0].ceil()),
        ("round", 1) => Ok(args[0].round()),
        _ => Err(format!(
            "unknown function or wrong arity: {name}({} args)",
            args.len()
        )),
    }
}

fn evaluate_expression(src: &str, params: &HashMap<String, f64>) -> Result<f64, String> {
    let tokens = tokenize(src)?;
    let mut p = Parser {
        tokens: &tokens,
        pos: 0,
        params,
    };
    let v = p.parse_expr()?;
    if !matches!(p.peek(), Tok::End) {
        return Err(format!(
            "trailing input after expression: {:?}",
            &p.tokens[p.pos..]
        ));
    }
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(src: &str, params: &[(&str, f64)]) -> Result<f64, String> {
        let map: HashMap<String, f64> =
            params.iter().map(|(k, v)| (k.to_string(), *v)).collect();
        evaluate_expression(src, &map)
    }

    #[test]
    fn basic_arithmetic() {
        assert_eq!(ev("1 + 2 * 3", &[]).unwrap(), 7.0);
        assert_eq!(ev("(1 + 2) * 3", &[]).unwrap(), 9.0);
        assert_eq!(ev("10 / 4", &[]).unwrap(), 2.5);
        assert_eq!(ev("-5 + 3", &[]).unwrap(), -2.0);
    }

    #[test]
    fn variables() {
        assert_eq!(ev("$x + 1", &[("x", 5.0)]).unwrap(), 6.0);
        assert_eq!(ev("$a * $b", &[("a", 3.0), ("b", 4.0)]).unwrap(), 12.0);
    }

    #[test]
    fn functions() {
        assert!((ev("sqrt(16)", &[]).unwrap() - 4.0).abs() < 1e-12);
        assert!((ev("abs(-7.5)", &[]).unwrap() - 7.5).abs() < 1e-12);
        assert!((ev("min(3, 5)", &[]).unwrap() - 3.0).abs() < 1e-12);
        assert!((ev("max(3, 5)", &[]).unwrap() - 5.0).abs() < 1e-12);
    }

    #[test]
    fn missing_var_errors() {
        assert!(ev("$nope", &[]).is_err());
    }

    #[test]
    fn malformed_errors() {
        // unclosed paren
        assert!(ev("(1 + 2", &[]).is_err());
        // trailing operator
        assert!(ev("1 +", &[]).is_err());
        // empty
        assert!(ev("", &[]).is_err());
        // bare identifier (must be $name or func call)
        assert!(ev("x + 1", &[]).is_err());
    }
}
