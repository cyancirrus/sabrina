#![allow(unused)]
use crate::environment::grid::{Grid, translate};
use crate::global::types::Belief;
use std::error::Error;
use std::fmt;
use std::fs;

impl fmt::Display for Grid {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in (self.seen.min_y..=self.seen.max_y).rev() {
            let mut line = String::new();
            for j in self.seen.min_x..=self.seen.max_x {
                let symbol = match self.information.get(&(j, i)) {
                    // None => '\u{00b7}',
                    None => ' ',
                    Some(Belief::Free) => ' ',
                    Some(Belief::Occupied) => '#',
                    Some(Belief::Unknown) => '?',
                };
                line.push('[');
                line.push(symbol);
                line.push(']');
            }
            writeln!(f, "{}", line);
        }
        Ok(())
    }
}
