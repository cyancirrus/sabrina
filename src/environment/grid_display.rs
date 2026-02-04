#![allow(unused)]
use crate::environment::grid::Grid;
use crate::global::types::{ACoord, Belief};
use std::error::Error;
use std::fmt;
use std::fs;

impl fmt::Display for Grid {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for y in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for x in self.bounds.min_x..=self.bounds.max_x {
                let symbol = match self.information.get(&ACoord { x, y }) {
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
