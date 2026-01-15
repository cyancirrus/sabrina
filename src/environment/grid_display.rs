#![allow(unused)]
use crate::environment::grid::{Grid, Object};
use std::error::Error;
use std::fmt;
use std::fs;


impl fmt::Display for Grid {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..=self.bounds.max_x {
                let symbol = match self.information.get(&(j, i)) {
                    // None => '\u{00b7}',
                    None => ' ',
                    Some(Object::Corner) => 'x',
                    Some(Object::Doorway) => '+',
                    Some(Object::Obstacle) => '*',
                    Some(Object::Unknown) => '?',
                    Some(Object::Wall) => '#',
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
