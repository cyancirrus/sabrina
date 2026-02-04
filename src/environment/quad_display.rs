use crate::environment::quad::QuadTree;
use crate::global::types::{ACoord, Belief};
use std::fmt;
impl fmt::Debug for QuadTree {
    fn fmt(&self, _f: &mut fmt::Formatter) -> fmt::Result {
        for y in (self.bounds.min_y..self.bounds.max_y).rev() {
            let mut line = String::new();
            for x in self.bounds.min_x..self.bounds.max_x {
                match self.get_coord(ACoord { x, y }) {
                    None => line.push_str("[ ]"),
                    Some((lvl, Belief::Occupied)) => line.push_str(&format!("[{lvl:}]")),
                    Some((lvl, Belief::Unknown)) => line.push_str(&format!("[{lvl:}]")),
                    Some((lvl, Belief::Free)) => line.push_str(&format!("[{lvl:}]")),
                };
            }
            println!("{}", line);
        }
        Ok(())
    }
}

impl fmt::Display for QuadTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for y in (self.bounds.min_y..self.bounds.max_y).rev() {
            let mut line = String::new();
            for x in self.bounds.min_x..self.bounds.max_x {
                let symbol = match self.get_coord(ACoord { x, y }) {
                    None => 'x',
                    Some((_, Belief::Free)) => ' ',
                    Some((_, Belief::Occupied)) => '#',
                    Some((_, Belief::Unknown)) => '?',
                };
                line.push('[');
                line.push(symbol);
                line.push(']');
            }
            let _ = writeln!(f, "{}", line);
        }
        Ok(())
    }
}
