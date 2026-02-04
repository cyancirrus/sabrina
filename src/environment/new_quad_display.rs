use crate::environment::new_quad::QuadTree;
use crate::global::types::{ACoord, Belief};
use std::fmt;
impl QuadTree {
    pub fn display_with_levels(&self) {
        for y in (self.seen.min_y..=self.seen.max_y).rev() {
            let mut line = String::new();
            for x in self.seen.min_x..=self.seen.max_x {
                match self.get_cell(ACoord{ x, y }) {
                    None => line.push_str("[ ]"),
                    Some((lvl, Belief::Occupied)) => line.push_str(&format!("[{lvl:}]")),
                    Some((lvl, Belief::Unknown)) => line.push_str(&format!("[{lvl:}]")),
                    Some((lvl, Belief::Free)) => line.push_str(&format!("[{lvl:}]")),
                };
            }
            println!("{}", line);
        }
    }
}

impl fmt::Display for QuadTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for y in (self.seen.min_y..=self.seen.max_y).rev() {
            let mut line = String::new();
            for x in self.seen.min_x..=self.seen.max_x {
                let symbol = match self.get_cell(ACoord{ x, y }) {
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
