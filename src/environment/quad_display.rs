use crate::environment::quad::{Belief,QuadTree};
use std::fmt;
impl QuadTree {
    pub fn display_with_levels(&self) {
        for i in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..=self.bounds.max_x {
                match self.get_cell(&(j, i)) {
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
        for i in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..=self.bounds.max_x {
                let symbol = match self.get_cell(&(j, i)) {
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
