use std::fmt;
use std::hash::Hash;

#[derive(Eq, PartialEq, Copy, Clone, Hash, Debug)]
pub struct ACoord {
    pub x: isize,
    pub y: isize,
}

pub const CARDINALS: [ACoord; 4] = [
    ACoord { x: 1, y: 0 },
    ACoord { x: 0, y: 1 },
    ACoord { x: -1, y: 0 },
    ACoord { x: 0, y: -1 },
];

impl fmt::Display for ACoord {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let _ = writeln!(f, "(x: {:?}, y: {:?})", self.x, self.y);
        Ok(())
    }
}
