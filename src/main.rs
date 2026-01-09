#![allow(unused)]

use sabrina::parser::map::readmap;
use sabrina::environment::representation::Environment;
use std::collections::{HashSet,HashMap};
use std::fmt;
use std::fs;
use std::error::Error;

fn main() {
    // let mut sample = samplepath();
    // println!("{sample:}");
    let path = "./data/sample/test0.map";
    match readmap(path) {
        Ok(env) => {
            println!("{env}");
        },
        Err(e) => {
            println!("Err\n{e:?}");
        }
    }
}
