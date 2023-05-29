use crate::init::init;
use tokio::*;
//use crate::sim::Sim;



mod init;
mod sim;

#[tokio::main]
async fn main(){

    let mut sim=init();
    sim.update();
    sim.input();

}
