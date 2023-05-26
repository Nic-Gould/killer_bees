use init::init;
use lib::Sim;



mod init;
mod lib;

fn main(){

    let mut sim=init();
    sim.update();
    sim.input();

}
