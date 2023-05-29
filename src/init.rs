use crate::sim::{Init, Sim};



pub(crate) fn init()->Sim{
    let init = Init{
        roll: 0.0,      //Degrees right wing down from horizontal
        pitch:0.0,      //Degrees nose down from horizontal
        heading:0.0,    //Degrees from north
    
        wind_speed:0.0,
        wind_direction:0.0,
        altitude:0.0, 
    };


Sim::new(init)

}
