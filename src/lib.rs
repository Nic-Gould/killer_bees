
use std::{f32::consts::PI, time::{SystemTime, Instant, Duration}};


pub struct NASA_MODEL{
    weight:f32,
    IXX:f32,
    IYY:f32,
    IZZ:f32,
    IXZ:f32,
    wing_surface_area:f32,
    chord_length:f32,
    wing_length:f32,
    centre_of_gravity:f32,
    theta: [f32;45],
     
    max_elevator:f32, 	
    max_ailerons:f32,	
    max_rudder:f32,		
    max_thrust:u32 
}
impl NASA_MODEL {
    
    pub fn new()->NASA_MODEL{
        let F16XL=NASA_MODEL{
            weight:27867.0,   //filthy imperials.
            IXX:18581.0,
            IYY:118803.0,
            IZZ:135198.0,
            IXZ:74.0,
            wing_surface_area:663.0,
            chord_length:24.7,
            wing_length:32.4,
            centre_of_gravity:0.1,
            theta: [0.025,-0.085,2.009,0.812,-1.046,34.81,1.736,14.85,-77.87,-13.89,0.099,0.0,0.0,0.031,0.099,-0.081,2.254,4.545,0.648,-8.237,-2.203,19.03,-25.83,-0.005,-0.202,0.058,-0.093,0.015,0.009,0.088,-0.272,-0.174,-4.315,15.47,-0.365,-18.25,0.848,0.581,0.102,-0.007,-0.282,-0.014,-0.046,0.086,-0.14],
            max_elevator:0.0, 	
            max_ailerons:0.0,	
            max_rudder:0.0,		
            max_thrust:0, 
        };
        return F16XL;
    }
}

pub struct Dynamics{

    force_x: f32,
    force_y: f32,
    force_z:  f32,
    
    accel_x: f32,
    accel_y: f32,
    accel_z: f32,
    
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    
    displacement_x:f32,
    displacement_y:f32,
    displacement_z:f32,

    
    moment_x: f32,
    moment_y: f32,
    moment_z: f32,
    
    ang_accel_x: f32,
    ang_accel_y: f32,
    ang_accel_z: f32,
    
    ang_vel_x: f32,
    ang_vel_y: f32,
    ang_vel_z: f32,

    roll: f32,
    pitch:f32,
    yaw:f32,
  

    angle_of_attack:f32,
    sideslip:f32,
    airspeed:f32,
    altitude:f32, 

    elevator:f32, 	//elevator deflection positive down (radians).A positive δe produces a positive lift and a negative pitch moment.
    ailerons:f32,	//ailerons deflection positive left (radians).A positive δa produces a negative roll momen
    rudder:f32,		//positive nose left (radians).A positive δr producesa positive sideforce and a negative yaw moment.
    thrust:f32, 
}
       

          
    
impl Dynamics {
         
   fn new(init:&Init)->Dynamics{
       
        let mut x=Dynamics{
    
            force_x: 0.0,
            force_y: 0.0,
            force_z:  0.0,
            
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: 0.0,
            
            vel_x: 0.0,
            vel_y: 0.0,
            vel_z: 0.0,
    
            displacement_x:0.0,
            displacement_y:0.0,
            displacement_z:0.0,
            
            moment_x: 0.0,
            moment_y: 0.0,
            moment_z: 0.0,
            
            ang_accel_x: 0.0,
            ang_accel_y: 0.0,
            ang_accel_z: 0.0,
            
            ang_vel_x: 0.0,
            ang_vel_y: 0.0,
            ang_vel_z: 0.0,
    
            roll:   init.roll,
            pitch:  init.pitch,
            yaw:    init.heading,
           
            
            angle_of_attack:0.0,
            sideslip:0.0,
            airspeed:0.0,
            altitude:0.0,

            elevator:0.0,
            ailerons:0.0,
            rudder:0.0,
            thrust:0.0,
        };
        return x
    } 
  
    fn update(&mut self, model: &NASA_MODEL, dt:f32, environment:&Environment){
        
        
       
        let v2          = self.airspeed*2.0;
        let alpha       = self.angle_of_attack;
        let alphaq      = alpha * self.ang_vel_y;
        let alphade     = alpha* self.elevator;
        let alpha2      = alpha * alpha;
        let alpha2q     = alpha2 * self.ang_vel_y;
        let alpha2de    = alpha2 * self.elevator;
        let alpha3      = alpha2 * alpha;
        let alpha3q     = alpha3 * self.ang_vel_y;
        let alpha3de    = alpha3 * self.elevator;
        let alpha4      = alpha3 * alpha;
        let beta        = self.sideslip;
        let beta2       = beta * beta;
        let beta3       = beta2 * beta;
        let p_wave      = self.ang_vel_x/(model.wing_length/v2);
        let q_wave      = self.ang_vel_y/(model.chord_length/v2);
        let r_wave      = self.ang_vel_z/(model.wing_length/v2);
        let da          = self.ailerons;
        let dr          = self.rudder;
        let de          = self.elevator;
        let dynamic_pressure = 0.5* environment.air_density*self.airspeed.powi(2);
        let qbar_S = dynamic_pressure*model.wing_surface_area;
        let b = model.wing_length;
        let cbar = model.chord_length;
        
        let coeff_drag      =model.theta[1] + model.theta[2]*alpha + model.theta[3]*alphaq + model.theta[4]*alphade + model.theta[5]*alpha2 + model.theta[6]*alpha2q + model.theta[7]*alpha2de + model.theta[8]*alpha3+model.theta[9]*alpha3q + model.theta[10]*alpha4;
        let coeff_sideforce =model.theta[11]*beta + model.theta[12]*p_wave + model.theta[13]*r_wave + model.theta[14]*da + model.theta[15]*dr;
        let coeff_lift      =model.theta[16] + model.theta[17]*alpha + model.theta[18]*q_wave + model.theta[19]*de + model.theta[20]*alphaq + model.theta[21]*alpha2 + model.theta[22]*alpha3 + model.theta[23]*alpha4;
        let coeff_roll      =model.theta[24]*beta + model.theta[25]*p_wave + model.theta[26]*r_wave + model.theta[27]*da + model.theta[28]*dr;
        let coeff_pitch     =model.theta[29] + model.theta[30]*alpha + model.theta[31]*q_wave + model.theta[32]*de + model.theta[33]*alphaq + model.theta[34]*alpha2q + model.theta[35]*alpha2de + model.theta[36]*alpha3q + model.theta[37]*alpha3de+model.theta[38]*alpha4;
        let coeff_yaw       =model.theta[39]*beta + model.theta[40]*p_wave + model.theta[41]*r_wave + model.theta[42]*da + model.theta[43]*dr + model.theta[44]*beta2 + model.theta[45]*beta3;
        
        let    pq 	= 	self.ang_vel_x 	* self.ang_vel_y	; 
        let    pr 	= 	self.ang_vel_x 	* self.ang_vel_z	;
        let    qr	= 	self.ang_vel_y 	* self.ang_vel_z	;
        let    qq	=	self.ang_accel_x	* self.ang_accel_x ;
        let    pp	=	self.ang_accel_y * self.ang_accel_y ;
        let    rr	=	self.ang_accel_z * self.ang_accel_z ;
        
           /*  (1) qdot =(cm*cbar*qbar_S-(IXX-IZZ)*pr -IXZ(pp-rr))/IYY;
    
            (2) cl*qbar_S*b + IYY*qr+IXZ*pr -IXZ*qr = IXX*pdot -IXZ*rdot;
            
            (3) rdot =(cn*qbar_S*b  -IXZ*qr - IYY*pq +IZZ*pq + IXZ*pdot)/IZZ;
    
            Substitute 2 into 3;
    
                
            (4) pdot= (cl*qbar_S*b + IYY*qr+IXZ*pr -IXZ*qr IXZ/IZZ*cn*qbar_S*b  -IXZ*IXZ/IZZ*qr - IYY*IXZ/IZZ*pq + IZZ*IXZ/IZZ*pq )/(IXX - IXZ*IXZ/IZZ); 
    
    
    } */
        self.ang_accel_x = (coeff_roll*qbar_S*b + model.IYY*qr+model.IXZ*pr -model.IXZ*qr +model.IXZ/model.IZZ*coeff_yaw*qbar_S*b  -model.IXZ*model.IXZ/model.IZZ*qr - model.IYY*model.IXZ/model.IZZ*pq + model.IZZ*model.IXZ/model.IZZ*pq )/(model.IXX - model.IXZ*model.IXZ/model.IZZ);
        self.ang_accel_y = (coeff_pitch*cbar*qbar_S-(model.IXX-model.IZZ)*pr -model.IXZ*(pp-rr))/model.IYY;
        self.ang_accel_z = (coeff_yaw *qbar_S*b  -model.IXZ*qr - model.IYY*pq +model.IZZ*pq + model.IXZ*self.ang_accel_x)/model.IZZ;
    
        self.ang_vel_x += self.ang_accel_x * dt;
        self.ang_vel_y += self.ang_accel_y * dt;
        self.ang_vel_z += self.ang_accel_z * dt;

        let sin_cos_alpha = alpha.sin()+alpha.cos();
        self.accel_x = (coeff_drag*qbar_S/sin_cos_alpha + self.thrust)/model.weight;// grav*2*(ex*ez - ey*e0) 
        self.accel_y = coeff_sideforce*qbar_S/model.weight;// grav*2*(ey*ez - ex*eo) 
        self.accel_z = coeff_lift*qbar_S*sin_cos_alpha/model.weight;// grav*(ez*ez+e0*e0-ex*ex-ey*ey)
                
        self.vel_x += self.accel_x * dt;
        self.vel_y += self.accel_y * dt;
        self.vel_z += self.accel_z * dt;

        
        
        
    }
    
}


struct Quat{
    e0		:f32,
    ex		:f32,
    ey		:f32,
    ez		:f32,
    delta_e0:f32,
    delta_ex:f32,
    delta_ey:f32,
    delta_ez:f32,

}
impl Quat{ //need to define the launch_ahrs
    fn new(init:&Init)->Quat{
       
        
        //quaternion initialization
        let spsi=f32::sin(init.roll);
        let cpsi=f32::cos(init.roll);
        let stht=f32::sin(init.pitch);
        let ctht=f32::cos(init.pitch);
        let sphi=f32::sin(init.heading);
        let cphi=f32::cos(init.heading);

        let quat = Quat{
            e0:cpsi*ctht*cphi+spsi*stht*sphi,
            ex:cpsi*ctht*sphi-spsi*stht*cphi,
            ey:cpsi*stht*cphi+spsi*ctht*sphi,
            ez:-cpsi*stht*sphi+spsi*ctht*cphi,
            delta_e0:0.0,
            delta_ex:0.0,
            delta_ey:0.0,
            delta_ez:0.0,
        };
        return quat;
    }
    fn	update(&mut self, dynamics:&mut Dynamics, dt:f32){
        let int_step = dt/2.0; //stepsize/2
        let erq	= 1.0-self.e0*self.e0+self.ex*self.ex+self.ey*self.ey+self.delta_ez*self.ez;
        
        let new_delta_e0	=	0.5 * ((-self.ex*dynamics.ang_vel_x) 	+ 	(self.ey*dynamics.ang_vel_y)	+	(-self.ez*dynamics.ang_vel_z));
        let new_delta_ey	=	0.5 * ((self.ez*dynamics.ang_vel_x)	    +	(self.e0*dynamics.ang_vel_y) 	-	(self.ex*dynamics.ang_vel_z));
        let new_delta_ez	=	0.5 * ((-self.ey*dynamics.ang_vel_x) 	+	(self.ex*dynamics.ang_vel_y) 	+	(self.e0*dynamics.ang_vel_z));
        let new_delta_ex	=	0.5 * ((self.e0*dynamics.ang_vel_x) 	-	(self.ez*dynamics.ang_vel_y)	+	(self.ey*dynamics.ang_vel_z));
        
        self.e0 += (self.delta_e0+new_delta_e0)* erq * int_step;
        self.ex += (self.delta_ex+new_delta_ey)* erq * int_step;
        self.ey += (self.delta_ey+new_delta_ez)* erq * int_step;
        self.ez += (self.delta_ez+new_delta_ex)* erq * int_step;

        self.delta_e0 = new_delta_e0;
        self.delta_ex = new_delta_ey;
        self.delta_ey = new_delta_ez;
        self.delta_ez = new_delta_ex;
        
        let a12 =   2.0_f32 * (self.ex * self.ey + self.e0 * self.ez);
        let a22 =   self.e0 * self.e0 + self.ex * self.ex - self.ey * self.ey - self.ez * self.ez;
        let a31 =   2.0_f32 * (self.e0 * self.ex + self.ey * self.ez);
        let a32 =   2.0_f32 * (self.ex * self.ez - self.e0 * self.ey);
        let a33 =   self.e0 * self.e0 - self.ex * self.ex - self.ey * self.ey + self.ez * self.ez;
        
        dynamics.pitch = -f32::asin(a32);
        dynamics.roll  = f32::atan2(a31, a33);
        dynamics.yaw   = f32::atan2(a12, a22)%(2.0*PI);
            
            
    }
}

struct Environment{ 
    air_density:f32,
    reference_density:f32,
    standard_temperature:f32,
    temperature_lapse_rate:f32,
    reference_height:f32,
    gravity:f32,
    gas_constant:f32,
    air_molar_mass:f32,
    wind_speed:f32,
    wind_direction:f32,
}
impl Environment{
    fn	wind_generator(){
        //stochastic wind generator?. 
         
    }
    fn new(init:&Init)->Environment{
        let mut out = Environment{
            air_density             :1.2250,
            reference_height        :0.0,
            reference_density       :1.22501,
            standard_temperature    :288.15,
            temperature_lapse_rate  :-0.0065, 
            gravity                 :9.80665,
            gas_constant            :8.3144598,
            air_molar_mass          :0.0289644,
            wind_speed              :init.wind_speed,
            wind_direction          :init.wind_direction,
        };
        out.update(init.altitude);
        return out
    }

    fn update(&mut self, altitude:f32){
        self.update_atmosphere(altitude);
        self.get_air_density(altitude);
    }

    fn get_air_density(&mut self, altitude:f32){
            self.air_density = self.reference_density*(self.standard_temperature
                                        /(self.standard_temperature+(altitude-self.reference_height)*self.temperature_lapse_rate))
                                        *(1.0+(self.gravity*self.air_molar_mass)/(self.gas_constant*self.temperature_lapse_rate))
    }

    fn update_atmosphere(&self, altitude:f32)->[f32;4]{
                        //match sucks
                        //reference_height, reference_density, standard_temperature, temperature_lapse_rate

            if altitude < 11_000.0  {[0.0,         1.2250,      288.15, 	-0.0065 ]}
        else if altitude < 20_000.0 {[11_000.0,    0.36391,     216.65, 	0.0     ]}   
        else if altitude < 32_000.0 {[20_000.0,    0.08803,     216.65, 	0.001   ]}	    
        else if altitude < 47_000.0 {[32_000.0,    0.01322,     228.65, 	0.0028 	]}   
        else if altitude < 51_000.0 {[47_000.0,    0.00143,     270.65, 	0.0 	]}
        else if altitude < 71_000.0 {[51_000.0,    0.00086,     270.65, 	-0.0028 ]}	
        else                        {[71_000.0,    0.000064,    214.65, 	-0.002 	]}    
        
    }
}
pub struct  Init{
    roll: f32,
    pitch: f32,
    heading: f32,
   
    wind_speed: f32,
    wind_direction: f32,
    altitude: f32, 
}

pub struct Sim {
    model:NASA_MODEL,
    dynamics:Dynamics,
    quat:Quat,
    environment:Environment,
    timer:Instant,
    dt:f32,
}
impl Sim{
    pub fn new(init:Init)->Sim{
        Sim {
            model:NASA_MODEL::new(),
            dynamics: Dynamics::new(&init),
            quat:Quat::new(&init),
            environment: Environment::new(&init),
            timer: Instant::now(),
            dt:0.1,
        }
    }
    pub async fn input(&mut self){
        let keypress = "a";
        match keypress{
            
            "w" => self.dynamics.ailerons+=1.0, 
            "s" => self.dynamics.ailerons-=1.0,
            
            "a" =>  self.dynamics.rudder-=1.0,
            "d" => self.dynamics.rudder+=1.0,
            
            "upArrow"   => self.dynamics.thrust += 1000.0,
            "downArrow" => self.dynamics.thrust -= 1000.0,
            
            "leftArrow" => self.dynamics.rudder-=1.0,
            "rightArrow"=> self.dynamics.rudder-=1.0,
            _ => return,
        }
    }
    pub async fn update(&mut self){
        let now = Instant::now();
        self.dt = Instant::duration_since(&now, self.timer).as_secs_f32();
        self.timer=now;

        self.dynamics.update(&self.model, self.dt, &self.environment);
        self.quat.update(&mut self.dynamics, self.dt);
        self.environment.update(self.dynamics.altitude);
    }
}