use ndarray::*;

fn main(){
/*     struct FlightBody{
        body_type: BodyType,
        control_surfaces: ControlSurfaces,
        dimensional_characteristics: DimensionalCharacteristics,
        coefficients:Coefficients,
    
        body_reference: BodyReference,
        ground_reference: GroundReference,
        forces: Forces,
        flight:Flight,
    
    }  */
    enum BodyType{
        Rocket,
        Missile,
        Plane,
    
    }
    struct ControlSurfaces{
    
        elevator:f32, 	//elevator deflection positive down (radians).A positive δe produces a positive lift and a negative pitch moment.
        aileron:f32,	//aileron deflection positive left (radians).A positive δa produces a negative roll momen
        rudder:f32,		//positive nose left (radians).A positive δr producesa positive sideforce and a negative yaw moment.
        
    }
    struct	DimensionalCharacteristics{
        
        wing_surface_area:f32,	//m2
        wing_span:f32,			//m
        chord_length:f32,		//m
        weight:f32,				//kg
        roll_inertia:f32,		//Ixx
        pitch_inertia:f32,		//Iyy
        yaw_inertia:f32,		//Izz
    
    }
    struct Coefficients {
        lift_0:f32,
        lift_alpha:  f32,
        lift_q: f32,
        lift_delta: f32,
        
        drag_0:f32,
        drag_alpha:f32,
        drag_delta:f32,
    
        sideforce_beta:f32,
        sideforce_delta: f32,
    }
    struct BodyReference {
        
        velocity:[f32;3],	//vx,vy,vz
        acceleration:[f32;3],
        
        orientation:[f32;3],	
        angular_velocity:[f32;3],	//roll, pitch, yaw
        angular_acceleration:[f32;3],	//roll, pitch, yaw
        
    }
    struct GroundReference {
        coords:[f32;3], //XYZ 
        velocity:[f32;3],	//vx,vy,vz
        bearing:[f32;3],
        velocity_total:f32,// azimuth, roll, elevation
        angle_of_attack: f32,
        sideslip: f32,
    }
    struct Forces {
        directional:[f32;3],
        moments: [f32;3],
    }
    struct Flight{
        tailwind:f32, //not really environmental, it's the interface of environment and flightbody. // need to confirm that this is already resolved into a single vector.
        altitude:f32,
        dynamic_pressure:f32, //not really environmental, it's the interface of environment and flightbody.
        //

    }
    struct InitCond{
        // init conditions describe the orientation and velocity of the flight_body at t=0
        // for initial build this will be ground launch only
        // for later builds maybe add option to start with an altidute and velicity
        //  where flight is assumed to be steady and stable with no accelerations or angular rates at t=0,
        //	might also need to initialise with a displacement vector of multiple flight bodies are being initialised at t=0	
            roll:f32,
            pitch:f32,
            yaw:f32,
        }
        
    struct FlightBody{
        ahrs:AHRS,
        forces:Forces,
        quat:Quat,
        dimensions:Dimensions,
    
    }
    struct FlightBody{
        body_type: BodyType,
        control_surfaces: ControlSurfaces,
        dimensional_characteristics: DimensionalCharacteristics,
        coefficients:Coefficients,
    
        body_reference: BodyReference,
        ground_reference: GroundReference,
        forces: Forces,
        flight:Flight,
    
    }
    struct <T> AHRS{
        where T impl Coord_system;
        x:T,					
        y:T,					
        z:T,					
    
        roll:T,					
        pitch:T,
        yaw:T,
    }	
    struct <T>Dynamics<T>{
        where T impl Coord_system;
        force_x: T,
        force_y: T,
        force_z: T,
        
        accel_x:T,
        accel_y:T,
        accel_z:T,
        
        vel_x:T,
        vel_y:T,
        vel_z:T,
        
        moment_x:T,
        moment_y:T,
        moment_z:T,
        
        ang_accel_x:T,
        ang_accel_y:T,
        ang_accel_z:T,
        
        ang_vel_x:T,
        ang_vel_y:T,
        ang_vel_z:T,
        
    }
    
    trait	Coord_System;
    struct Body_Ref(i32); 
    struct Ground_Ref(i32);
    struct Wind_Ref(i32);
    struct Stability_Ref(i32);
    impl Coord_System for Body_Ref, Ground_Ref,Wind_Ref, Stability_Ref;
    
    impl FlightBody{
        fn init(body_type: BodyType)->FlightBody{	// I think? I'd like to be able to pass a rocket/missile or plane.
            FlightBody{
            body_type: BodyType,
        // initialisation values for structs can use a lookup based on type? 
        //maybe have one plane, one missile and one rocket type for prototyping/ 
        // only relevant for some stucts, TBD.
            control_surfaces: ControlSurfaces::new(body_type),		
            dimensional_characteristics: DimensionalCharacteristics::new(body_type),
            coefficients: Coefficients::new(body_type),
            
            body_reference:  BodyReference::new(body_type),
            ground_reference: GroundReference::new(body_type),
            forces: Forces::new(body_type),
            flight: Flight::new(body_type),
            }
        }
    
        fn update_flight_body(&self){
            self.update_control_surfaces();
            self.update_body_reference_system();
            self.update_ground_reference_system();
            self.update_forces();
            self.update_flight();
        }	
    
        fn update_body_reference_system(&self){
            self.body_reference.velocity += self.body_reference.acceleration * timestep;
            self.body_reference.angular_velocity += self.body_reference.angular_acceleration * timestep;
            for i in 0..3{
                self.body_reference.orientation[i] += self.body_reference.angular_velocity[i];
            }
        }
    
        fn update_ground_reference_system(&self){
            self.convert_reference_frame();
            
            self.ground_reference.velocity_total = 
            ( self.ground_reference.velocity[0].powi(2)
            + self.ground_reference.velocity[1].powi(2)
            + self.ground_reference.velocity[2].powi(2)).powf(0.5)
        }
    
        fn convert_reference_frame(&self){
            
       
        }
    
        fn update_forces(&self){
            	//total non-gravitational forces
/* 	FAPB[0]=pdynmc*refa*cxt+thrust;
	FAPB[1]=pdynmc*refa*cyt;
	FAPB[2]=pdynmc*refa*czt;
	
	//aerodynamic moment
	FMB[0]=pdynmc*refa*refb*clt;
	FMB[1]=pdynmc*refa*refc*cmt;
	FMB[2]=pdynmc*refa*refb*cnt; */
            let duplicated_calculation =	self.control_surfaces.elevator *((self.ground_reference.velocity_total + self.flight.tailwind)/self.ground_reference.velocity_total).powi(2)
                        * self.flight.dynamic_pressure*self.dimensional_characteristics.wing_surface_area;
            
            //eq) 3.3
            let lift 	= self.coefficients.lift_0 + self.coefficients.lift_alpha * self.ground_reference.angle_of_attack 
                        + self.coefficients.lift_q*self.flight.dynamic_pressure *(self.dimensional_characteristics.chord_length/(2.0*self.ground_reference.velocity_total))
                        + self.coefficients.lift_alpha*self.ground_reference.angle_of_attack *(self.dimensional_characteristics.chord_length/(2.0*self.ground_reference.velocity_total))
                        + self.coefficients.lift_delta * duplicated_calculation;
                        
            //eq)3.4
            let drag = self.coefficients.drag_0 + self.coefficients.drag_alpha*self.ground_reference.angle_of_attack + self.coefficients.drag_delta * self.control_surfaces.elevator
                        * duplicated_calculation;
            
            
            //eq.)3.5
            let sideforce = (self.coefficients.sideforce_beta + self.coefficients.sideforce_delta * self.control_surfaces.rudder) * self.flight.dynamic_pressure;
            
            
            let f_x = lift * sin(self.ground_reference.angle_of_attack) - drag * sin(self.ground_reference.angle_of_attack) - sideforce * sin(self.ground_reference.sideslip);	//eq 3.6
            let f_y = sideforce * cos(self.ground_reference.sideslip);																                                            //eq 3.7
            let f_z = - lift * cos(self.ground_reference.angle_of_attack) - drag * sin(self.ground_reference.angle_of_attack);								                    //eq3.8
            
        }
        fn update_motion(flight:Dynamics, ahrs:AHRS, inertia_moments:arr2){ 
            let accel_x: Body_Ref = g_on_w * flight.vel_x + grav*2*(ex*ez - ey*e0) 			+ (flight.ang_vel_z*flight.vel_y) - (flight.ang_vel_y*flight.vel_z);
            let accel_y: Body_Ref = g_on_w * flight.vel_y + grav*2*(ey*ez - ex*eo) 			+ (flight.ang_vel_x*flight.vel_z) - (flight.ang_vel_z*flight.vel_x);
            let accel_z: Body_Ref = g_on_w * flight.vel_z + grav*(ez*ez+e0*e0-ex*ex-ey*ey) 	+ (flight.ang_vel_y*flight.vel_x) - (flight.ang_vel_x*flight.vel_y);
        
            let vel_x: Body_Ref += accel_x * dt;
            let vel_y: Body_Ref += accel_y * dt;
            let vel_z: Body_Ref += accel_z * dt;
        
        //
        //	LET THE DE-STUCTURING BEGIN!!!
        //
        
        pq 	= 	ang_vel_x 	* ang_vel_y	; 
        pr 	= 	ang_vel_x 	* ang_vel_z	;
        qr	= 	ang_vel_y 	* ang_vel_z	;
        qq	=	ang_accel_x	* ang_accel_x ;
        pp	=	ang_accel_y * ang_accel_y ;
        rr	=	ang_accel_z * ang_accel_z ;
        
        Ixx = inertia_moments[0][0];
        Ixy = inertia_moments[0][1];
        Ixz = inertia_moments[0][2];
        Iyy = inertia_moments[1][1];
        Iyz = inertia_moments[1][2];
        Izz = inertia_moments[2][2];
        
        
        ang_rates 	= arr1 (&	[flight.ang_vel_x, 	flight.ang_vel_y, 	flight.ang_vel_z]);
        moments 	= arr1 (&	[flight.moment_x,	flight.moment_y,	flight.moment_z	]);
        coriolis	= arr1 (&[	[(Iyy-Izz) * qr + Iyz * (qq - rr) + Ixz * pq + Ixy * pr	],
                                [(Izz-Ixx) * pr + Ixz * (rr - pp) + Ixy * qr + Iyz * pq	],
                                [(Iyy-Izz) * pq + Ixy * (pp - qq) + Iyz * pr + Ixz * qr	],
                            ]);
        
        
        ang_accel_x = inertia_moments.dot(&gyro.dot(&ang_rates)) + &moments+ &coriolis;
        
        ang_rates +=	ang_accel * dt;
        
        }
        fn update_flight(&self, environment:Environment){
          //  self.flight.tailwind = ;
          //  self.flight.altitude = ;
            self.flight.dynamic_pressure = 0.5* environment.air_density*self.ground_reference.velocity_total.powi(2);
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
        air_molar_mass:f32
    }
    impl Environment{
        fn	wind_generator(){
            //stochastic wind generator?. 
             
        }
        fn new(altitude:f32)->Environment{
            let atmos = update_atmosphere(altitude);
            let out = Environment{
                air_density             :atmos[1], //just because initialising structs is the worst.
                reference_height        :atmos[0],
                reference_density       :atmos[1],
                standard_temperature    :atmos[2],
                temperature_lapse_rate  :atmos[3], 
                gravity                 :9.80665,
                gas_constant            :8.3144598,
                air_molar_mass          :0.0289644,
            };
            get_air_density(&out, altitude);
            out
        }
    }

    fn get_air_density(environment:&Environment, altitude:f32){
            environment.air_density = environment.reference_density*(environment.standard_temperature
                                        /(environment.standard_temperature+(altitude-environment.reference_height)*environment.temperature_lapse_rate))
                                        *(1.0+(environment.gravity*environment.air_molar_mass)/(environment.gas_constant*environment.temperature_lapse_rate))
    }

    fn update_atmosphere(altitude:f32)->[f32;4]{
                        //match sucks
                        //reference_height, reference_density, standard_temperature, temperature_lapse_rate

             if altitude < 11_000.0{  [0.0,         1.2250,      288.15, 	-0.0065 ]}
        else if altitude < 20_000.0{  [11_000.0,    0.36391,     216.65, 	0.0     ]}   
        else if altitude < 32_000.0{  [20_000.0,    0.08803,     216.65, 	0.001   ]}	    
        else if altitude < 47_000.0{  [32_000.0,    0.01322,     228.65, 	0.0028 	]}   
        else if altitude < 51_000.0{  [47_000.0,    0.00143,     270.65, 	0.0 	]}
        else if altitude < 71_000.0{  [51_000.0,    0.00086,     270.65, 	-0.0028 ]}	
        else                       {  [71_000.0,    0.000064,    214.65, 	-0.002 	]}    
        
    }
    let init_cond = InitCond{
        roll:0.0,
        pitch:0.0,
        yaw:90.0,	//90 degree positive yaw = east facing launch
    };
    
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
        fn init(init_cond:Init_Cond)->Quat{
            roll 	= launch_ahrs[0];
            pitch 	= launch_ahrs[1];
            yaw 	= launch_ahrs[2];
            
            //quaternion initialization
            spsi=sin(roll);
            cpsi=cos(roll);
            stht=sin(pitch);
            ctht=cos(pitch);
            sphi=sin(yaw);
            cphi=cos(yaw);
    
            Quat{
                e0:cpsi*ctht*cphi+spsi*stht*sphi,
                ex:cpsi*ctht*sphi-spsi*stht*cphi,
                ey:cpsi*stht*cphi+spsi*ctht*sphi,
                ez:-cpsi*stht*sphi+spsi*ctht*cphi,
                delta_e0:0,
                delta_ex:0,
                delta_ey:0,
                delta_ez:0,
            }
        }
        fn	update(ang_vel_x:Body_Ref,ang_vel_y:Body_Ref,ang_vel_z:Body_Ref,){
            let int_step = model.params.int_step/2;
            let erq	= 1.0-q0*q0+q1*q1+q2*q2+q3*q3;
            
            let new_delta_e0	=	0.5 * ((-ex*ang_vel_x) 	+ 	(ey*ang_vel_y)	+	(-ez*ang_vel_z));
            let new_delta_ey	=	0.5 * ((ez*ang_vel_x)	+	(e0*ang_vel_y) 	-	(ex*ang_vel_z));
            let new_delta_ez	=	0.5 * ((-ey*ang_vel_x) 	+	(ex*ang_vel_y) 	+	(e0*ang_vel_z));
            let new_delta_ex	=	0.5 * ((e0*ang_vel_x) 	-	(ez*ang_vel_y)	+	(ey*ang_vel_z));
            
            e0 += (delta_e0+new_delta_e0)* erq * int_step;
            ex += (delta_ex+new_delta_ey)* erq * int_step;
            ey += (delta_ey+new_delta_ez)* erq * int_step;
            ez += (delta_ez+new_delta_ex)* erq * int_step;
    
            delta_e0 = new_delta_e0;
            delta_ex = new_delta_ey;
            delta_ey = new_delta_ez;
            delta_ez = new_delta_ex;
        }	
    
    }
        let inertia_moments = arr2 (&[
            [12875,		0,		-1331.4	],
            [0,			75673,	0		],
            [-1331.4,	0,		85551	]
        ]);	
}


