fn main(){
/*     struct RigidBody{
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
        
        orientation:[f32;3],	//roll, pitch, yaw - actually there's no yaw in the BCS, kooky.
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
    struct RigidBody{
        body_type: BodyType,
        control_surfaces: ControlSurfaces,
        dimensional_characteristics: DimensionalCharacteristics,
        coefficients:Coefficients,
    
        body_reference: BodyReference,
        ground_reference: GroundReference,
        forces: Forces,
        flight:Flight,
    
    }
    
    impl RigidBody{
        fn init(body_type: BodyType)->RigidBody{	// I think? I'd like to be able to pass a rocket/missile or plane.
            RigidBody{
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
            
            //quaternion magic, steal from MPU driver.
     /*        fn to_ref_frame(q:[f32;4])-> &[f32;6]{
                let a12 =   2.0_f32 * (q[1] * q[2] + q[0] * q[3]);
                let a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
                let a31 =   2.0_f32 * (q[0] * q[1] + q[2] * q[3]);
                let a32 =   2.0_f32 * (q[1] * q[3] - q[0] * q[2]);
                let a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
                let pitch = -f32::asin(a32);
                let roll  = f32::atan2(a31, a33);
                let yaw   = f32::atan2(a12, a22);
                pitch *= DEG;
                yaw   *= DEG;
                yaw   += declination; // 
                if yaw < 0.0 {yaw   += 360.0}; // Ensure yaw stays between 0 and 360
                roll  *= DEG;
                
                let lin_ax = ax + a31;
                let lin_ay = ay + a32;
                let lin_az = az - a33;
                &[pitch, yaw, roll, lin_ax, lin_ay, lin_az]
            } */
        }
    
        fn update_forces(&self){
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
}


