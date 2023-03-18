fn main(){
struct GroundReference {
	coords:[f32;3], //XYZ 
	velocity:[f32;3],	//vx,vy,vz
	bearing:[f32;3],
	velocity_total:f32,// azimuth, roll, elevation
	angle_of_attack: f32,
	sideslip: f32,
}
struct BodyReference {
	
	velocity:[f32;3],	//vx,vy,vz
	acceleration:[f32;3],
	
	orientation:[f32;3],	//roll, pitch, yaw - actually there's no yaw in the BCS, kooky.
	angular_velocity:[f32;3],	//roll, pitch, yaw
	angular_acceleration:[f32;3],	//roll, pitch, yaw
	
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

struct Forces {
	directional:[f32;3],
	moments: [f32;3],
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

enum BodyType{
	rocket,
	missile,
	plane,

}

struct RigidBody{
	body_type: BodyType,
	control_surfaces: ControlSurfaces,
	dimensional_characteristics: DimensionalCharacteristics,
	coefficients:Coefficients,

	body_reference: BodyReference,
	ground_reference: GroundReference,
	forces: Forces,

}

impl RigidBody{
	fn init(body_type: BodyType)->RigidBody{	// I think? I'd like to be able to pass a rocket/missile or plane.
		RigidBody{
		body_type = BodyType,
		control_surfaces = ControlSurfaces::new(body_type),		//lookup based on type? maybe have one plane, one missile and one rocket type for prototyping/ 
		dimensional_characteristics = DimensionalCharacteristics::new(body_type),
		coefficients = Coefficients::new(body_type),
		
		body_reference = BodyReference::new(body_type),
		ground_reference = GroundReference::new(body_type),
		forces = Forces::new(body_type),
		}
	}


	fn update_flight(){
		update_control_surfaces();
		update_environment();
		update_body_reference_system();
		update_ground_reference_system();
		update_forces();
	}	

	fn update_body_reference_system(&self){
		self.body_reference.velocity += self.body_reference.acceleration * timestep;
		self.body_reference.angular_velocity += self.body_reference.angular_acceleration * timestep;
		self.body_reference.orientation += self.body_reference.angular_velocity;
	}



	fn update_ground_reference_system(&self){
		self.ground_reference.velocity = convert_reference_frame(self.body_reference.velocity);
		
		self.ground_reference.velocity_total = 
		( self.ground_reference.velocity[0].powi(2)
		+ self.ground_reference.velocity[1].powi(2)
		+ self.ground_reference.velocity[2].powi(2)).powf(0.5)
	}

	fn convert_reference_frame(&self){
		
		//quaternion magic, steal from MPU driver.
	}

	fn update_forces(&self){
		let duplicated_calculation =	control_surfaces.elevator *((self.ground_reference.velocity_total+self.flight.tailwind)/self.ground_reference.velocity_total).powi(2)
					* dynamic_pressure*self.dimensional_characteristics.wing_surface_area;
		
		//eq) 3.3
		let lift 	= self.coefficients.lift_0 + self.coefficients.lift_alpha * alpha 
					+ self.coefficients.lift_q*dynamic_pressure *(self.dimensional_characteristics.chord_length/(2*self.ground_reference.velocity_total))
					+ self.coefficients.lift_alpha*angle_of_attack *(self.dimensional_characteristics.chord_length/(2*self.ground_reference.velocity_total))
					+ self.coefficients.lift_delta * duplicated_calculation;
					
		//eq)3.4
		let drag = self.coefficients.drag_0 + self.coefficients.drag_alpha*angle_of_attack + self.coefficients.drag_delta * self.control_surfaces.elevator
					* duplicated_calculation;
		
		
		//eq.)3.5
		let sideforce = (self.coefficients.sideforce_beta + self.coefficients.sideforce_delta * self.control_surfaces.rudder) * dynamic_pressure;
		
		
		let f_x = lift * sin(angle_of_attack) - drag * sin (angle_of_attack) - sideforce * sin(sideslip);	//eq 3.6
		let f_y = sideforce * cos(sideslip);																//eq 3.7
		let f_z = - lift * cos(angle_of_attack) - drag * sin(angle_of_attack);								//eq3.8
		
	}
}



struct Environment{
	tailwind:[f32;2], //not really environmental, it's the interface of environment and flightbody.
	altitude:f32,
	air_density:f32,
	dynamic_pressure:f32, //not really environmental, it's the interface of environment and flightbody.
	//
}
impl Environment{
	fn	wind_generator(){
		//stochastic wind generator based on met files. 
	}
	fn update_environment(){
		dynamic_pressure = 0.5*air_density*velocity_total.powi(2);
	}
}
}