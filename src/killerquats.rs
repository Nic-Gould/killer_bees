
struct Ground_Reference {
	coords:[f32,f32,f32], //XYZ 
	velocity:[f32,f32,f32],	//vx,vy,vz
	bearing:[f32,f32,f32],	// azimuth, roll, elevation
	
}
struct Body_Reference {
	
	velocity:[f32,f32,f32],	//vx,vy,vz
	acceleration:[f32,f32,f32],
	
	orientation:[f32,f32,f32],	//roll, pitch, yaw - actually there's no yaw in the BCS, kooky.
	angular_velocity:[f32,f32,f32],	//roll, pitch, yaw
	angular_acceleration:[f32,f32,f32],	//roll, pitch, yaw
	
}

struct Control_Surfaces{

	elevator:f32, 	//elevator deflection positive down (radians).A positive δe produces a positive lift and a negative pitch moment.
	aileron:f32,	//aileron deflection positive left (radians).A positive δa produces a negative roll momen
	rudder:f32,		//positive nose left (radians).A positive δr producesa positive sideforce and a negative yaw moment.
	
}

struct	Dimensional_Characteristics{
	
	wing_surface_area:f32,	//m2
	wing_span:f32,			//m
	chord_length:f32,		//m
	weight:f32,				//kg
	roll_inertia:f32,		//Ixx
	pitch_inertia:f32,		//Iyy
	yaw_inertia:f32,		//Izz

}

struct Forces{
	directional:[f32,f32,f32],
	moments: [f32,f32,f32],
}

struct angle (f32);
struct Flight{
	tailwind:[f32,f32],	//
}