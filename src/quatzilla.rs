fn mahony(mpu_data: [i16; 7], mag_data: [i16; 3], q:[f32;4]) {

                
	let ax = -mpu_data[0];
	let ay = mpu_data[1];
	let az = mpu_data[2];
	let gx = mpu_data[3]*RAD;
	let gy = -mpu_data[4]*RAD;
	let gz = -mpu_data[5]*RAD;
	let my = mag_data[0];
	let mx = mag_data[1];
	let mz = mag_data[2];

	/*         float norm;
	float vx, vy, vz;
	float ex, ey, ez;  //error terms
	float qa, qb, qc;
	static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
	float tmp; */

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	let tmp = ax * ax + ay * ay + az * az;
	if tmp !=0.0 {
		// Normalise accelerometer (assumed to measure the direction of gravity in body frame)
		let norm = fisr(tmp);
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Estimated direction of gravity in the body frame (factor of two divided out)
		let vx = q[1] * q[3] - q[0] * q[2];
		let vy = q[0] * q[1] + q[2] * q[3];
		let vz = q[0] * q[0] - 0.5 + q[3] * q[3];

		// Error is cross product between estimated and measured direction of gravity in body frame
		// (half the actual magnitude)
		let ex = ay * vz - az * vy;
		let ey = az * vx - ax * vz;
		let ez = ax * vy - ay * vx;

		// Compute and apply to gyro term the integral feedback, if enabled
		if Ki > 0.0 {
			let ix += Ki * ex * deltaT;  // integral error scaled by Ki
			let iy += Ki * ey * deltaT;
			let iz += Ki * ez * deltaT;
			gx += ix;  // apply integral feedback
			gy += iy;
			gz += iz;
		}

		// Apply proportional feedback to gyro term
		gx += Kp * ex;
		gy += Kp * ey;
		gz += Kp * ez;
	}

	// Integrate rate of change of quaternion, q cross gyro term
	deltaT = 0.5 * deltaT;
	gx *= deltaT;  // pre-multiply common factors
	gy *= deltaT;
	gz *= deltaT;
	let qa = q[0];
	let qb = q[1];
	let qc = q[2];
	q[0] += -qb * gx - qc * gy - q[3] * gz;
	q[1] += qa * gx + qc * gz - q[3] * gy;
	q[2] += qa * gy - qb * gz + q[3] * gx;
	q[3] += qa * gz + qb * gy - qc * gx;

	// renormalise quaternion
	let qnorm = fisr(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] = q[0] * qnorm;
	q[1] = q[1] * qnorm;
	q[2] = q[2] * qnorm;
	q[3] = q[3] * qnorm;
}
fn to_ref_frame(q:[f32;4])-> &[f32;6]{
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
}