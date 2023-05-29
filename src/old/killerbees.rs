///
/// 
/// 
///   Probably wont bother re-writing much more of this. I'll start porting ideas over to main instead.
/// 
/// 
/// 


const DEBUG_FLIGHTMODEL:bool = false;

// get temperture in kelvin
fn get_air_temperature(altitude:f32)->f32 {
  return 288.15 - 0.0065 * altitude;  // kelvin
}

// only accurate for altitudes < 11km
fn get_air_density(altitude:f32) ->f32{
  assert!(0.0 <= altitude && altitude <= 11000.0);
  let temperature:f32 = get_air_temperature(altitude);  // kelvin
  let pressure:f32 = 101325.0 * std::pow(1 - 0.0065* (altitude / 288.15), 5.25);
  return 0.00348 * (pressure / temperature);
}

fn get_propellor_thrust(rigid_body: phi::RigidBody , engine_horsepower:f32, propellor_rpm:f32, propellor_diameter:f32)->f32{
  let speed:f32 = rigid_body.get_speed();
  let engine_power = phi::units::watts(engine_horsepower);

  let a = 1.83; 
  let b = -1.32;
   let propellor_advance_ratio = speed / ((propellor_rpm / 60.0) * propellor_diameter);
   let propellor_efficiency = a * propellor_advance_ratio + std::pow(b * propellor_advance_ratio, 3.0);

   let  C = 0.12;
   let air_density = get_air_density(rigid_body.position.y);
   let sea_level_air_density = get_air_density(0.0);
   let power_drop_off_factor = ((air_density / sea_level_air_density) - C) / (1 - C);

  return ((propellor_efficiency * engine_power) / speed) * power_drop_off_factor;
}

// https://aerotoolbox.com/airspeed-conversions/
fn get_indicated_air_speed(rigid_body: phi::RigidBody) {   //input type was a const ref or something
    let airspeed:f32 = rigid_body.get_speed();
    let air_density:f32 = get_air_density(rigid_body.position.y);
    let sea_level_air_density:f32 = get_air_density(0.0);
    let dynamic_pressure:f32 = 0.5 * air_density * phi::sq(airspeed);  // bernoulli's equation
      return std::sqrt(2 * dynamic_pressure / sea_level_air_density);
}

// get g-force in pitch direction
fn get_g_force(rigid_body: &phi::RigidBody) {
  let velocity = rigid_body.get_body_velocity();
  let angular_velocity = rigid_body.angular_velocity;

    // avoid division by zero
  if angular_velocity.z ==0.0{
    let turn_radius = 0.0;
  }else{  
    let turn_radius = velocity.x / angular_velocity.z; /*   (std::abs(angular_velocity.z) < phi::EPSILON) ? std::numeric_limits<float>::max(): velocity.x / angular_velocity.z; */
  }  
  
  // centrifugal force = mass * velocity^2 / radius
  // centrifugal acceleration = force / mass
  // simplified, this results in:
  let centrifugal_acceleration = phi::sq(velocity.x) / turn_radius;

  let g_force = centrifugal_acceleration / phi::EARTH_GRAVITY;
  g_force += (rigid_body.up().y * phi::EARTH_GRAVITY) / phi::EARTH_GRAVITY;  // add earth gravity
  return g_force;
}

fn get_mach_number( rigid_body: phi::RigidBody) {
  let speed = rigid_body.get_speed();
  let temperature = get_air_temperature(rigid_body.position.y);
  let speed_of_sound = std::sqrt(1.402 * 286.0 * temperature);
  return speed / speed_of_sound;
}

struct Airfoil {
  
  curve: std::vector<glm::vec3>,  // alpha, cl, cd
  min_alpha:f32,
  max_alpha:f32,
}
impl Airfoil{
  fn new (curve: std::vector<glm::vec3>)->Airfoil{ 
    Airfoil{
    curve, 
    min_alpha: curve.front().x, 
    max_alpha: curve.back().x,
    }
  }
  // get lift coefficent and drag coefficient
  fn sample(&self, alpha:f32) -> (f32,f32) {
    let max_index = data.size() - 1;

    let t = (alpha - min_alpha)/(min_alpha- max_alpha) * max_index;
    let integer = std::floor(t);
    let fractional = t - integer;
    let index = integer as i32;
     
    let value = curve[index] + fractional * (curve[index + 1]-curve[index]);
    
    return (value.y, value.z);
  }
};

struct Engine{
   throttle:  f32,
   thrust: f32,
}
impl phi::ForceEffector for Engine;
impl Engine {
 fn new(thrust:f32)->Engine{
    Engine{
      thrust,
      throttle = 0.0,
    }
 }
fn apply_forces(rigid_body: phi::RigidBody, dt:phi::Seconds) {
    rigid_body.add_relative_force([thrust * throttle, 0.0, 0.0]);
  }
}
  

struct Wing {
  const float area;
  const float wingspan;
  const float chord;
  const float aspect_ratio;
  const Airfoil* airfoil;
  const glm::vec3 normal;
  const glm::vec3 center_of_pressure;
  const float incidence;
  const float efficiency_factor;

  float lift_multiplier = 1.0;
  float drag_multiplier = 1.0;

  float deflection = 0.0;
  float control_input = 0.0;
  float min_deflection = -10.0;
  float max_deflection = +10.0;
  float max_actuator_speed = 90.0;
  float max_actuator_torque = 6000.0;
  bool is_control_surface = true;

if DEBUG_FLIGHTMODEL == false{
  log = false;
  log_timer = 0.0;
  name = "None";
}
impl phi::ForceEffector for Wing;
impl Wing{
  fn new(const glm::vec3& position, float wingspan, float chord, const Airfoil* airfoil, const glm::vec3& normal = phi::UP, float incidence = 0.0)
      : center_of_pressure(position),
        area(chord * wingspan),
        chord(chord),
        wingspan(wingspan),
        airfoil(airfoil),
        normal(normal),
        efficiency_factor(1.0),
        incidence(incidence),
        aspect_ratio(std::pow(wingspan, 2) / area) {}

  // controls how much the wing is deflected
  void set_control_input(float input) { control_input = glm::clamp(input, -1.0, 1.0); }

  // how far the wing can be deflected, degrees
  void set_deflection_limits(float min, float max) { min_deflection = min, max_deflection = max; }

  // compute and apply aerodynamic forces
  void apply_forces(phi::RigidBody& rigid_body, phi::Seconds dt) override {
    glm::vec3 local_velocity = rigid_body.get_point_velocity(center_of_pressure);
    float speed = glm::length(local_velocity);

    if (speed <= phi::EPSILON) return;

    // control surfaces can be rotated
    glm::vec3 wing_normal = is_control_surface ? deflect_wing(rigid_body, dt) : normal;

    // drag acts in the opposite direction of velocity
    glm::vec3 drag_direction = glm::normalize(-local_velocity);

    // lift is always perpendicular to drag
    glm::vec3 lift_direction = glm::normalize(glm::cross(glm::cross(drag_direction, wing_normal), drag_direction));

    // angle between chord line and air flow
    float angle_of_attack = glm::degrees(std::asin(glm::dot(drag_direction, wing_normal)));

    // sample our aerodynamic data
    auto [lift_coefficient, drag_coefficient] = airfoil->sample(angle_of_attack);

    // induced drag
    float induced_drag_coefficient = std::pow(lift_coefficient, 2) / (phi::PI * aspect_ratio * efficiency_factor);

    // air density depends on altitude
    float air_density = get_air_density(rigid_body.position.y);

    float tmp = 0.5f * std::pow(speed, 2) * air_density * area;
    glm::vec3 lift = lift_direction * lift_coefficient * lift_multiplier * tmp;
    glm::vec3 drag = drag_direction * (drag_coefficient + induced_drag_coefficient) * drag_multiplier * tmp;

#if DEBUG_FLIGHTMODEL
    if (log && (log_timer -= dt) <= 0.0) {
      log_timer = 0.2f;

      auto force = rigid_body.transform_direction(lift);
      auto torque = glm::cross(center_of_pressure, lift);

      printf("[%s] aoa = %.2f, cl = %.2f, t = %.2f, p = %.2f\n", name.c_str(), angle_of_attack, lift_coefficient,
             torque.z, rigid_body.angular_velocity.z);
    }
#endif

    // aerodynamic forces are applied at the center of pressure
    rigid_body.add_force_at_point(lift + drag, center_of_pressure);
  }

  // returns updated wing normal according to control input and deflection
  glm::vec3 deflect_wing(phi::RigidBody& rigid_body, phi::Seconds dt) {
    // with increased speed control surfaces become harder to move
    float torque = std::pow(rigid_body.get_speed(), 2) * area;
    float compression = glm::degrees(std::asin(max_actuator_torque / torque));
    compression = glm::clamp(compression, 0.0, 1.0);

    float target_deflection =
        (control_input >= 0.0 ? max_deflection : min_deflection) * compression * std::abs(control_input);
    deflection = phi::move_towards(deflection, target_deflection, max_actuator_speed * compression * dt);

    auto axis = glm::normalize(glm::cross(phi::FORWARD, normal));
    auto rotation = glm::rotate(glm::mat4(1.0), glm::radians(incidence + deflection), axis);
    return glm::vec3(rotation * glm::vec4(normal, 1.0));
  }
};

struct Airplane {
  Engine engine;
  std::vector<Wing> wings;
  phi::RigidBody rigid_body;
  glm::vec3 joystick{};  // roll, yaw, pitch

  Airplane(float mass, float thrust, glm::mat3 inertia, std::vector<Wing> elements)
      : wings(elements), rigid_body({.mass = mass, .inertia = inertia}), engine(thrust) {
#if DEBUG_FLIGHTMODEL
    wings[0].log = false;
    wings[0].name = "lw";

    wings[1].log = false;
    wings[1].name = "la";

    wings[2].log = true;
    wings[2].name = "ra";

    wings[3].log = false;
    wings[3].name = "rw";

    wings[4].log = false;
    wings[4].name = "el";

    wings[5].log = false;
    wings[5].name = "rd";
#endif

    wings[0].is_control_surface = false;
    wings[1].set_deflection_limits(-10.0, 10.0);
    wings[2].set_deflection_limits(-10.0, 10.0);
    wings[3].is_control_surface = false;
    wings[4].set_deflection_limits(-12.0, 12.0);
    wings[5].set_deflection_limits(-5.0, 5.0);

    wings[1].max_actuator_torque = 2000.0;
    wings[2].max_actuator_torque = 2000.0;
  }

  fn update(dt: phi::Seconds) {
  if 1 {  //NOOO Black Magick
    let aileron:f32 = joystick.x; 
    let rudder:f32 = joystick.y; 
    let elevator:f32 = joystick.z;

    wings[1].set_control_input(+aileron);
    wings[2].set_control_input(-aileron);
    wings[4].set_control_input(-elevator);
    wings[5].set_control_input(-rudder);
  else
    rigid_body.add_relative_torque(glm::vec3(400000.0, 100000.0, 1500000.0) * joystick);
  endif

  for (wings: Wing& wing) {
      wing.apply_forces(rigid_body, dt);
    }

    engine.apply_forces(rigid_body, dt);

    rigid_body.update(dt);
  }
};