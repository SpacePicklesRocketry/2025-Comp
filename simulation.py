from rocketpy import Environment, SolidMotor, Rocket, Flight
import datetime

env = Environment(latitude=32.990254, longitude=-106.974998, elevation=1400)

tomorrow = datetime.date.today() + datetime.timedelta(days=1)
env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))

env.set_atmospheric_model(type="Forecast", file="GFS")
env.max_expected_height = 5000 
env.info()

thrust_curve = [
    (0, 0),   
    (0.5, 500),
    (1, 1000),
    (1.5, 1500),
    (2, 1200),
    (2.5, 800),
    (3, 400),
    (3.5, 0)
]

Pro75M1670 = SolidMotor(
    thrust_source=thrust_curve,
    dry_mass=1.815,
    dry_inertia=(0.125, 0.125, 0.002),
    nozzle_radius=33 / 1000,
    grain_number=5,
    grain_density=1815,
    grain_outer_radius=33 / 1000,
    grain_initial_inner_radius=15 / 1000,
    grain_initial_height=120 / 1000,
    grain_separation=5 / 1000,
    grains_center_of_mass_position=0.397,
    center_of_dry_mass_position=0.317,
    nozzle_position=0,
    burn_time=3.5,
    throat_radius=11 / 1000,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

Pro75M1670.info()

custom_power_off_drag = [(0, 0.5), (100, 0.4), (200, 0.35), (300, 0.3), (400, 0.25)]
custom_power_on_drag = [(0, 0.6), (100, 0.45), (200, 0.4), (300, 0.35), (400, 0.3)]

calisto = Rocket(
    radius=127 / 2000,
    mass=14.426,
    inertia=(6.321, 6.321, 0.034),
    power_off_drag=custom_power_off_drag, 
    power_on_drag=custom_power_on_drag, 
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose",
)

rail_buttons = calisto.set_rail_buttons(
    upper_button_position=0.0818,
    lower_button_position=-0.618,
    angular_position=45,
)


calisto.add_motor(Pro75M1670, position=-1.255)

nose_cone = calisto.add_nose(length=0.55829, kind="vonKarman", position=1.278)

fin_set = calisto.add_trapezoidal_fins(
    n=4,
    root_chord=0.120,
    tip_chord=0.060,
    span=0.110,
    position=-1.04956,
    cant_angle=0.5,
    airfoil=None  
)
tail = calisto.add_tail(
    top_radius=0.0635, bottom_radius=0.0435, length=0.060, position=-1.194656
)

calisto.all_info()

Main = calisto.add_parachute(
    "Main",
    cd_s=10.0,
    trigger=800, 
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)
Drogue = calisto.add_parachute(
    "Drogue",
    cd_s=1.0,
    trigger="apogee",
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)

test_flight = Flight(
    rocket=calisto, 
    environment=env, 
    rail_length=5.2,  
    inclination=85,   
    heading=0,       
    verbose=True
)

test_flight.plot_trajectory_3d() 

test_flight.all_info()

test_flight.export_kml(
    file_name="trajectory.kml",
    extrude=True,
    altitude_mode="relative_to_ground",
)
