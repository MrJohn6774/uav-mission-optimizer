import aerosandbox as asb
from aerosandbox.atmosphere import Atmosphere as atmo
from aerosandbox.library import aerodynamics as aero
from aerosandbox.library import propulsion_electric, propulsion_propeller
import aerosandbox.numpy as np
from aerosandbox.tools import units as u
import copy
import math
from scipy.optimize import fsolve
from typing import Union, Callable, Optional


def main() -> int:
    opti = asb.Opti()

    ####
    takeoff_altitude = 728    # [m]
    safety_factor = 1.5
    limit_load_factor = 3
    ultimate_load_factor = limit_load_factor * 1.5
    mass_total = opti.variable(init_guess=3, scale=0.01, upper_bound=7)    # [kg]
    masses = {}    # list of components' mass [kg]

    # ------------------------------------------------------------------
    #  Payload
    mass_electronic_package = opti.variable(init_guess=2, scale=0.1) * u.kg    # mass of payload inside fuselage

    antenna_length = opti.variable(init_guess=0.1, scale=0.01, lower_bound=0.05) * u.meter    # antenna length
    mass_antenna = antenna_length * 0.238 * u.kg     # mass of antenna [kg] (https://www.engineeringtoolbox.com/pvc-cpvc-pipes-dimensions-d_795.html)
    mass_ballast = mass_antenna

    # ------------------------------------------------------------------
    #  Propulsion
    ## Battery
    batt_num_cell = np.floor(opti.variable(init_guess=4, scale=1, lower_bound=2, upper_bound=6))     # number of cells per battery pack (16.4V - 12V)

    batt_nominal_voltage = 3.7 * batt_num_cell               # [V]
    batt_discharge_rating = opti.variable(init_guess=20, scale=5, lower_bound=20, upper_bound=100)    # [C]
    batt_capacity_per_pack = opti.variable(init_guess=4500, scale=100, upper_bound=7000)             # [mAh]
    batt_max_discharge_current = batt_capacity_per_pack / 1000 * batt_discharge_rating    # [A]
    batt_energy_per_pack = batt_capacity_per_pack / 1000 * batt_nominal_voltage      # [Wh]

    opti.subject_to(batt_energy_per_pack < 100)

    ## Motor
    motor_count = np.floor(opti.variable(init_guess=1, scale=1, lower_bound=1))
    Kv = opti.variable(init_guess=400, scale=10, lower_bound=400, upper_bound=1355)

    ## Propellor
    prop_diameter = opti.variable(init_guess=9, scale=0.1, lower_bound=0.1) * u.inch    # [m]
    # prop_pitch = opti.variable()


    # ------------------------------------------------------------------
    #  Aerodynamics
    wing_span = opti.variable(init_guess=1.5, scale=0.01)    # [m]
    wing_area = opti.variable(init_guess=0.5, scale=0.01)    # [m^2]
    wing_ar = wing_span**2 / wing_area

    # thin airfoil theory
    C_L_max = 1.2    # assume no flaps
    C_L = opti.variable(init_guess=0.8, scale=0.01, lower_bound=0, upper_bound=C_L_max)
    k = 1 / (np.pi * wing_ar * 0.9)
    C_D = 0.04 + k * C_L**2


    # ------------------------------------------------------------------
    #  Performance
    ## initialize atmosphere
    atmosphere = atmo(altitude=takeoff_altitude)
    g = 9.81

    ## takeoff requirement
    ## takeoff run available 60ft
    thrust_at_to = opti.variable(init_guess=10, scale=0.1, lower_bound=1, upper_bound=100)    # [N]
    v_stall = np.sqrt(2*mass_total*g / (atmosphere.density() * wing_area * C_L_max))
    v_lo = 1.2 * v_stall
    C_L_to = 0.694 * C_L_max

    # Introduction to Aircraft Performance p. 167
    friction_coeff = 0.05    # hard turf from Table 7.1
    omega = np.sqrt(mass_total*g / (0.5*atmosphere.density()*wing_area*(C_D - friction_coeff*C_L_to)))
    thrust_to_weight = thrust_at_to / (mass_total * g)

    # distance to to lift off (assume constant thrust as V_r is well below mach .1)
    x_lo = omega**2/(2*g) * np.log((thrust_to_weight - friction_coeff) / (thrust_to_weight - friction_coeff - (v_lo/omega)**2))

    # Equation 7.15
    D_lo = 0.5 * atmosphere.density() * v_lo**2 * wing_area * C_D
    x_climb_to_5ft = mass_total*g / (thrust_at_to - D_lo) * (5*u.ft + v_stall**2/(8*g))
    x_to = x_lo + x_climb_to_5ft

    opti.subject_to(x_to < 60 * u.ft)

    # endurance

    # ------------------------------------------------------------------
    #  Mass estimation
    # mass_batt_per_pack = propulsion_electric.mass_battery_pack(batt_energy_per_pack) * u.kg
    mass_batt_per_pack =  (0.09522 * batt_capacity_per_pack - 0.8383 * batt_max_discharge_current + 4.34e-5 * batt_capacity_per_pack * batt_max_discharge_current + 34.3982 * batt_nominal_voltage - 365.0336) * u.gram

    # disk actuator theory
    disk_area = np.pi * (prop_diameter/2)**2
    max_power = 1.1 * propulsion_propeller.propeller_shaft_power_from_thrust(
        thrust_force=thrust_at_to,
        area_propulsive=disk_area,
        airspeed=v_lo,
        rho=atmosphere.density())

    mass_motor = propulsion_electric.mass_motor_electric(max_power=max_power, kv_rpm_volt=Kv, method="hobbyking")
    # assume 1 battery pack serves 1 motor
    mass_propulsion = (mass_batt_per_pack + mass_motor) * motor_count  # mass of esc, motor and propellors [kg]

    opti.subject_to(mass_batt_per_pack * motor_count < 100)

    opti.subject_to(mass_total >= sum(masses))

    # ------------------------------------------------------------------
    #  Optimization

    objective = -mass_total
    opti.minimize(objective)
    solutions = opti.solve()

    s = lambda x: solutions.value(x)

    ##### Section: Printout
    print_title = lambda s: print(s.upper().join(["*" * 20] * 2))

    def fmt(x):
        return f"{s(x):.6g}"


    print_title("Outputs")
    # for k, v in {
    #     "AUW"                   : f"{fmt(mass_total)} kg ({fmt(mass_total / u.lbm)} lbm)",
    #     "L/D (actual)"          : fmt(LD_cruise),
    #     "Cruise Airspeed"       : f"{fmt(cruise_op_point.velocity)} m/s",
    #     "Cruise AoA"            : f"{fmt(cruise_op_point.alpha)} deg",
    #     "Cruise CL"             : fmt(aero['CL']),
    #     "breakeven_climb_rate"  : f"{fmt(breakeven_climb_rate)} m/s ({fmt(breakeven_climb_rate / (u.foot / u.minute))} ft/min)",
    #     "breakeven_climb_angle" : f"{fmt(breakeven_climb_angle_deg)} deg",
    #     "Cma"                   : fmt(aero['Cma']),
    #     "Cnb"                   : fmt(aero['Cnb']),
    #     "Cm"                    : fmt(aero['Cm']),
    #     "Wing Reynolds Number"  : eng_string(cruise_op_point.reynolds(s(wing.mean_aerodynamic_chord()))),
    #     "AVL: Cma + Cma_fuse"   : avl_aero['Cma'] + aero['Cma_fuse'],
    #     "AVL: Cnb + Cnb_fuse"   : avl_aero['Cnb'] + aero['Cnb_fuse'],
    #     "AVL: Cm"               : avl_aero['Cm'],
    #     "AVL: Clb Cnr / Clr Cnb": avl_aero['Clb Cnr / Clr Cnb'] * avl_aero['Cnb'] / (
    #             avl_aero['Cnb'] + aero['Cnb_fuse']),
    #     "CG location"           : "(" + ", ".join([fmt(xyz) for xyz in mass_props_TOGW.xyz_cg]) + ") m",
    #     "Wing Span"             : f"{fmt(wing_span)} m ({fmt(wing_span / u.foot)} ft)",
    #     "Propeller Diameter"    : f"{fmt(prop_diameter)} m ({fmt(prop_diameter / u.inch)} in)",
    #     "S_ailerons / S_wing"   : fmt(aileron_area_fraction),
    # }.items():
    #     print(f"{k.rjust(25)} = {v}")


    # rpm_at_to = opti.variable(init_guess=10_000)
    # motor_perf_at_to = propulsion_electric.motor_electric_performance(rpm=rpm_at_to, kv=Kv)
    # power_at_to = motor_perf_at_to['shaft power']    # [W]

    # # calculate thrust by using disk actuator theory
    # 
    # static_thrust_at_to = (2 * atmosphere.density() * disk_area * power_at_to**2) ** (1/3)


    # ## Simulate takeoff run
    # force_y = - mass_total * 9.81
    # alpha = 0   # [radian]
    # alpha_dot = 3 * np.pi/180    # [radian] rotation rate at 3 degree per second


    # x = 0
    # airspeed = 0
    # airspeed_stall = np.sqrt(2*mass_total / (atmosphere.density() * wing_area * C_L_max))
    # friction_coeff = 0.7

    # delta_time = 1/20    # [s] 20Hz sim rate

    # while (force_y < 0 and x > 60 * u.ft):
    #     # pass v_r
    #     if airspeed > 1.2*airspeed_stall:
    #         alpha += alpha_dot * delta_time

    #     # disk actuator theory
    #     def thrust_equation(thrust):
    #         return thrust - power_at_to / (airspeed/2 + np.sqrt(airspeed**2/4 + thrust/(2*atmosphere.density()*disk_area)))
    #     thrust = fsolve(thrust_equation, 1)[0]

    #     drag = 0.5 * atmosphere.density() * airspeed**2 * wing_area * C_D - friction_coeff * (min(force_y, 0))

    #     acc_x = (thrust * np.cos(alpha) - drag) / (mass_total * 9.81)
    #     airspeed += acc_x * delta_time

    #     lift = 0.5 * atmosphere.density() * airspeed**2 * C_L * wing_area

    #     force_y = lift * np.cos(alpha) + thrust * np.sin(alpha) - mass_total * 9.81

    return 0


if __name__ == "__main__":
    main()