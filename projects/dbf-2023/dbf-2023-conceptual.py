import aerosandbox as asb
from aerosandbox.atmosphere import Atmosphere as atmo
from aerosandbox.library import aerodynamics as aero
from aerosandbox.library import propulsion_electric
import aerosandbox.numpy as np
from aerosandbox.tools import units as u
import copy
import math
from typing import Union, Callable, Optional


def main() -> int:
    variables, constraints, problem = setup()
    solutions = problem.solve()
    show(solutions)

    return 0


def setup() -> asb.Opti:
    opti = asb.Opti

    ####
    safety_factor = 1.5

    #### Payload
    mass_electronic_package = opti.variable(init_guess=2, scale=0.1) * u.kg    # mass of payload inside fuselage

    antenna_length = opti.variable(init_guess=0.1, scale=0.01, lower_bound=0.05) * u.meter    # antenna length
    mass_antenna = antenna_length * 0.238 * u.kg     # mass of antenna [kg] (https://www.engineeringtoolbox.com/pvc-cpvc-pipes-dimensions-d_795.html)
    # mass_ballast = 

    #### Propulsion
    ## Battery
    batt_num_cell = math.floor(opti.variable(init_guess=4, scale=1, lower_bound=2, upper_bound=6))     # number of cells per battery pack (16.4V - 12V)

    batt_nominal_voltage = 3.7 * batt_num_cell               # [V]
    batt_discharge_rating = opti.variable(init_guess=20, scale=5, lower_bound=20, upper_bound=100)    # [C]
    batt_capacity_per_pack = opti.variable(init_guess=4500, scale=100, upper_bound=7000)             # [mAh]
    batt_max_discharge_current = batt_capacity_per_pack / 1000 * batt_discharge_rating    # [A]
    batt_energy_per_pack = batt_capacity_per_pack / 1000 * batt_nominal_voltage      # [Wh]
    # mass_batt_per_pack = propulsion_electric.mass_battery_pack(batt_energy_per_pack) * u.kg
    mass_batt_per_pack =  (0.09522 * batt_capacity_per_pack - 0.8383 * batt_max_discharge_current + 4.34e-5 * batt_capacity_per_pack * batt_max_discharge_current + 34.3982 * batt_nominal_voltage - 365.0336) * u.gram

    opti.subject_to(batt_energy_per_pack < 100)

    ## Motor
    num_motor = math.floor(opti.variable(init_guess=1, scale=1, lower_bound=1))
    kV = opti.variable(init_guess=400, scale=10, lower_bound=400, upper_bound=2000)
    max_power = opti.variable(init_guess=500, scale=1)    # [W]
    mass_motor = propulsion_electric.mass_motor_electric(max_power=max_power, kv_rpm_volt=kV)

    # assume 1 battery pack serves 1 motor
    mass_propulsion = (mass_batt_per_pack + mass_motor) * num_motor  # mass of esc, motor and propellors [kg]

    opti.subject_to(mass_batt_per_pack * num_motor < 100)

    ## Aerodynamics


    ## Performance
    # takeoff

    # endurance

    return opti


def show(solutions: asb.OptiSol) -> None:
    pass


if __name__ == "__main__":
    main()