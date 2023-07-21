import aerosandbox as asb
from aerosandbox.atmosphere import Atmosphere as atmo
from aerosandbox.library import aerodynamics as aero
from aerosandbox.library import propulsion_electric
import aerosandbox.numpy as np
from aerosandbox.tools import units as u
import copy
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

    #### Propulsion
    ## Battery
    num_batt_cell = opti.variable(init_guess=4, upper_bound=6)     # number of cells per battery pack (16.4V - 12V)

    battery_discharge_rating = opti.variable(init_guess=20, scale=5)    # [C]
    capacity_per_battery_pack = opti.variable(init_guess=3.7, scale=0.1)             # [Ah]
    battery_nominal_voltage = 3.7               # [V]
    energy_per_battery_pack = capacity_per_battery_pack * battery_nominal_voltage      # cell capacity Ah * 3.7V  [Wh]
    mass_battery_pack = propulsion_electric.mass_battery_pack(energy_per_battery_pack)
    

    ## Motor
    num_motor = opti.variable(init_guess=1, scale=1, lower_bound=1)
    kV = opti.variable(init_guess=400, scale=10, lower_bound=400, upper_bound=2000)

    mass_motor = propulsion_electric.mass_motor_electric(kv_rpm_volt=kV)

    mass_propulsion = mass_battery_pack + mass_motor  # mass of esc, motor and propellors [kg]

    opti.subject_to(energy_per_battery_pack * num_motor < 100)


    return opti


def show(solutions: asb.OptiSol) -> None:
    pass


if __name__ == "__main__":
    main()