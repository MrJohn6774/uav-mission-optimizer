import aerosandbox as asb
from aerosandbox.atmosphere import Atmosphere as atmo
from aerosandbox.library import aerodynamics as aero
import aerosandbox.numpy as np
from aerosandbox.tools import units as u
import copy
from typing import Union, Callable, Optional
from models.Aircraft import Aircraft


ac: Aircraft = None


def main():
    problem = setup()
    solutions = problem.solve()

    ac.substitute_solution(solutions)
    ac.draw_three_view()


def setup() -> asb.Opti:
    ## Initialize optimizer
    opti = asb.Opti()

    global ac
    ac = Aircraft(opti)

    mission_setup(ac)

    return opti


## mission modeling
def mission_setup(aircraft: Aircraft):
    #### Environment setup
    atmosphere = atmo(altitude=100)

    opti = aircraft.opti

    op_pt = asb.OperatingPoint(
        atmosphere=atmosphere
    )

    aero = asb.AeroBuildup(
        airplane=aircraft,
        op_point=op_pt,
        include_wave_drag=False
    ).run_with_stability_derivatives(
        alpha=True,
        beta=True,
        p=False,
        q=False,
        r=False,
    )

    static_margin = (aero["x_np"] - aircraft.mass_props_TOGW.x_cg) / aircraft.wing.mean_aerodynamic_chord()

    # objective = aircraft.weights['payload'] * lap_number + antenna_length / lap_time

    # opti.minimize(objective)


if __name__ == '__main__':
    main()
