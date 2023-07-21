import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.tools import units as u
import pandas as pd


"""
Coordinate system:

Geometry axes. Datum (0, 0, 0) is coincident with the quarter-chord-point of the centerline cross section of the main 
wing.

"""
class Aircraft(asb.Airplane):
    def __init__(self, opti: asb.Opti):
        self.opti = opti

        self.wing_span = self.opti.variable(init_guess=1, upper_bound=3)
        self.wing_dihedral_angle_deg = self.opti.variable(init_guess=0, upper_bound=3)
        self.aspect_ratio = self.opti.variable(init_guess=7, lower_bound=4, upper_bound=12)


        self.weights = {}

        ### Define x-stations
        x_nose = self.opti.variable(
            init_guess=-0.1,
            upper_bound=1e-3,
        )
        x_tail = self.opti.variable(
            init_guess=0.7,
            lower_bound=1e-3
        )

        ### Wing
        self.wing_root_chord = self.opti.variable(
            init_guess=0.15,
            lower_bound=1e-3
        )

        wing_ys = np.sinspace(
            0,
            self.wing_span / 2,
            11,
            reverse_spacing=True
        )

        self.wing = asb.Wing(
            name="Main Wing",
            symmetric=True,
            xsecs=[
                asb.WingXSec(
                    xyz_le=self.wing_rot([
                        -self.wing_chord(wing_ys[i]),
                        wing_ys[i],
                        0
                    ]),
                    chord=self.wing_chord(wing_ys[i]),
                    airfoil=None,
                    twist=self.wing_twist(wing_ys[i]),
                )
                for i in range(np.length(wing_ys))
            ]
        ).translate([
            0.75 * self.wing_root_chord,
            0,
            0
        ])

        self.vtail_dihedral_angle_deg = self.opti.variable(init_guess=0, upper_bound=45)
        vtail_twist = self.opti.variable(init_guess=0, lower_bound=-15, upper_bound=15)

        vtail_shape_data = pd.DataFrame(
            {
                "x_le_inches" : [0, 0.28, 0.63, 0.93, 1.25, 1.75],
                "y_le_inches" : [0, 2, 4, 5, 5.58, 6],
                "chord_inches": [2.75, 2.50, 2.06, 1.72, 1.36, 0.75]
            }
        )

        self.vtail = asb.Wing(
            name="VTail",
            symmetric=True,
            xsecs=[
                asb.WingXSec(
                    xyz_le=self.vtail_rot([
                        row["x_le_inches"] * u.inch,
                        row["y_le_inches"] * u.inch,
                        0
                    ]),
                    chord=row["chord_inches"] * u.inch,
                    twist=vtail_twist,
                    airfoil=None
                )
                for i, row in vtail_shape_data.iterrows()
            ]
        ).translate([
            x_tail - 0.7 * vtail_shape_data.iloc[0, :]["chord_inches"] * u.inch,
            0,
            0
        ])

        self.fuselage = asb.Fuselage(
            name="Fuselage",
            xsecs=[
                asb.FuselageXSec(
                    xyz_c=[x_nose, 0, 0],
                    radius=7e-3 / 2
                ),
                asb.FuselageXSec(
                    xyz_c=[x_tail, 0, 0],
                    radius=7e-3 / 2
                )
            ]
        )

        # self.opti.subject_to([
        #     x_nose < -0.25 * self.wing_root_chord - 0.5 * u.inch,  # propeller must extend in front of wing
        #     x_tail - x_nose < 0.826,  # due to the length of carbon tube I have
        #     self.vtail.area() * np.cosd(self.vtail_dihedral_angle_deg) ** 2 * x_tail / (
        #                 self.wing.area() * self.wing.mean_aerodynamic_chord()) > 0.25
        # ])

        super().__init__(
            name="uav",
            wings=[
                self.wing,
                self.vtail
            ],
            fuselages=[self.fuselage]
        )


    def wing_rot(self, xyz):
        dihedral_rot = np.rotation_matrix_3D(
            angle=np.radians(self.wing_dihedral_angle_deg),
            axis="X"
        )

        return dihedral_rot @ np.array(xyz)


    def wing_chord(self, y):
        spanfrac = y / (self.wing_span / 2)
        chordfrac = 1 - 0.4 * spanfrac - 0.47 * (1 - (1 - spanfrac ** 2 + 1e-16) ** 0.5)

        # c_over_c_root = 0.1 + 0.9 * (1 - (y / half_span) ** 2) ** 0.5
        return chordfrac * self.wing_root_chord


    def wing_twist(self, y):
        return np.zeros_like(y)


    def vtail_rot(self, xyz):
        dihedral_rot = np.rotation_matrix_3D(
            angle=np.radians(self.vtail_dihedral_angle_deg),
            axis="X"
        )

        return dihedral_rot @ np.array(xyz)



