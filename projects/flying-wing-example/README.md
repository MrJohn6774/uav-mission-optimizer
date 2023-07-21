### Preliminary sizing results

A hugely simplified optimization problem is formulated to obtain the preliminary weight and wing sizing of a conventional wing and tube design. With the most straight-forward goal in mind - straight and level unaccelerated flight, the optimizer is able to come up with a solution given the following conditions:

- Altitude 100 m
- Wing Span <= 1 m
- C_d0 0.03
- Payload (avionics + mission equipment) mass 0.5 kg
- Propulsion mass 0.3 kg
- Useful-load-mass ratio 0.36
- Throttle 70%
- Elec-Mech efficiency 80%
- Allowable battery discharge 85%
- Maximum wing loading 155 N/m^2

The following results are obtained:
- Flight time: 30.00 minutes
- Total mass: 3.88 kg
- Wing mass: 1.16 kg
- Wing loading: 155.00 N/m^2
- Wing area: 0.25 m^2
- Wing span: 1.00 m
- AR: 4.07
- RE: 220307
- C_L_cruise: 1.00
- TAS_CRUISE: 57.7 km/h
- Alt: 100 m
- PWR: 73.56 W
- Battery config: 2C4P

With an aspect ratio (AR) of 10.82, the optimizer is likely suggesting a glider design. This is, in fact, expected as glider offers the best aerodynamic performance. Especially when we aim for endurance or best glide angle, a glider configuration will often, if not always, gives the best result thanks to its low induced drag generation.

However, observe that the cruise airspeed is actually very close to its stall speed. Since the AR is quite large, the induced drag term doesn't have much influence on the optimizing variables so the optimizer bumps up the C_L as if the drag term is independent from C_L. I have to manually limit the C_L to 1.0 so that a safe margin is maintained from the critical AOA.