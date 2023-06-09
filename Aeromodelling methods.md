Heirarchy (3)
Options to improve performance can involve simiplificaiton of the flow by making certain assumptions. This results in the following modelling options in order of fidelity: 
* Time dependant navier stokes equations are the most accurate and most computationally expensive, and only have solutions for a specific subset of problems.
* inviscid euler (non-linear)
* full potential equations - FPE (non-linear)
* laplace (linear) or Linear Potential Equations - LPE (lineaer)

Alternatively, other methods of solution or simiplificaiton result in the following modelling approaches, in order of fidelity:
* (DNS - direct numerical simulation,   LES - large eddy simulation,  Reynolds averaged Navier Stokes - RANS,)
* thin layer navier stokes TLNS
* classical boundary layer

Navier Stokes computational tools had the best overall agreement with test data when run laminar for low angles of attack and turbulent for higher angles of attack (1).
The low-order, potential flow solvers, especially LSAero, provided good predictions in the linear region for significantly cheaper cost both computationally and turn-around time.(1) 

It is recommended to combine methods to acheive a complete solution (3).  A combination of full potential equations (FPE) + Boundary layer equations (BLE) offer good accuracy (comparable to RANS) vs computational intensity. (4) Field panel method (fpm) recommended to solve (FPE) due to ease of computation and compatability with BLE. (4)recommends finite element modelling as FPM is unable to accurately model shock. "Lifting line and lifting surface theory, as well as vortex lattice methods, are limited to thin wings. Panel methods, on the other hand, are able to represent thickness." (3)

an analysis of 11 solvers shows XLFR as providing VLM, panel  and LLT/LST methods for modelling inviscid irrotational incompressible flow, combined with XFOIL modelling including boundary layer modelling. another advantage of xfoil is it was able to predict flow transition, where most 2d/hyrbid models failed (3) a comparison of DATCOM and XFLR for a 747 show DATCOM providing higher fidelity results for longitudintal coeffecients, while XFLR better estimated lateral coefficients, when each was compared to published data. (2)

Non linear solution methods (for systems of non-linear equations)
    finite difference
    finiite Element
    finite volume

Sources

    (1) Schafer & Lynch https://ntrs.nasa.gov/api/citations/20140006172/downloads/20140006172.pdf

    (2) Ahmad

    (3) peerlings

    (4) Crovato

NASA 1972 Contractor Report CR2144 Aircraft Handling Qualities Data
NASA 1973 Contractor Report CR2177
NASA 1979 Technical Paper TP1538
NASA - A Generic Nonlinear Aerodynamic Model for AircraftRobert C Nelson 1998 - Flight Stability and Automatic Control

Predictions of basic turbulent flows with Fluent (fluidcodes.com)
turbulence_2021_OF8.pdf (wolfdynamics.com)
ADA086558.pdf (dtic.mil)

References for the Digtal Datcom Program (pdas.com)

ADB072483.pdf

19630012235.pdf (nasa.gov)
 