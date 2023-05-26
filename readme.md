This project will be a conglomeration of ideas from a few different sources. 

* While working on the MPU driver, I was learning about quaternions and came across [this paper](https://apps.dtic.mil/sti/pdfs/ADA247484.pdf) (USNAVY 1993).

* I've also been learning  some [CADAC](https://arc.aiaa.org/doi/suppl/10.2514/4.102509) modelling for flight vehicle  and missile simulation, and thought it would be cool to try and re-implement it in rust, and in a way that makes sense to me. I typically do these projects both to practice programming and learn more about flight/defence applications, so duplicating existing work doesn't bother me. I learn the same either way. 

* Finally, I came across [this post](https://www.jakobmaier.at/posts/flight-simulation/) recently, which at first I thought might be an implementation of the paper I cited above. It seems to provide a good example of structuring a basic flight dynamic simulation.

Essentially I'll be referencing ideas from all 3 of these sources to create some sort of unholy mess. 



UPDATE: after comparing the aerodynamic variables used in both the CADAC f16 model and the NAVY model it became clear that the CADAC model shares a high degree of similarity to this NASA paper upon which it is primarily based. However on first inpection it seems that the actual coefficents used in CADAC differ from those in the paper, and it would take additional work to reconcile these differences. The NAVY model however, seems to use a simpler set of aerodynamic variables which can easily be obtained from freely available software such as the DIGITAL DATCOM and XFLR5. Also I broke my hand on monday....

In the absence of suitable DATCOM / XFLR models, Graer & Morelli [NASA](./Literature/nasa%20regression%20model.pdf) published a model of 8 aircraft using statistical regression to determine coeffecients for sensitive paramaters (acheiving R^2 of 0.9961). This approach would be highly suited to using ML/AI to fit curves to published flight data. This approach significantly simplifies the number of calculations at each time step especially with regards to table lookups for updated coefficients each time there is a change in flight conditions. 


NOTE: I also got distracted and started a re-write of the 50KLOC fortran code to modernise Digital Datcom. While it was nice to learn some FORTRAN, existing tools such as XFLR can provide similar outputs, and ultimately I scrapped the project. A comparison of different options for estimating aerodynamic coefficients has also prompted me to start reviewing the literature to determine the best use case for each scenario (see literature folder). I've also started doing some CFD in ANSYS Fluent, and OPENFoam.


NEXT STEPS: Figure out Async, add inputs.