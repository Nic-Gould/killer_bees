This project will be a conglomeration of ideas from a few different sources. 

* While working on the MPU driver, I was learning about quaternions and came across [this paper](https://apps.dtic.mil/sti/pdfs/ADA247484.pdf) (USNAVY 1993).

* I've also been learning  some [CADAC](https://arc.aiaa.org/doi/suppl/10.2514/4.102509) modelling for flight vehicle  and missile simulation, and thought it would be cool to try and re-implement it in rust, and in a way that makes sense to me. I typically do these projects both to practice programming and learn more about flight/defence applications, so duplicating existing work doesn't bother me. I learn the same either way. 

* Finally, I came across [this post](https://www.jakobmaier.at/posts/flight-simulation/) recently, which at first I thought might be an implementation of the paper I cited above. It seems to provide a good example of structuring a basic flight dynamics program.

Essentially I'll be referencing ideas from all 3 of these sources to create some sort of unholy mess. 



UPDATE: after comparing the aerodynamic variables used in both the CADAC f16 model and the NAVY model it became clear that the CADAC model shares a high degree of similarity to this NASA paper upon which it is primarily based. However on first inpection it seems that the actual coefficents used in CADAC differ from those in the paper, and it would take additional work to reconcile these differences. 

The NAVY model however, seems to use a simpler set of aerodynamic variables which can easily be obtained from freely available software such as the DIGITAL DATCOM and XFLR5. As such, this project will use the simpler NAVY approach, to facilitate ease of development and greater availability of model data.

I also got distracted and started a re-write of the 50KLOC fortran code for Digital Datcom. While it was nice to learn some FORTRAN, existing tools such as XFLR can provide similar outputs. A comparison of different options has also prompted me to start reviewing the literature WRT estimation/modelling of aerodynamic coefficients to determine the best use case for each scenario (see literature folder). 
