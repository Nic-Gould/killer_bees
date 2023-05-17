This project will be a conglomeration of ideas from a few different sources. 

While working on the MPU driver, I was learning about quaternions and came across [this paper](https://apps.dtic.mil/sti/pdfs/ADA247484.pdf) (USNAVY 1993).

I've also been learning  some [CADAC](https://arc.aiaa.org/doi/suppl/10.2514/4.102509) modelling for flight vehicle  and missile simulation, and thought it would be cool to try and re-implement it in rust, and in a way that makes sense to me. I typically do these projects both to practice programming and learn more about flight/defence applications, so duplicating existing work doesn't bother me. I learn the same either way. 

Finally, I came across [this post](https://www.jakobmaier.at/posts/flight-simulation/) recently, which at first I thought might be an implementation of the paper I cited above. It seems to provide a good example of structuring a basic flight dynamics program.

Essentially I'll be referencing ideas from all 3 of these sources to create some sort of unholy mess. 


Also, I know that no-one is reading this except me, but I PROMISE this will be the last new project, until I start finishing some. Starting projects is, for me, the equivelent of writing ideas down. It's a good way to park a thought so you can come back to it later. 


CADAC
doc.asc is a handy reference for the variable names.

xxx_aero_deck.asc   -   aerodynamic response data
XXX_prop_deck.asc   -   propultion system data.
actuator            -   defines different actuator structs and  associated funtions for each model. could be good to re-work this a bit. 
                        maybe start with a straight re-write. I always get distracted by stuff like this.
control             -   autopilot functions? won't be implemented at this time.
aerodynamics        -   checks if flight conditions have exceeded airframe limits.
f16c09_7.asc and f16c11_2.asc contain 90% the exact same data/parameters with minor differences. These seem to load different parameters depending on what's being modelled.
forces              - same/similar approch to other methods
euler               - conversion functions?
kinematics          - update quaternions and angles.



utility functions is just a matrix library, and air density calculator. Uses Bsearch instead of if statements and array. Might be worth it.
class_function - literally just functions for building the classes and allocating memory.
class_heirarchy- also param/method definitions.
plane_functions - more io, mostly output this time.
