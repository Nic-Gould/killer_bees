struct Coefficients ();

let coefficients = Coefficients{
	//specifying reference values
	let refa(27.87);					//"Reference area for aero coefficients - m^2"
	let refb(9.14);						//"Reference span for aero coefficients - m"
	let refc(3.45);						//"Reference cord for aero coefficients - m"

	//F16 mass parameters (constant)
	let vmass=9496;						//"Aircraft mass - kg"
	


	//engine angular momentum
	let eng_ang_mom(70000);				//"Engine angular momentum - kg*m^2/sec"

	//Run termination criteria
	//If any of the termination limits is violated a number code is
	//stored in 'trcond'. If mstop=1 the simulation will stop.
	//If mstop=0 (default) the simulation will continue, and additional
	//code numbers, if any, will enter on the left of 'trcond' until the
	//run is stopped by other means.
	//
	//	   code  term.cond. module				description
	//		 2     trmach     environment	minimum mach number
	//		 3     trdynm     environment	minimum dynamic pressure - Pa
	//		 4     trload     aerodynamics	minimum load factor - g's
	//		 5     tralppx    kinematics   maximum pos angle of attack - deg
	//		 6     tralpnx    kinematics   minimum neg angle of attack - deg
	//		 7     trbetx     kinematics   maximum sidelsip angle - deg

	let trmach(0.8);
	let trdynm(10.e+3);
	let trload(3);
	let tralppx(21);
	let tralpnx(-6);
	let trbetx(5);
	//
	let trcode(0);		//"Termination code number"
	let tmcode(0);		//"Dummy variable initialized to zero"

	

///////////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Plane'
//
//This module performs the following functions:
// (1) Aerodynamic table look-up from file 'plane6_aero_deck.asc'
//     Ref area 'refa' = 27.87 m^2,  span 'refb' = 9.14 m, chord 'refc' = 3.45 m
// (2) Calculation of aero coefficients in body coordinates
// Note: Rolling moment from positive aileron deflection is negative
//		in high subsonic regime. This phenomena is caused by wing twist.
//		Therefore, the aircraft uses spoilers in this region. In this sim
//		the sign of 'clda' is reversed since spoiler are not implements
//
//030627 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////


	//localizing module-variables
	//input data
	/* let alplimpx=plane[160].real();
	let alplimnx=plane[164].real();
	let xcg=plane[193].real();
	let xcgr=plane[194].real();
	//from initialization
	let refa=plane[104].real();
    let refb=plane[105].real();
    let refc=plane[106].real();
	let trcode=plane[180].real();
	let trload=plane[185].real();
	let vmass=plane[190].real();;			
	//input from other modules
	let time=flat6[0].real();
	let alphax=flat6[144].real();
	let betax=flat6[145].real();
	let vmach=flat6[56].real();
	let pdynmc=flat6[57].real();
	let dvba=flat6[75].real();
	let ppx=flat6[160].real();
	let qqx=flat6[161].real();
	let rrx=flat6[162].real();
	let delax=plane[619].real();
	let delex=plane[620].real();
	let delrx=plane[621].real(); */
	//-------------------------------------------------------------------------
	//common parameters
	let c2v = refc/(2*dvba);
	let b2v =r efb/(2*dvba);

	//axial force coefficient
	let cx=aerotable.look_up("cx_vs_elev_alpha",delex,alphax);
	let cxq=aerotable.look_up("cxq_vs_alpha",alphax);
	cxt=cx+c2v*cxq*qqx*RAD;		//"X-force coefficient - ND"

	//side force coefficient
	let cyr=aerotable.look_up("cyr_vs_alpha",alphax);

	let cyp=aerotable.look_up("cyp_vs_alpha",alphax);
	cyt=-0.02*betax+0.021*delax/20+0.086*delrx/30+b2v*(cyr*rrx*RAD+cyp*ppx*RAD);

	//down force coefficient
	let cz=aerotable.look_up("cz_vs_alpha",alphax);
	let czq=aerotable.look_up("czq_vs_alpha",alphax);
	czt=cz*(1-pow(betax*RAD,2))-0.19*delex/25+c2v*czq*qqx*RAD;

	//rolling moment coefficient
	let cl=aerotable.look_up("cl_vs_beta_alpha",betax,alphax); 						//"Lift coefficient - ND"
	let cldr=aerotable.look_up("cldr_vs_beta_alpha",betax,alphax);
	let clda=aerotable.look_up("clda_vs_beta_alpha",betax,alphax);
	clda=-clda; //see note in header!
	let clr=aerotable.look_up("clr_vs_alpha",alphax);
	let clp=aerotable.look_up("clp_vs_alpha",alphax);
	clt=cl+clda*delax/20+cldr*delrx/30+b2v*(cllr*rrx*RAD+clp*ppx*RAD);					//"Rolling moment coefficient - ND"

	//pitching moment coefficient
	let cm=aerotable.look_up("cm_vs_elev_alpha",delex,alphax);
	let cmq=aerotable.look_up("cmq_vs_alpha",alphax);								//"Pitch dampning deriv(alpha,mach) - 1/rad"
	cmt=cm+c2v*cmq*qqx*RAD+czt*(xcgr-xcg)/refc;											//"Pitching moment coefficient - ND"

	//yawing moment coefficient
	let cn=aerotable.look_up("cn_vs_beta_alpha",betax,alphax);
	let cnda=aerotable.look_up("cnda_vs_beta_alpha",betax,alphax);
	let cndr=aerotable.look_up("cndr_vs_beta_alpha",betax,alphax);
	let cnr=aerotable.look_up("cnr_vs_alpha",alphax);
	let cnp=aerotable.look_up("cnp_vs_alpha",alphax);
	cnt=cn+cnda*delax/20+cndr*delrx/30-cyt*(xcgr-xcg)/refb								//"Yawing moment coefficient - ND"
		+b2v*(cnr*rrx*RAD+cnp*ppx*RAD);

	//calculating the positive and negative load factors available
	let czp=aerotable.look_up("cz_vs_alpha",alplimpx);
	let czn=aerotable.look_up("cz_vs_alpha",alplimnx);
	let alpx=-czp*pdynmc*refa;
	let alnx=-czn*pdynmc*refa;
	let weight=vmass*AGRAV;
	gmax=alpx/weight;																	//"Max maneuverability limited by strct_pos_limitx- g's"
	gminx=alnx/weight;																	//"Min maneuverability limited by strct_neg_limitx - g's"

	//diagnostic: lift and drag coefficients
	let cosa=cos(alphax*RAD);
	let sina=sin(alphax*RAD);
	cdrag=-cxt*cosa-czt*sina;			//"Drag coefficient - ND",
	clift=cxt*sina-czt*cosa;			//"Lift coefficient - ND",
	clovercd=clift/cdrag;				//"Lift over drag ratio - ND"

	//preparing deriv (all moments start with'clxxx'to distinguish from lift'cl')
	let clde=0.19/25;	//per deg		"Lift force due to elevator (alpha.mach), - 1/deg"
	let cyb=-0.02*RAD;	//per deg		"Weather vane der wrt beta(alpha,mach) - 1/deg"
	let cydr=-0.086/30; //per deg		"Side force due to rudder deriv(alpha,mach) - 1/deg"
	let cmq=cmq;		//per rad/s
	let clnr=b2v*cnr;	//per rad/s		"Yaw damping deriv(alpha,mach) - 1/rad"
	let clndr=cndr/30;	//per deg		"Yaw moment due to rudder deriv(alpha,mach) - 1/deg",
	let cllp=b2v*clp;	//per rad/s		"Roll damping deriv(alpha,mach) - 1/rad"
	let cllda=clda/20;	//per deg		"Roll control effectiveness(alpha,mach), - 1/deg"

	//run-termination if max g capability is less than 'trload'
	if(gmax<trload)trcode=4;
	//-------------------------------------------------------------------------

	//-------------------------------------------------------------------------	
	//function call for derivative calculations

///////////////////////////////////////////////////////////////////////////////
//On-line calculation of aerodynamic derivatives for aero-adaptive autopilot 
//Member function of class 'Plane'
//Input/output occurs over module-variables
//
//This subroutine performs the following functions:
// (1) calculates dimensional derivaties (rad, m, sec)
// (2) calculates the airframe's rigid modes
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

/* 
	//localizing module-variables
	//from initialization
	let refa=plane[104].real();
    let refb=plane[105].real();
    let refc=plane[106].real();
	let xcg=plane[193].real();
	let xcgr=plane[194].real();
	//from other modules
	let time=flat6[0].real();
	let vmach=flat6[56].real();
	let pdynmc=flat6[57].real();
	let dvba=flat6[75].real();
	let alphax=flat6[144].real();
	let betax=flat6[145].real();
    let vmass= plane[190].real();
	Matrix IBBB=plane[191].mat(); 
	let delex=plane[620].real();
	//from aerodynamics module		
	let cla=plane[123].real(); //cla=-cza interpolate from cz	//"Lift slope derivative(alpha,mach) - 1/deg"
	let clde=plane[124].real();//clde=0.19
	let cyb=plane[125].real();//const = -0.02*RAD (deg)
	let cydr=plane[127].real();//const = 0.086 (max value for delrx=30deg)
	let cllda=plane[129].real();//available from look-up
	let cllp=plane[130].real();//available from look-up
	let cmde=plane[134].real();//interpolate from cm				//"Pitch control effectiveness(alpha,mach), - 1/deg"
	let cmq=plane[135].real();//available from look-up
	let clndr=plane[138].real();//available from look-up
	let clnr=plane[140].real();//available from look-up */
	//-------------------------------------------------------------------------
	//interpolating to get remaining nondimensional derivatives
	//lift slope derivative	 (per degree)
	let czp=aerotable.look_up("cz_vs_alpha",alphax+1.5);
	let czn=aerotable.look_up("cz_vs_alpha",alphax-1.5);
	let cza=(czp-czn)/3;
	cla=-cza;  //"Lift slope derivative(alpha,mach) - 1/deg"

	//pitching moment due to alpha derivative (per degree)
	let cmp=aerotable.look_up("cm_vs_elev_alpha",delex,alphax+1.5);
	let cmn=aerotable.look_up("cm_vs_elev_alpha",delex,alphax-1.5);
	let dum=(cmp-cmn)/3;
	let cma=dum+cza*(xcgr-xcg)/refc;		//"Pitch moment due to alpha deriv(alpha,mach) -1/deg"

	//elevator control derivative (per degree)
	cmp=aerotable.look_up("cm_vs_elev_alpha",delex+1.5,alphax);
	cmn=aerotable.look_up("cm_vs_elev_alpha",delex-1.5,alphax);
	cmde=(cmp-cmn)/3;

	//yawing moment due to beta derivative (per degree)
	let cnp=aerotable.look_up("cn_vs_beta_alpha",betax+1.5,alphax);
	let cnn=aerotable.look_up("cn_vs_beta_alpha",betax-1.5,alphax);
	dum=(cnp-cnn)/3;
	clnb=dum-cyb*(xcgr-xcg)/refb;			//"Yaw moment deriv(alpha,mach) - 1/deg"
	
	//dimensional derivatives
	//MOI components
	let ibbb11=IBBB.get_loc(0,0);
	let ibbb22=IBBB.get_loc(1,1);
	let ibbb33=IBBB.get_loc(2,2);

	//Dimensional derivatives for pitch plane (converted to 1/rad where required)
	let duml=(pdynmc*refa/vmass)/RAD;
	dla=duml*cla;							//"Lift slope derivative - m/s^2"
	dlde=duml*clde;							//"Lift elevator control derivative - m/s^2"
	let dumm=pdynmc*refa*refc/ibbb22;
    dma=dumm*cma/RAD;						//"Pitch moment derivative - 1/s^2"
    dmq=dumm*(refc/(2.*dvba))*cmq;			//"Pitch damping derivative - 1/s"
    dmde=dumm*cmde/RAD;						//"Pitch control derivative - 1/s^2"

	//Dimensional derivatives in lateral plane (converted to 1/rad where required)
    let dumy=pdynmc*refa/vmass;
    dyb=dumy*cyb/RAD;						//"Side force derivative - m/s^2"
	dydr=dumy*cydr/RAD;						//"Side force control derivative - m/s^2"
    let dumn=pdynmc*refa*refb/ibbb33;
    dnb=dumn*clnb/RAD;						//"Yawing moment derivative - 1/s^2"
    dnr=dumn*(refb/(2.*dvba))*clnr;			//"Yaw dampnig derivative - 1/s"
    dndr=dumn*clndr/RAD;					//"Yaw control derivative - 1/s^2"

	//Dimensional derivatives in roll (converted to 1/rad where required)
	let dumll=pdynmc*refa*refb/ibbb11;
    dllp=dumll*(refb/(2.*dvba))*cllp;		//"Roll damping derivative - 1/s",
    dllda=dumll*cllda/RAD;					//"Roll control derivative - 1/s^2"

	//static margin (per chord length 'refc')
	if(cla) stmarg=-cma/cla;		//"Static margin (+stable, -unstable) - caliber"

	//diagnostics: pitch plane roots		
	let a11=dmq;
	let a12=dma/dla;
	let a21=dla;
	let a22=-dla/dvba;
	
	let arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
	   wnp=0.;												//"Natural frequency of airframe pitch dynamics - rad/s"
	   zetp=0.;												//"Damping of airframe pitch dynamics - NA"
	   let dum=a11+a22;
	   realp1=(dum+sqrt(arg))/2.;							//"First real root of airframe pitch dyn  - rad/s"
	   realp2=(dum-sqrt(arg))/2.;							//"Second real root of airframe pitch dyn - rad/s"
	   rpreal=(realp1+realp2)/2.;							//"Real part or mean value (real roots) of pitch  - rad/s"
	}
	else
	{
	   realp1=0.;
	   realp2=0.;
	   wnp=sqrt(a11*a22-a12*a21);
	   zetp=-(a11+a22)/(2.*wnp);
	   rpreal=-zetp*wnp;
	}
	
	//diagnostics: yaw plane roots		
	a11=dnr;
	a12=dnb/dyb;
	a21=-dyb;
	a22=dyb/dvba;
	
	arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
	   wny=0.;											//"Natural frequency of airframe yaw dynamics - rad/s"
	   zety=0.;											//"Damping of airframe yaw dynamics - NA"
	   let dum=a11+a22;
	   realy1=(dum+sqrt(arg))/2.;						//"First real root of airframe yaw dynamics - rad/s"
	   realy2=(dum-sqrt(arg))/2.;						//"Second real root of airframe yaw dynamics - rad/s"
	   ryreal=(realy1+realy2)/2.;						//"Real part or mean value (real roots) of yaw - rad/s"
	}
	else
	{
	   realy1=0.;
	   realy2=0.;
	   wny=sqrt(a11*a22-a12*a21);
	   zety=-(a11+a22)/(2.*wny);
	   ryreal=-zety*wny;
	}
	//-------------------------------------------------------------------------
//initial conditions
let sbel1	=	0;			//Initial north comp of SBEL - m module newton
let	sbel2	=	0;			//Initial east comp of SBEL - m module newton
let sbel3	=	-1000;		//Initial down comp of SBEL - m module newton
let alpha0x	=	1;			//Initial angle-of-attack - deg module newton
let beta0x	=	0;			//Initial side slip angle - deg module newton
let	thtblx	=	1;			//Pitching angle of vehicle - deg module kinematics
let dvbe	=	180;		//Plane speed - m/s module newton

//aerodynamics
let	alplimnx	=	-6;		//Minimum neg alpha permissible (with neg sign) - deg module aerodynamics
let alplimpx	=	16;		//Maximum positive alpha permissible - deg module aerodynamics
let xcg			=	1;		//Actual c.g location - m module aerodynamics
let xcgr		=	1;		//Reference c.g location - m module aerodynamics


//propulsion
let mprop		=	2;		//'int' =0: Motor off, =1:Motor on module propulsion
let vmachcom	=	0.6;	//Commanded Mach # - ND module propulsion
let gmach		=	30;		//Gain conversion from Mach to throttle - ND module propulsion

//actuator
let mact	=	2;			//'int' =0:no dynamics, =2:second order module actuator	
let dlimx	=	20;			//Control fin limiter - deg module actuator
let ddlimx	=	400;		//Control fin rate limiter - deg/s module actuator	
let wnact	=	50;			//Natural frequency of actuator - rad/s module actuator
let zetact	=	0.7;		//Damping of actuator - ND module actuator

//autopilot
let maut	=	45;			//'int' maut=|mauty|mautp| see 'control' module module control
let maut	=	33;			//'int' maut=|mauty|mautp| see 'control' module module control
let dalimx	=	20;			//Aileron limiter - deg module control
let delimx	=	20;			//Elevator limiter - deg module control
let drlimx	=	20;			//Rudder limiter - deg module control
let anlimnx	=	6;			//Neg structural accel limiter (data is positive) - g's module control
let anlimpx	=	9;			//Positive structural acceleration limiter - g's module control

//roll controller
let phicomx	=	0;			//Roll angle command - deg module control
let philimx	=	70;			//Roll angle limiter - deg module control
let wrcl	=	15;			//Freq of roll closed loop complex pole - rad/s module control
let zrcl	=	0.7;		//Damping of roll closed loop pole - ND module control

//SAS
let zetlagr	=	0.7;		//Desired damping of closed rate loop ND module control

//heading controller
let psivlcomx	=	0;		//Heading command - deg module control
let facthead	=	-0.9;	//Fact to reduce heading gain gainpsi*(1.+facthead) - ND module control

//pitch acceleration controller
let ancomx	=	1;			//Pitch acceleration command - g's module control
let gainp	=	0;			//Proportional gain in pitch acceleration loop - s^2/m module control
let paclp	=	10;			//Close loop real pole - ND module control
let waclp	=	4;			//Nat freq of accel close loop complex pole - rad/s module control
let zaclp	=	0.3;		//Damping of accel close loop complex pole - ND module control


//altitude hold
let gainalt		=	0.3;	//Altitude gain - 1/s module control
let gainaltrate	=	0.7;	//Altitude rate gain - 1/s module control

let altcom		=	1000;		//Altitude command - m module control
if time > 2 
{altcom			=	1100};	

if time > 12 	
{altcom			=	1000};	









//lateral acceleration controller

let alcomx	=	0;			//Lateral (horizontal) acceleration comand - g's module control
let gainl	=	1;			//Gain in lateral acceleration loop - rad/g's module control

//pitch line guidance
let line_gain 		=	1;		//Line guidance gain - 1/s module guidance
let mguid			=	3;		//'int' Switch for guidance options - ND module guidance
let nl_gain_fact	=	0.4;	//Nonlinear gain factor - ND module guidance
let decrement		=	500;	//distance decrement - m module guidance

////////IP
let swel1	=	5000;	//North coordiante of way point - m module guidance
let swel2	=	0;		//East coordinate of way point - m module guidance
let swel3	=	-200;	//Altitude of way point - m module guidance
let thtflx	=	-30;	//Pitch line-of-attack angle - deg module guidance

//table look-up coefficients
/* 	//plane[109].init("cd",0,"Drag coefficient - ND","aerodynamics","diag",""); // I think this is defined as 'cdrag' in this file, so this may be unused?
	plane[120].init("cd0",0,"Reference drag coeff(alpha,mach) - ND","aerodynamics","diag","");
	plane[121].init("cda",0,"Delta drag force due to alpha(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[122].init("cl0",0,"Reference lift coeff(alpha,mach) - ND","aerodynamics","diag","");
	plane[126].init("cyda",0,"Side force due to aileron deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[132].init("cm0",0,"Pitch moment coeff(alpha,mach) - ND","aerodynamics","diag","");
	plane[137].init("clnda",0,"Yaw moment due to aileron deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[139].init("clnp",0,"Yaw moment due to roll rate deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
    plane[141].init("clldr",0,"Roll moment due to rudder - 1/deg","aerodynamics","diag","");
	plane[128].init("cllb",0,"Dutch-roll deriv(alpha,mach) - 1/deg","aerodynamics","diag",""); 
	plane[131].init("cllr",0,"Roll moment due to yaw rate deriv(alpha,mach) - 1/rad","aerodynamics","diag","");*/
	
}	
	
    
 	
