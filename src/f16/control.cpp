
	//local module-variables
	double delacx(0),delecx(0),delrcx(0);
	double phi(0);
	
	
	//-------------------------------------------------------------------------
	//return if no control
	if(maut==0) return;

	//decoding control flag
    int mauty=maut/10;
    int mautp=maut%10; 

	//calling yaw and pitch rate stabilizers, assuming zero rate input
	if(mauty==2) 
		 delrcx=control_yaw_rate(rcomx);
	if(mautp==2) 
		 delecx=control_pitch_rate(qcomx);

	//calling acceleration controller in normal- and lateral-plane
	if(mauty==3)
		phicomx=control_lateral_accel(alcomx);

	if(mautp==3){
		//limiting normal acceleration by max alpha or max structure load
		if(ancomx>gmax) ancomx=gmax;
		if(ancomx<gminx) ancomx=gminx;
		delecx=control_normal_accel(ancomx,int_step);
	}

	//calling vertical flight path angle controller
	if(mautp==4)
		delecx=control_gamma(thtvlcomx);
	//calling heading and SAS(rcomx=0) controllers
	if(mauty==4){
		phicomx=control_heading(psivlcomx);
		delrcx=control_yaw_rate(rcomx);
	}

	//calling altitude control
	if(mautp==5){
		ancomx=control_altitude(altcom);
		//limiting normal acceleration by pos and neg structural limiter
		if(ancomx>anlimpx) ancomx=anlimpx;
		if(ancomx<-anlimnx) ancomx=-anlimnx;

		//limiting normal acceleration by max alpha or max structure load
		if(ancomx>gmax)
			ancomx=gmax;
		if(ancomx<gminx)
			ancomx=gminx;
		delecx=control_normal_accel(ancomx,int_step);
	}

	//selecting roll-postion or roll-rate control
	if(mroll==0){
		if(fabs(phicomx)>philimx) phicomx=philimx*sign(phicomx);
		delacx=control_roll(phicomx);
	}
	else if(mroll==1)
		delacx=control_roll_rate(pcomx);

	//limiting control commands
	if(fabs(delacx)>dalimx)delacx=dalimx*sign(delacx);
	if(fabs(delecx)>delimx)delecx=delimx*sign(delecx);
	if(fabs(delrcx)>drlimx)delrcx=drlimx*sign(delrcx);
	//-------------------------------------------------------------------------
	//ouput to other modules

}	
///////////////////////////////////////////////////////////////////////////////
//Roll controller
//Member function of class 'Plane'
//Pole placement technique, specifying closed loop conj complex poles
// (1) Calculates gains
// (2) Calculates commanded roll control
//
//Return output
//		delacx = roll control command - deg
//Parameter input
//		phicomx = roll command - deg
//
//011212 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_roll(double phicomx)
{
	//local module-variables
	double gkp(0);
	double gkphi(0);

	//-------------------------------------------------------------------------
	//calculating gains
	gkp=(2.*zrcl*wrcl+dllp)/dllda;
    gkphi=wrcl*wrcl/dllda;

	//roll position control
    double ephi=gkphi*(phicomx-phiblx)*RAD;
    double dpc=ephi-gkp*ppx*RAD;
    double delacx=dpc*DEG;
	//-------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
//Roll rate controller 
//Member function of class 'Plane'
// (1) Calculates roll rate gyro feedback gain
// (2) Calculates commanded roll control based on roll rate input
//Return ouput
//		delacx = roll control command - deg
//Parameter input
//		pcomx = roll rate command - deg/s
//
//021203 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_roll_rate(double pcomx)
{

	//-------------------------------------------------------------------------
	//roll rate gain to achieve closed loop time constant tp
	double kp=(1/tp+dllp)/dllda;

	//commanded roll control fin
	double delacx=kp*(pcomx-ppx);
	//-------------------------------------------------------------------------
	return delacx;
}
///////////////////////////////////////////////////////////////////////////////
//SAS controller for pitch rate 
//Member function of class 'Plane'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded pitch control based on zero input
//
//Return ouput
//		delecx = pitch control command - deg
//Input parameter
//		qcomx = pitch rate command - deg/s
//
//021016 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_pitch_rate(double qcomx)
{
	//localizing module-variables
	//input data
	double zetlagr=plane[549].real();
	//input from other modules
	double dla=plane[145].real();
	double dlde=plane[146].real();
	double dma=plane[147].real();
	double dmq=plane[148].real();
	double dmde=plane[149].real();
	double qqx=flat6[161].real();
	double dvbe=flat6[236].real();
	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    double zrate=dla/dvbe-dma*dlde/(dvbe*dmde);
    double aa=dla/dvbe-dmq;
    double bb=-dma-dmq*dla/dvbe;

	//feecback gain of rate loop, given desired close loop 'zetlager'
    double dum1=(aa-2.*zetlagr*zetlagr*zrate);
    double dum2=aa*aa-4.*zetlagr*zetlagr*bb;
    double radix=dum1*dum1-dum2;
    if(radix<0.)radix=0.;
	if(fabs(dmde)<SMALL)dmde=SMALL*sign(dmde);
    double grate=(-dum1+sqrt(radix))/(-dmde);

	//commanded pitch control fin
	double delecx=grate*(qqx-qcomx);
	//-------------------------------------------------------------------------

	return delecx;
}
///////////////////////////////////////////////////////////////////////////////
//SAS controller for yaw rate 
//Member function of class 'Plane'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded yaw control based on zero input
//
//Return ouput
//		delrcx = yaw control command - deg
//Input parameter
//		rcomx = yaw rate command - deg/s
//
//021016 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_yaw_rate(double rcomx)
{
	//local module-variables
	double zrate(0);
	double grate(0);
	double wnlagr(0);


	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    zrate=-dyb/dvbe+dnb*dydr/(dvbe*dndr);
    double aa=-dyb/dvbe-dnr;
    double bb=dnb+dyb*dnr/dvbe;

	//feecback gain of rate loop, given desired close loop 'zetlager'
    double dum1=(aa-2.*zetlagr*zetlagr*zrate);
    double dum2=aa*aa-4.*zetlagr*zetlagr*bb;
    double radix=dum1*dum1-dum2;
    if(radix<0.)radix=0.;
	if(fabs(dndr)<SMALL) dndr=SMALL*sign(dndr);
    grate=(-dum1+sqrt(radix))/(-dndr);

    //natural frequency of closed rate loop
    double dum3=grate*dndr*zrate;
    radix=bb+dum3;
    if(radix<0.)radix=0.;
    wnlagr=sqrt(radix);

	//commanded yaw control fin
	double delrcx=grate*(rrx-rcomx);
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[560].gets(zrate);
	plane[561].gets(grate);
	plane[562].gets(wnlagr);

	return delrcx;
}
///////////////////////////////////////////////////////////////////////////////
//Acceleration controller in normal (pitch) plane
//Member function of class 'Plane'
//Employs pole placement technique (no matrix inversion required) 
//Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded pitch control deflection
//
//Return output
//		delecx = pitch control command - deg
//Parameter input
//		ancomx = normal loadfactor command - g
//		int_step = integration step size - s
//		
//021015 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_normal_accel(double ancomx,double int_step)
{
	//local module-variables
	Matrix GAINFP(3,1);

	//localizing module-variables
	//input data
	double waclp=plane[504].real();
	double zaclp=plane[505].real();
	double paclp=plane[506].real();
	double gainp=plane[525].real();
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double dla=plane[145].real();
	double dma=plane[147].real();
	double dmq=plane[148].real();
	double dmde=plane[149].real();
	double qqx=flat6[161].real();
	Matrix FSPB=flat6[230].vec();
	double dvbe=flat6[236].real();
	//state variables
	double zzd=plane[516].real();
	double zz=plane[517].real();
	//-------------------------------------------------------------------------
	//10km, 0.55M q=5500 Pa: waclp=2, zaclp=0.7,paclp=5
	//1km, 0.6M q=21800 Pa: waclp=4, zaclp=0.3,paclp=10
	waclp=2+0.0001226*(pdynmc-5500);
	zaclp=0.7-0.0000245*(pdynmc-5500);
	paclp=5+0.0003067*(pdynmc-5500);
	
	//gain calculation
	double gainfb3=waclp*waclp*paclp/(dla*dmde);
	double gainfb2=(2.*zaclp*waclp+paclp+dmq-dla/dvbe)/dmde;
	double gainfb1=(waclp*waclp+2.*zaclp*waclp*paclp+dma+dmq*dla/dvbe
				-gainfb2*dmde*dla/dvbe)/(dla*dmde)-gainp;

	//pitch loop acceleration control, pitch control command
	double fspb3=FSPB[2];
    double zzd_new=AGRAV*ancomx+fspb3;
	zz=integrate(zzd_new,zzd,zz,int_step);
	zzd=zzd_new;
	double dqc=-gainfb1*(-fspb3)-gainfb2*qqx*RAD+gainfb3*zz+gainp*zzd;
    double delecx=dqc*DEG;

	//diagnostic output
	GAINFP.build_vec3(gainfb1,gainfb2,gainfb3);
	//--------------------------------------------------------------------------
	//loading module-variables
	//state variables
	plane[516].gets(zzd);
	plane[517].gets(zz);
	//diagnostics
	plane[524].gets_vec(GAINFP);

	return delecx;
}

///////////////////////////////////////////////////////////////////////////////
//Converting lateral acceleration command into roll angle command
//Member function of class 'Plane'
//
//Return output
//		phicomx = roll angle command - deg
//Parameter input
//		alcomx = lateral (horizontal) loadfactor command - g's
//		
//030620 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_lateral_accel(double alcomx)
{
	//local variable
	double phicomx(0);

	//localizing module-variables
	//input data
	double gainl=plane[526].real();
	//input from other modules
	double grav=flat6[55].real();
	Matrix FSPB=flat6[230].vec();
	//-------------------------------------------------------------------------
	double fspb3=FSPB[2];

	//converting lateral acceleration into roll angle
	phicomx=-DEG*gainl*atan(alcomx)*sign(fspb3);
	return phicomx;
}
///////////////////////////////////////////////////////////////////////////////
//Flight path angle tracking (gamma-hold) control (mautp=4)
//employs pole placement technique
//feedback signals are: body rate (gyro), body pitch angle 
// and flight path angle 
//
//(1) Calculates three feedback and one feed-foreward gain
//    based on input of closed loop conjugate complex pair
//    and one real pole ('wgam', 'zgam', 'pgam').
//(2) Calculates the commanded pitch control deflections
//
//Return output:
//         delecx=Pitch flap command deflection - deg
//Parameter input:
//         thtvlcomx=Flight path angle command - deg
//
//020614 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double Plane::control_gamma(double thtvlcomx)
{
	//local variables
	Matrix AA(3,3);
	Matrix BB(3,1);
	Matrix DP(3,3);
	Matrix DD(3,1);
	Matrix HH(3,1);

	//local module-variables
	Matrix GAINGAM(3,1);
	double gainff=0;

	//localizing module-variables
	//input data
	double pgam=plane[563].real();
	double wgam=plane[564].real();
	double zgam=plane[565].real();
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double thtblx=flat6[138].real();
	double qqx=flat6[161].real();
	double thtvlx=flat6[241].real();
	double dvbe=flat6[236].real();
	double dla=plane[145].real();
	double dlde=plane[146].real();
	double dma=plane[147].real();
	double dmq=plane[148].real();
	double dmde=plane[149].real(); 
	//--------------------------------------------------------------------------

	//prevent division by zero
	if(dvbe==0)dvbe=dvbe;
	
	//building fundamental matrices (body rate, acceleration, fin deflection)
	AA.build_mat33(dmq,dma,-dma,1.,0.,0.,0.,dla/dvbe,-dla/dvbe);
	BB.build_vec3(dmde,0.,dlde/dvbe);

	//feedback gains from closed-loop pole placement
	double am=2.*zgam*wgam+pgam;
	double bm=wgam*wgam+2.*zgam*wgam*pgam;
	double cm=wgam*wgam*pgam;
	double v11=dmde;
	double v12=0.;
	double v13=dlde/dvbe;
	double v21=dmde*dla/dvbe-dlde*dma/dvbe;
	double v22=dmde;
	double v23=-dmq*dlde/dvbe;
	double v31=0.;
	double v32=v21;
	double v33=v21;
	DP.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33);
	DD.build_vec3(am+dmq-dla/dvbe,bm+dma+dmq*dla/dvbe,cm);
	Matrix DPI=DP.inverse();
	GAINGAM=DPI*DD;

	//steady-state feed-forward gain to achieve unit gamma response
	Matrix DUM33=AA-BB*~GAINGAM;
	Matrix IDUM33=DUM33.inverse();
	Matrix DUM3=IDUM33*BB;
	HH.build_vec3(0.,0.,1.);
	double denom=HH^DUM3;
	gainff=-1./denom;

	//pitch control command
	double thtc=gainff*thtvlcomx*RAD;
	double qqf=GAINGAM[0]*qqx*RAD;
	double thtblf=GAINGAM[1]*thtblx*RAD;
	double thtvlf=GAINGAM[2]*thtvlx*RAD;
	double delec=thtc-(qqf+thtblf+thtvlf);
	double delecx=delec*DEG;

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[566].gets_vec(GAINGAM);
	plane[567].gets(gainff);

	return delecx;
}

///////////////////////////////////////////////////////////////////////////////
// Heading hold control (mauty=4)
// Generates bank command for roll control
// Employs pole placement technique with the condition that the
//  real pole of the closed loop heading transfer function equals
//  the product of natural frequency and damping of the roll transfer function
//
// This subroutine performs the following functions:
// (1) Calculates the heading loop forward gain.
// (2) Outputs roll position command
//
// Return output:
//          phicomx=Roll position command - deg
// Parameter input:
//			psivdocx=Heading command - deg
//
//030522 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double Plane::control_heading(double psivlcomx)
{
	//local variable
	double phicomx(0);
	
	//local module-variables
	double gainpsi(0);

	//localizing module-variables
	//input data
	double wrcl=plane[512].real();
	double zrcl=plane[513].real();
	double facthead=plane[551].real();
	//input from other modules
	double grav=flat6[55].real();
	double dvbe=flat6[236].real();
	double psivlx=flat6[240].real();
	//--------------------------------------------------------------------------

	//calculating heading gain
	gainpsi=(dvbe/grav)*zrcl*wrcl*(1.-zrcl*zrcl)*(1.+facthead);

	//roll command
	phicomx=gainpsi*(psivlcomx-psivlx);

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[552].gets(gainpsi);

	return phicomx;
}
///////////////////////////////////////////////////////////////////////////////
// Altitude hold control (mautp=5)
// Generates accleration command for pitch acceleration autopilot,
//  correcting for bank angle
// MAUTP=5
//
// This subroutine performs the following functions:
// (1) Calculates the pitch acceleration command
//
// Return output
//          ancomx=Pich acceleration command - g's
// Parameter input
//			altcom=Altitude command - m
//
//030522 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double Plane::control_altitude(double altcom)
{
	//local variable
	double ancomx(0);
	
	//local module-variables
	double altrate(0);

	//localizing module-variables
	//input data
	double gainalt=plane[528].real();
	double gainaltrate=plane[529].real();
	//input from other modules
	double grav=flat6[55].real();
	double phiblx=flat6[139].real();
	Matrix VBEL=flat6[233].vec();
	double hbe=flat6[239].real();
	//--------------------------------------------------------------------------
	//altitude rate feedback
	altrate=-VBEL[2];

	//acceleration command
	double eh=gainalt*(altcom-hbe);
	if(phiblx==0) phiblx=SMALL;
	ancomx=(1./cos(phiblx*RAD))*(gainaltrate*(eh-altrate)+grav)/AGRAV;

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[530].gets(altrate);

	return ancomx;
}





