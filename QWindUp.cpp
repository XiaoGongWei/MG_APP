#include "QWindUp.h"


QWindUp::QWindUp(void)
{
}


QWindUp::~QWindUp(void)
{
}

//Calculate satellite antenna phase winding
double QWindUp::getWindUp(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *StaPos,double *RecPos,double &phw,double *psunpos)
{
	gtime_t obsGPST;
    double ep[6] = {(double)Year, (double)Month, (double)Day, (double)Hours, (double)Minuts, (double)Seconds};
	obsGPST = m_qCmpClass.epoch2time(ep);
	windupcorr(obsGPST,StaPos,RecPos,&phw,psunpos);
	return phw;
}


/* phase windup correction -----------------------------------------------------
* phase windup correction (ref [7] 5.1.2)
* args   : gtime_t time     I   time (GPST)
*          double  *rs      I   satellite position (ecef) {x,y,z} (m)
*          double  *rr      I   receiver  position (ecef) {x,y,z} (m)
*          double  *phw     IO  phase windup correction (cycle)
* return : none
* notes  : the previous value of phase windup correction should be set to *phw
*          as an input. the function assumes windup correction has no jump more
*          than 0.5 cycle.
*-----------------------------------------------------------------------------*/
void QWindUp::windupcorr(gtime_t time, const double *rs, const double *rr,
	double *phw,double *psunpos)
{
	double ek[3],exs[3],eys[3],ezs[3],ess[3],exr[3],eyr[3],eks[3],ekr[3],E[9];
	double dr[3],ds[3],drs[3],r[3],pos[3],rsun[3],cosp,ph,erpv[5]={0};
	int i;

	//trace(4,"windupcorr: time=%s\n",time_str(time,0));

	/* sun position in ecef */
	if (psunpos)
	{
		rsun[0] = psunpos[0];rsun[1] = psunpos[1];rsun[2] = psunpos[2];
	}
	else
		m_qCmpClass.sunmoonpos(m_qCmpClass.gpst2utc(time),erpv,rsun,NULL,NULL);
	/* unit vector satellite to receiver */
	for (i=0;i<3;i++) r[i]=rr[i]-rs[i];
	if (!m_qCmpClass.normv3(r,ek)) return;

	/* unit vectors of satellite antenna */
	for (i=0;i<3;i++) r[i]=-rs[i];
	if (!m_qCmpClass.normv3(r,ezs)) return;
	for (i=0;i<3;i++) r[i]=rsun[i]-rs[i];
	if (!m_qCmpClass.normv3(r,ess)) return;
	m_qCmpClass.cross3(ezs,ess,r);
	if (!m_qCmpClass.normv3(r,eys)) return;
	m_qCmpClass.cross3(eys,ezs,exs);

	/* unit vectors of receiver antenna */
	m_qCmpClass.ecef2pos(rr,pos);
	m_qCmpClass.xyz2enu(pos,E);
	exr[0]= E[1]; exr[1]= E[4]; exr[2]= E[7]; /* x = north */
	eyr[0]=-E[0]; eyr[1]=-E[3]; eyr[2]=-E[6]; /* y = west  */

	/* phase windup effect */
	m_qCmpClass.cross3(ek,eys,eks);
	m_qCmpClass.cross3(ek,eyr,ekr);
	for (i=0;i<3;i++) {
		ds[i]=exs[i]-ek[i]*m_qCmpClass.dot(ek,exs,3)-eks[i];
		dr[i]=exr[i]-ek[i]*m_qCmpClass.dot(ek,exr,3)+ekr[i];
	}
	cosp=m_qCmpClass.dot(ds,dr,3)/m_qCmpClass.norm(ds,3)/m_qCmpClass.norm(dr,3);

	//RTKLAB
	if      (cosp<-1.0) cosp=-1.0;
	else if (cosp> 1.0) cosp= 1.0;
    ph=acos(cosp)/2.0/MM_PI;
	m_qCmpClass.cross3(ds,dr,drs);
	if (m_qCmpClass.dot(ek,drs,3)<0.0) ph=-ph;

	*phw=ph+floor(*phw-ph+0.5); /* in cycle */

}
