#include <mujoco/mujoco.h>
#include <cmath>
#include <algorithm>
#include <cstring>
#ifdef _WIN32
#define EXPORT_STEP __declspec(dllexport)
#else
#define EXPORT_STEP
#endif
extern "C" {
struct Register {
 int armq[7],armv[7],fq[14],fv[14],digit[14],oppq,oppv,ball,ballv,table,body,palm,little,thumb,labels[1024];
 double spring[14],A,H,radius,elastic,dt,tau,sensorTransmission,maxTorque[7],kp[7],kd[7];
};
// Returns the true simulated loads for OFFLINE evaluation. The only hand measurement
// sent to the control process is the delayed tension bridge voltage converted to N.
EXPORT_STEP void advance(mjModel* m,mjData* d,const Register* s,const double* refs,int count,double* sensor,double* out){
 double* dest=out;const double dt=m->opt.timestep;
 for(int n=0;n<count;n++,dest+=44){
  std::fill(dest,dest+44,0.);double take[5]={},f[5]={},jac[14];
  for(int i=0;i<7;i++){
    const int a=s->armv[i];const double tau=d->qfrc_bias[a]+s->kp[i]*(refs[i]-d->qpos[s->armq[i]])+s->kd[i]*(refs[i+7]-d->qvel[a]);
    d->qfrc_applied[a]=std::clamp(tau,-s->maxTorque[i],s->maxTorque[i]);dest[28+i]=d->qfrc_applied[a];dest[35+i]=d->qpos[s->armq[i]];
  }
  for(int j=0;j<14;j++){
   const double q=d->qpos[s->fq[j]],v=std::sin(q*.5),c=std::cos(q*.5);
   take[s->digit[j]]+=2*s->A*(1-c)+2*s->H*v-s->radius*q;jac[j]=s->A*v+s->H*c-s->radius;
  }
  for(int i=0;i<5;i++){f[i]=std::clamp((refs[14+i]-take[i])*(i==3?s->elastic:10000.),0.,25.);dest[10+i]=f[i];}
  sensor[0]+=(f[3]+.061*std::sin(2*3.14159265359*47*d->time)-sensor[0])*(1-std::exp(-dt/s->tau));
  // The proximal force is sensed before transmission loss across the wrist.
  f[3]*=s->sensorTransmission;
  for(int j=0;j<14;j++)d->qfrc_applied[s->fv[j]]=f[s->digit[j]]*jac[j]-(s->spring[j]+.001)*d->qpos[s->fq[j]]-.002;
  d->qfrc_applied[s->oppv]=std::clamp(1.5*(refs[19]-d->qpos[s->oppq])-.035*d->qvel[s->oppv],-.12,.12);
  for(int i=0;i<d->ncon;i++){
   auto& con=d->contact[i];int a=con.geom[0],b=con.geom[1];if(a!=s->ball&&b!=s->ball&&a!=s->table&&b!=s->table)continue;
   double w[6];mj_contactForce(m,d,i,w);
   if(a==s->ball||b==s->ball){int other=a==s->ball?b:a;dest[s->labels[other]]+=w[0];}
   else dest[8]+=w[0];
  }
  dest[9]=d->time;for(int i=0;i<3;i++)dest[15+i]=d->xpos[3*s->body+i];
  const double* R=d->xmat+9*s->palm;
  for(int j=0;j<3;j++)for(int i=0;i<3;i++)dest[18+j]+=R[3*i+j]*(dest[15+i]-d->xpos[3*s->palm+i]);
  dest[21]=d->geom_xmat[9*s->ball+5];dest[22]=d->geom_xpos[3*s->little+1]-d->geom_xpos[3*s->thumb+1];dest[23]=sensor[0];
  for(int i=0;i<3;i++)dest[24+i]=d->qvel[s->ballv+i];dest[27]=d->qvel[s->ballv+3]*d->qvel[s->ballv+3]+d->qvel[s->ballv+4]*d->qvel[s->ballv+4]+d->qvel[s->ballv+5]*d->qvel[s->ballv+5];
  dest[42]=d->ncon; dest[43]=take[3]; mj_step(m,d);
 }
}
}
