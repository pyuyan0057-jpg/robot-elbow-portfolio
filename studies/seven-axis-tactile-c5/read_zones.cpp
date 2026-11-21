#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

// Velocity at the coincident contact point includes rotational motion of each body.
static void point_velocity(const mjModel* m, const mjData* d, int body,
                           const mjtNum* point, double* v) {
    double twist[6], radius[3], arm[3];
    mj_objectVelocity(m, d, mjOBJ_BODY, body, twist, 0);
    // mjOBJ_BODY reports velocity at the inertial frame (body COM).
#ifdef REPRODUCE_C5_LEGACY_ORIGIN
    mju_sub3(radius, point, d->xpos + 3*body);
#else
    mju_sub3(radius, point, d->xipos + 3*body);
#endif
    mju_cross(arm, twist, radius);
    mju_add3(v, twist+3, arm);
}

extern "C" EXPORT void read_zones(const mjModel* m, const mjData* d,
    const int* zone_by_geom, const int* sites, int object, int table,
    const int* glove, double* rows, double* audit) {
    // Per zone: normal load, shear magnitude sum, load-weighted tangent speed.
    std::fill(rows, rows+18, 0.);
    std::fill(audit, audit+3, 0.);
    for (int i=0; i<d->ncon; ++i) {
        const auto& c=d->contact[i];
        int a=c.geom[0], b=c.geom[1];
        if (a<0 || b<0 || c.efc_address<0) continue;
        if (zone_by_geom[a]<0 && zone_by_geom[b]<0 && a!=object && b!=object) continue;
        double f[6], va[3], vb[3], rel[3], cf[3];
        mj_contactForce(m,d,i,f);
        if (f[0] <= 0.) continue;
        point_velocity(m,d,m->geom_bodyid[a],c.pos,va);
        point_velocity(m,d,m->geom_bodyid[b],c.pos,vb);
        mju_sub3(rel,va,vb);
        mju_mulMatVec3(cf,c.frame,rel);
        const double speed=1000.*std::hypot(cf[1],cf[2]);
        if (a==object || b==object) {
            const int other=(a==object ? b : a);
            if (other==table) audit[2]+=f[0];
            if (glove[other]) {audit[0]+=f[0];audit[1]+=f[0]*speed;}
        }
        // No object identity filter: table contact is also visible to a sensor.
        for(int g:{a,b}) {
            const int z=zone_by_geom[g];
            if(z<0) continue;
            const int sid=sites[z];
            double delta[3], p[3];
            mju_sub3(delta,c.pos,d->site_xpos+3*sid);
            mju_mulMatTVec3(p,d->site_xmat+9*sid,delta);
            double distance=0;
            for(int k=0;k<3;++k) distance+=std::pow(p[k]/m->site_size[3*sid+k],2);
            if(distance>1.) continue;
            rows[3*z]+=f[0]; rows[3*z+1]+=std::hypot(f[1],f[2]);
            rows[3*z+2]+=f[0]*speed;
        }
    }
    for(int z=0;z<6;++z) if(rows[3*z]>0) rows[3*z+2]/=rows[3*z];
    if(audit[0]>0) audit[1]/=audit[0];
}
