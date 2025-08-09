#include "fdm.h"

static environment isa_env(float altitude_m){
    // Simple ISA up to 11km
    const float T0=288.15f; // K
    const float L =0.0065f; // K/m
    const float p0=101325.0f; // Pa
    const float R =287.058f; // J/(kg·K)
    const float g =9.80665f; // m/s^2
    float T = T0 - L*altitude_m;
    float p = p0 * powf(1.0f - L*altitude_m/T0, g/(R*L));
    float rho = p/(R*T);
    environment e={0};
    e.rho=rho; e.g=g; e.wind_x=0; e.wind_y=0; e.wind_z=0;
    return e;
}

void fdm_init_c172(fdm* f){
    aircraft a={0};
    a.mass = 1043.0f; // kg
    a.I = v3(1285.31f, 1824.93f, 2666.89f);
    a.wing_area = 16.2f;
    a.wing_span = 11.0f;
    a.chord     = a.wing_area / a.wing_span;
    a.cl_alpha  = 5.5f; // per rad
    a.cd0       = 0.03f;
    a.k_induced = 1.0f/(M_PI * 0.8f * (a.wing_span*a.wing_span/a.wing_area)); // k=1/(pi e AR)
    a.cm_alpha  = -0.38f;
    a.thrust_max= 2100.0f*9.81f/2.0f * 0.1f; // rough N (tuned low for simplicity)

    f->ac=a;
    f->pos=v3(0, 1000, 0);
    f->vel=v3(40, 0, 0);
    f->orient=q_from_euler(0, 0, 0);
    f->omega=v3(0,0,0);
    f->env = isa_env(f->pos.y);
    f->u=(controls){0,0,0,0.6f};
}

static void forces_and_moments(const fdm* f, vec3* F_world, vec3* M_body){
    const aircraft* a = &f->ac;
    // Transform velocity to body frame
    // For simplicity, treat vel as world; rotate into body by inverse quat
    quat qc = q(f->orient.x*-1, f->orient.y*-1, f->orient.z*-1, f->orient.w);
    vec3 v_rel_world = v3_sub(f->vel, v3(f->env.wind_x,f->env.wind_y,f->env.wind_z));
    vec3 v_body = q_rotate(qc, v_rel_world);
    float V = fmaxf(0.1f, v3_len(v_body));
    float qdyn = 0.5f * f->env.rho * V*V;

    // Angles
    float alpha = atan2f(v_body.y, v_body.x);
    float beta  = atan2f(v_body.z, V); // sideslip approx

    // Lift & drag (wing)
    float Cl = a->cl_alpha * alpha + 0.1f * f->u.elevator;
    // Stall clamp
    Cl = sf_clamp(Cl, -1.5f, 1.5f);
    float Cd = a->cd0 + a->k_induced * Cl*Cl;

    float L = qdyn * a->wing_area * Cl;
    float D = qdyn * a->wing_area * Cd;

    // Body axes: X forward, Y up, Z right (RH system). We'll assume Z to the right.
    vec3 F_aero = v3( -D, L, 0.0f);

    // Rudder yaw -> side force (very rough)
    float Cy = 0.2f*beta + 0.2f*f->u.rudder;
    F_aero.z += qdyn * a->wing_area * Cy;

    // Convert to world
    *F_world = q_rotate(f->orient, F_aero);
    // Gravity
    F_world->y -= f->env.g * a->mass;

    // Moments: pitch from Cm, roll from aileron, yaw from rudder
    float Cm = a->cm_alpha * alpha + 0.05f * f->u.elevator;
    float Cl_roll = 0.2f * f->u.aileron;
    float Cn = 0.06f * f->u.rudder;

    float My = qdyn * a->wing_area * a->chord * Cm;
    float Lroll = qdyn * a->wing_area * a->wing_span * Cl_roll;
    float Nn = qdyn * a->wing_area * a->wing_span * Cn;

    *M_body = v3(Lroll, My, Nn);

    // Thrust in body X
    float thrust = a->thrust_max * sf_clamp(f->u.throttle,0,1) * (1.0f - 0.001f*V);
    vec3 F_thr_body = v3(thrust, 0,0);
    vec3 F_thr_world = q_rotate(f->orient, F_thr_body);
    *F_world = v3_add(*F_world, F_thr_world);
}

static void deriv(const fdm* f, vec3* dx_pos, vec3* dx_vel, quat* dq, vec3* domega){
    vec3 Fw, Mb;
    forces_and_moments(f, &Fw, &Mb);

    *dx_pos = f->vel;

    vec3 acc = v3_scale(Fw, 1.0f/f->ac.mass);
    *dx_vel = acc;

    // Quaternion derivative: 0.5*q*omega
    quat wq = q(f->omega.x, f->omega.y, f->omega.z, 0);
    quat qdot = q_mul(f->orient, wq);
    qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f; qdot.w *= 0.5f;
    *dq = qdot;

    // Angular rates derivative: I^{-1}(M - omega x I omega)
    vec3 I = f->ac.I;
    vec3 Iw = v3(f->omega.x*I.x, f->omega.y*I.y, f->omega.z*I.z);
    vec3 cross = v3_cross(f->omega, Iw);
    vec3 rhs = v3_sub(Mb, cross);
    *domega = v3(rhs.x/I.x, rhs.y/I.y, rhs.z/I.z);
}

void fdm_step(fdm* f, float dt){
    // Clamp dt to avoid blowups; substep if large
    int sub = (dt>0.02f)? (int)ceilf(dt/0.01f) : 1;
    float h = dt/(float)sub;

    for(int s=0;s<sub;s++){
        // RK4
        fdm k1=*f, k2=*f, k3=*f, k4=*f;
        vec3 dx1, dv1, do1; quat dq1;
        deriv(&k1, &dx1,&dv1,&dq1,&do1);

        k2.pos = v3_add(f->pos, v3_scale(dx1, h*0.5f));
        k2.vel = v3_add(f->vel, v3_scale(dv1, h*0.5f));
        k2.orient = q_norm(q( f->orient.x + dq1.x*h*0.5f, f->orient.y + dq1.y*h*0.5f, f->orient.z + dq1.z*h*0.5f, f->orient.w + dq1.w*h*0.5f ));
        k2.omega = v3_add(f->omega, v3_scale(do1, h*0.5f));
        vec3 dx2,dv2,do2; quat dq2; deriv(&k2,&dx2,&dv2,&dq2,&do2);

        k3.pos = v3_add(f->pos, v3_scale(dx2, h*0.5f));
        k3.vel = v3_add(f->vel, v3_scale(dv2, h*0.5f));
        k3.orient = q_norm(q( f->orient.x + dq2.x*h*0.5f, f->orient.y + dq2.y*h*0.5f, f->orient.z + dq2.z*h*0.5f, f->orient.w + dq2.w*h*0.5f ));
        k3.omega = v3_add(f->omega, v3_scale(do2, h*0.5f));
        vec3 dx3,dv3,do3; quat dq3; deriv(&k3,&dx3,&dv3,&dq3,&do3);

        k4.pos = v3_add(f->pos, v3_scale(dx3, h));
        k4.vel = v3_add(f->vel, v3_scale(dv3, h));
        k4.orient = q_norm(q( f->orient.x + dq3.x*h, f->orient.y + dq3.y*h, f->orient.z + dq3.z*h, f->orient.w + dq3.w*h ));
        k4.omega = v3_add(f->omega, v3_scale(do3, h));
        vec3 dx4,dv4,do4; quat dq4; deriv(&k4,&dx4,&dv4,&dq4,&do4);

        f->pos = v3_add(f->pos, v3_scale(v3_add(v3_add(dx1, v3_scale(dx2,2)), v3_add(v3_scale(dx3,2), dx4)), h/6.0f));
        f->vel = v3_add(f->vel, v3_scale(v3_add(v3_add(dv1, v3_scale(dv2,2)), v3_add(v3_scale(dv3,2), dv4)), h/6.0f));

        f->orient = q_norm(q(
            f->orient.x + (dq1.x + 2*dq2.x + 2*dq3.x + dq4.x)*h/6.0f,
            f->orient.y + (dq1.y + 2*dq2.y + 2*dq3.y + dq4.y)*h/6.0f,
            f->orient.z + (dq1.z + 2*dq2.z + 2*dq3.z + dq4.z)*h/6.0f,
            f->orient.w + (dq1.w + 2*dq2.w + 2*dq3.w + dq4.w)*h/6.0f
        ));

        f->omega = v3_add(f->omega, v3_scale(v3_add(v3_add(do1, v3_scale(do2,2)), v3_add(v3_scale(do3,2), do4)), h/6.0f));

        // Update environment (density) by altitude
        f->env = isa_env(f->pos.y);

            }
}
}

float fdm_get_airspeed(const fdm* f){
    quat qc = q(-f->orient.x, -f->orient.y, -f->orient.z, f->orient.w);
    vec3 v_body = q_rotate(qc, f->vel);
    return v3_len(v_body);
}
float fdm_get_altitude(const fdm* f){ return f->pos.y; }

void fdm_set_controls(fdm* f, float elev, float ail, float rud, float thr){
    f->u.elevator = sf_clamp(elev, -1, 1);
    f->u.aileron  = sf_clamp(ail, -1, 1);
    f->u.rudder   = sf_clamp(rud, -1, 1);
    f->u.throttle = sf_clamp(thr, 0, 1);
}

void fdm_reset(fdm* f){
    f->pos=v3(0, 1000, 0);
    f->vel=v3(40, 0, 0);
    f->orient=q_from_euler(0, 0, 0);
    f->omega=v3(0,0,0);
}
