#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include "comm/control.h"
#include "comm/changeInitialConditions.h"
#include <iostream>
#include <cmath>
#include <cvodes/cvodes.h>           
#include <nvector/nvector_serial.h>  
#include <cvodes/cvodes_dense.h>
#include <cstdlib>

using namespace std;

#define TB       RCONST(0.0)  // start time [s]  
//#define T        RCONST(1.0)  // time horizon [s]

#define RELTOL   RCONST(1.0e-3)
#define ABSTOL   RCONST(1.0e-4)

#define Q0_1     RCONST(10.0)       
#define Q0_2     RCONST(0.0)
#define Q0_3     RCONST(M_PI/2)

#define U0_1     RCONST(1.0)  // Initial control 
#define U0_2     RCONST(0.0)

#define M        2            // control dimention
#define N        3            // state dimention

typedef struct {
  realtype u[M];
} sPrm;

static int  fun(realtype t, N_Vector q, N_Vector dq, void *prm);


N_Vector  q;
realtype *data, t, x, y, phi;
void     *cvode_mem;
int       err;
sPrm      prm;
ros::Publisher pose_out;
float lastTime;

bool changeInitialConditions(comm::changeInitialConditions::Request  &req,
			     comm::changeInitialConditions::Response &res){
  //realtype *_data = NV_DATA_S(q);
  data[0] = req.x;
  data[1] = req.y;
  data[2] = req.theta;
  res.dummy = 1;
  return true;
}


void timeCallback(const std_msgs::Float32::ConstPtr& time){
  geometry_msgs::Pose2D msg;
  float T = time->data - lastTime;
  cout << "T: " << T << endl;
  cvode_mem = CVodeCreate(CV_ADAMS, CV_FUNCTIONAL);
  if(cvode_mem == NULL){
    exit(EXIT_FAILURE);
  }
  err = CVodeInit(cvode_mem, fun, TB, q);
  err = CVodeSStolerances(cvode_mem, RELTOL, ABSTOL);
  err = CVodeSetUserData(cvode_mem, &prm);
  err = CVDense(cvode_mem, N);
  err = CVode(cvode_mem, T, q, &t, CV_NORMAL);  
  msg.x = NV_Ith_S(q,0);
  msg.y = NV_Ith_S(q,1);
  msg.theta = NV_Ith_S(q,2);
  /* if(msg.theta  > 2*M_PI)
    msg.theta -= 2*M_PI;
  if(msg.theta  < 0)
  msg.theta = 2*M_PI - msg.theta; */
  pose_out.publish(msg);
  // cout << "theta: " << msg.theta << endl;
}

void controlCallback(const comm::control::ConstPtr& ctrl){
  prm.u[0] = ctrl->u1.data;
  prm.u[1] = ctrl->u2.data;
}

int main(int argc, char **argv){
  lastTime = 0.0;
  ros::init(argc, argv, "robot");
  ros::NodeHandle n;
  pose_out = n.advertise<geometry_msgs::Pose2D>("pose", 1000);
  ros::Subscriber time_in = n.subscribe("time", 1000, timeCallback);
  ros::Subscriber ctrl_in = n.subscribe("control", 1000, controlCallback);
  ros::ServiceServer service = n.advertiseService("changeInitialConditions", changeInitialConditions);
  
  /* Allocate state vector q */
  q = N_VNew_Serial(N);
  if(q == NULL){
    exit(EXIT_FAILURE);
  }
  /* Get pointer to data array in vector q */
  data = NV_DATA_S(q);
  /* Set initial state */
  data[0] = Q0_1;
  data[1] = Q0_2;
  data[2] = Q0_3;
  /* Set control */
  prm.u[0] = U0_1;
  prm.u[1] = U0_2;
  
  ros::spin();

  N_VDestroy_Serial(q);
  CVodeFree(&cvode_mem);
  return 0;

}


int fun(realtype t, N_Vector q, N_Vector dq, void *prm)
{
  realtype  x, y, phi;
  realtype  dx, dy, dphi;
  realtype  u1, u2;
  sPrm     *lprm;
  
  /* Get actual state */
  x = NV_Ith_S(q,0);
  y = NV_Ith_S(q,1);
  phi = NV_Ith_S(q,2);
  /* Get control signals */
  lprm = (sPrm*)prm;
  u1 = lprm->u[0];
  u2 = lprm->u[1];
  /* Calculate function */
  dx = cos(phi) * u1;
  dy = sin(phi) * u1;
  dphi = u2;
  /* Set output */
  NV_Ith_S(dq,0) = dx;
  NV_Ith_S(dq,1) = dy;
  NV_Ith_S(dq,2) = dphi;
  return 0;
}
