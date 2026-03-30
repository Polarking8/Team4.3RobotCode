void Ramsete(void){
  //does ramsete math using actaulP and PandVdes to find left and right wheel velocities
  //calculate local error (relative to frame of robot) (rotation matrix included)
  errorP.x = cos(actualP.theta*pi/180.0)*(PandVdes.p.x-actualP.x) + sin(actualP.theta*pi/180.0)*(PandVdes.p.y-actualP.y);
  errorP.y = -sin(actualP.theta*pi/180.0)*(PandVdes.p.x-actualP.x) + cos(actualP.theta*pi/180.0)*(PandVdes.p.y-actualP.y);
  errorP.theta = PandVdes.p.theta - actualP.theta;

  //calculate gain ramK
  ramK = 2.0 * ramD * pow((pow(PandVdes.w*pi/180.0, 2.0)+ramB*pow(PandVdes.v, 2.0)), (1/2));

  //calculate robot velocities
  //deal with discontenouity when errorP.theta = 0;
  if (errorP.theta == 0){
    errorP.theta = 0.000001;
  }
  ramVdes = PandVdes.v * cos(errorP.theta*pi/180.0) + ramK * errorP.x;
  ramWdes = 180.0/pi* (PandVdes.w*pi/180.0 + ramK*errorP.theta*pi/180.0 + (ramB*PandVdes.v*sin(errorP.theta*pi/180.0)*errorP.y)/(errorP.theta*pi/180.0));

  mLVelDes = ramVdes + ramWdes*pi/180.0 * wheelSpacing/2.0;
  mRVelDes = ramVdes - ramWdes*pi/180.0 * wheelSpacing/2.0;

}