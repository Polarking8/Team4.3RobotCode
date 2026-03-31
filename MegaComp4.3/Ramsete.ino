void Ramsete(void){
  //does ramsete math using actaulP and PandVdes to find left and right wheel velocities
  //calculate local error (relative to frame of robot) (rotation matrix included)
  errorP.x = cos(actualP.theta)*(PandVdes.p.x-actualP.x) + sin(actualP.theta)*(PandVdes.p.y-actualP.y);
  errorP.y = -sin(actualP.theta)*(PandVdes.p.x-actualP.x) + cos(actualP.theta)*(PandVdes.p.y-actualP.y);
  errorP.theta = PandVdes.p.theta - actualP.theta;

  //calculate gain ramK
  ramK = 2.0 * ramD * pow((pow(PandVdes.w, 2.0)+ramB*pow(PandVdes.v, 2.0)), (1/2));

  //calculate robot velocities
  //deal with discontenouity when errorP.theta = 0;
  if (errorP.theta == 0){
    errorP.theta = 0.000001;
  }
  ramVdes = PandVdes.v * cos(errorP.theta) + ramK * errorP.x;
  ramWdes = PandVdes.w + ramK*errorP.theta + (ramB*PandVdes.v*sin(errorP.theta)*errorP.y)/errorP.theta;

  mLVelDes = ramVdes + ramWdes * wheelSpacing/2.0;
  mRVelDes = ramVdes - ramWdes * wheelSpacing/2.0;

}