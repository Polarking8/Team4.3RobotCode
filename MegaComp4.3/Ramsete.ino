
void Ramsete(void){
  //does ramsete math using actaulP and PandVdes to find left and right wheel velocities
  //calculate local error (relative to frame of robot) (rotation matrix included)
  errorP.x = cos(actualP.theta)*(PandVdes.p.x-actualP.x) + sin(actualP.theta)*(PandVdes.p.y-actualP.y);
  errorP.y = -sin(actualP.theta)*(PandVdes.p.x-actualP.x) + cos(actualP.theta)*(PandVdes.p.y-actualP.y);
  errorP.theta = wrapPi(PandVdes.p.theta - actualP.theta);

  //calculate gain ramK
  ramK = 2.0 * ramD * sqrt(PandVdes.w*PandVdes.w+ramB*PandVdes.v*PandVdes.v);

  //calculate robot velocities
  ramVdes = PandVdes.v * cos(errorP.theta) + ramK * errorP.x;
  ramWdes = PandVdes.w + ramK*errorP.theta + ramB*PandVdes.v*sinc(errorP.theta)*errorP.y;

  mLVelDes = ramVdes - ramWdes * wheelSpacing/2.0;
  mRVelDes = ramVdes + ramWdes * wheelSpacing/2.0;

}

//stolen from chris (thanks)
// sinc(x) = sin(x)/x, with a Taylor series for small x to avoid division by zero
static double sinc(double x)
{
    return fabs(x) < 1e-4 ? 1.0 - x * x / 6.0 : sin(x) / x;
}

