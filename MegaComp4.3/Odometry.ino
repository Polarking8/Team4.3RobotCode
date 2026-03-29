void OdoUpdate(){
  // Fudge factor to make distances more accurate
  double distanceFudgeFactor = 1.023;
  
  //reads the encoders and performs math to update
  //sets global x, y, theta variables
  //Units are degrees and centimeters
  mRPosLast = mRPos; //Save the old positions of mR; //right is motor 1
  mLPosLast = mLPos;

  //read new encoder values
  mRPos = encoderR.read() * 0.005890486 * distanceFudgeFactor; //constant converts counts to linear cm
  //formula as follows: 1 / 64.0(counts/rev) / 70(gear ratio) * 2.0(part of circumfrence formula) * pi * 4.2(wheel radius)
  mLPos = encoderL.read() * 0.005890486 * distanceFudgeFactor; 
  
  //Calculate theta
  actualP.theta = actualP.theta + (mRPos-mRPosLast-mLPos+mLPosLast)/wheelSpacing/pi*180.0; //this will need calibration
  
  //calculate new xy cords
  //calculate distance moved forward in the last cycle (average of change in both encoder positions)
  distanceMoved = ((mRPos-mRPosLast)+(mLPos-mLPosLast))/2.0;
  //rotation matrix that with current theta to find offset from old position.
  actualP.x = actualP.x + distanceMoved*cos(actualP.theta*pi/180.0); // find delta x with polar to cartesian conversion
  actualP.y = actualP.y + distanceMoved*sin(actualP.theta*pi/180.0);

  mRVel = (mRPos-mRPosLast)/deltaTTraj*alpha+mRVel*(alpha-1); // velocity, in cm/s 
  mLVel = (mLPos-mLPosLast)/deltaTTraj*alpha+mRVel*(alpha-1); 
}
