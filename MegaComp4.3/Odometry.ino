void OdoUpdate(){
  //reads the encoders and performs math to update
  //sets global x, y, theta variables
  //Units are degrees and centimeters
  mRPosLast = mRPos; //Save the old positions of mR; //right is motor 1
  mLPosLast = mLPos;

  //read new encoder values
  mRPos = encoderR.read() * 4.2 * 2 * Pi * 70 / 64; //some constant that translates counts to linear cm;
  mLPos = encoderL.read() * 4.2 * 2 * Pi * 70 / 64; // 4.2 is current wheel radius number, 2pi is needed, 70 is gear ratio, 64 is countsperrev
  
  //Calculate theta
  theta = theta + (mRPos-mRPosLast-mLPos+mLPosLast)/wheelSpacing/pi*360; //this will need calibration
  
  //calculate new xy cords
  //calculate distance moved forward (average of change in both encoder positions)
  distanceMoved = ((mRPos-mRPosLast)+(mLPos-mLPosLast))/2;
  //rotation matrix that with current theta to find offset from old position.
  x = (x*cosd(theta))+ (-1*sind(theta)*y); // rotation matrix  -- xcos theta -ysintheta
  y = (x*sind(theta))+(y*cosd(theta)); //  -- xsintheta + ycostheta

}
