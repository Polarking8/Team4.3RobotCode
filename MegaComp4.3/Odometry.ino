void OdoUpdate(){
  //reads the encoders and performs math to update
  //sets global x, y, theta variables
  m1PosLast = m1Pos; //Save the old positions of m1;
  m2PosLast = m2Pos;

  //read new encoder values
  m1Pos = encoder1.read() * 1; //some constant that translates counts to linear cm;
  m2Pos = encoder2.read() * 1;

  //Calculate theta
  theta = theta + (m1Pos-m1PosOld-m2Pos+m2PosOld)/wheelSpacing/pi*360; //this will need calibration
  
  //calculate new xy cords
    //calculate distance moved forward (average of change in both encoder positions)
    //rotation matrix that with current theta to find offset from old position.
}