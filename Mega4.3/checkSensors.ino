bool checkSilverfish(void){
  int hallVal = analogRead(HallEffect);
  if (hallVal < 360 || hallVal > 550) {
    return true;
  } else{
    return false;
  }
}

char checkColor(void){
  // Select RED Filter
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(10);
  for (int i = 0; i < numSamples; i++){
    R[i] = readPulse(); // Read red (frequency)
  }

  // Select BLUE Filter
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delay(10);
  for (int i = 0; i < numSamples; i++){
    B[i] = readPulse();
  }

   // Select GREEN Filter
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(10);
  for (int i = 0; i < numSamples; i++){
    G[i] = readPulse();
  }

  // Select CLEAR Filter
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);
  delay(10);
  for (int i = 0; i < numSamples; i++){
    C[i] = readPulse();
  }

  // Calculate moving averages
  RF = 1 / movingAverage(R);
  GF = 1 / movingAverage(G);
  BF = 1 / movingAverage(B);
  CF = 1 / movingAverage(C);  

  // Values normalized by clear
  RN = 100 * RF / CF;
  GN = 100 * GF / CF;
  BN = 100 * BF / CF;
  
  //Print Vals
  // TODO: CHANGE TO SERIAL2
  Serial.print(RN, 4);
  Serial.print(",\t");
  Serial.print(GN, 4);
  Serial.print(",\t");
  Serial.print(BN, 4);
  Serial.print(",\t");
  Serial.println(CF, 4);

  // Map color sensor output to color guess
  if ((55 < BN  && BN < 82) && (10 < RN && RN < 20) && (22 < GN && GN < 20)) {
    //Serial.println("Blue block detected");
    return 'b';
  } else if ((20 < BN && BN < 30) && (60 < RN && RN < 80) && (10 < GN && GN < 20)) {
    //Serial.println("Red block detected");
    return 'r';
  } else if ((20 < BN && BN < 50) && (40 < RN && RN < 80) && (30 < GN && GN < 60)) {
    //Serial.println("Yellow block detected");
    return 'y';
  } else {
    //Serial.println("Unable to determine block color");
    return 'e';
  }
}

float readPulse(){
  return pulseIn(sOut, LOW)+pulseIn(sOut, HIGH);
}

float movingAverage(float * arr) {
  float sum = 0;
  for (int i = 0; i < numSamples; i++){
    sum += arr[i]/numSamples;
  }
  return sum;
} 

        