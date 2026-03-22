void readDistanceSensor(){
  // code to read the distance sensor and apply filtering and linearization
  distVal = analogRead(DistanceSensor);
  //add stuff to filter and do linearization
}

void readHallSensor(){
  hallVal = analogRead(HallEffect);
  //add filtering
}

bool checkSilverfish(){
  readHallSensor(); //sets hallVal global variable
  Serial.print("hall effect value is ");
  Serial.println(hallVal);
  if (hallVal < 360 || hallVal > 550) {
    return true;
  } else{
    return false;
  }
}

void readReflectanceSensor(){
  //reads the reflectance sensor and updates the following global variables
  //lineSensorValues //array of raw sensor readings
  //lineSensorValuesUnbiased //intermediate
  //lineAi, lineAid //intermediates
  //linePosition //position of the line in cm relative to center of sensor

  qtr.read(lineSensorValues); //get data (sets lineSensorValues to raw data)
  for (int i = 0; i < lineSensorCount; i++){
    lineSensorValuesUnbiased[i] = lineSensorValues[i] - lineSensorBias[i]; //removes bias
    if(lineSensorValuesUnbiased[i]>5000){ //remove outliers I think
      lineSensorValuesUnbiased[i] = 0;
    }
    //Serial.print(lineSensorValuesUnbiased[i]); //optional print the unbiased values
    //Serial.print('\t');
  }
  lineAi = 0;
  lineAid = 0;
  for(int i = 0; i < lineSensorCount; i++){
    lineAid = lineAid + lineSensorValuesUnbiased[i] * lineSensorPositions[i]; //do weighted average to find "center of mass"
    lineAi = lineAi + lineSensorValuesUnbiased[i];
  }
  //if lineAi = 0 it will cause a divide by zero error and break the code. Fix this by if lineAi = 0 set to very small number
  if (lineAi == 0){
    lineAi = 0.1;
    //can also use this as a line detection validity, if Ai = 0 or is very small there probably isnt a line in sight and value isnt valid;
  }
  linePosition = lineAid/lineAi-2.8; //-2.8 to put 0 in center of sensor

  //Serial.print(lineAid); //more optional telemetry
  //Serial.print('\t');
  //Serial.print(lineAi);
  //Serial.print('\t');
  //Serial.println(linePosition);

}

char readColorSensor(){
  //note this code is blocking and takes significant time to execute.
  //reads the color sensor and updates the following global variables. 
  //colorR, colorG, colorB, colorC //raw color sensor data arrays
  //colorRF, colorGF, colorBF, colorCF //filtered color weights
  //colorRN, colorGN, colorBN //normalized data
  //Also returns a char corosponding to the color "r,b,y, or e if error"
  
  // Select RED Filter
  digitalWrite(ColorS2, LOW);
  digitalWrite(ColorS3, LOW);
  delay(10);
  for (int i = 0; i < colorNumSamples; i++){
    colorR[i] = readPulse(); // Read red (frequency)
  }

  // Select BLUE Filter
  digitalWrite(ColorS2, LOW);
  digitalWrite(ColorS3, HIGH);
  delay(10);
  for (int i = 0; i < colorNumSamples; i++){
    colorB[i] = readPulse();
  }

   // Select GREEN Filter
  digitalWrite(ColorS2, HIGH);
  digitalWrite(ColorS3, HIGH);
  delay(10);
  for (int i = 0; i < colorNumSamples; i++){
    colorG[i] = readPulse();
  }

  // Select CLEAR Filter
  digitalWrite(ColorS2, HIGH);
  digitalWrite(ColorS3, LOW);
  delay(10);
  for (int i = 0; i < colorNumSamples; i++){
    colorC[i] = readPulse();
  }

  // Calculate moving averages
  colorRF = 1 / movingAverage(colorR);
  colorGF = 1 / movingAverage(colorG);
  colorBF = 1 / movingAverage(colorB);
  colorCF = 1 / movingAverage(colorC);  

  // Values normalized by clear
  colorRN = 100 * colorRF / colorCF;
  colorGN = 100 * colorGF / colorCF;
  colorBN = 100 * colorBF / colorCF;
  
  //Print Vals
  Serial.print(colorRN, 4);
  Serial.print(",\t");
  Serial.print(colorGN, 4);
  Serial.print(",\t");
  Serial.print(colorBN, 4);
  Serial.print(",\t");
  Serial.println(colorCF, 4);

  // Map color sensor output to color guess
  if ((65 < colorBN  && colorBN < 80) && (0 < colorRN && colorRN < 10) && (20 < colorGN && colorGN < 30)) {
    //Serial.println("Blue block detected");
    return 'b';
  } else if ((5 < colorBN && colorBN < 20) && (80 < colorRN && colorRN < 105) && (0 < colorGN && colorGN < 20)) {
    //Serial.println("Red block detected");
    return 'r';
  } else if ((10 < colorBN && colorBN < 25) && (30 < colorRN && colorRN < 40) && (45 < colorGN && colorGN < 60)) {
    //Serial.println("Yellow block detected");
    return 'y';
  } else {
    //Serial.println("Unable to determine block color");
    return 'e';
  }
}

//accessory for readColorSensor
float readPulse(){
  return pulseIn(ColorIN, LOW)+pulseIn(ColorIN, HIGH);
}

float movingAverage(float * arr) {
  float sum = 0;
  for (int i = 0; i < colorNumSamples; i++){
    sum += arr[i]/colorNumSamples;
  }
  return sum;
} 