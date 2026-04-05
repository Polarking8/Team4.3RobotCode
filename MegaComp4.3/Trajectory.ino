

void GenTrajectory(void){
  //sets global vars for current desired position, and desired velocity in time since trajectory start
  //while driving trajectories this should be called every cycle
  //on starting driving trajectory, set trajStep to 0 and all else is covered internaly.

  //reset timers and state var if the command is fresh
  switch (trajStep){
    case 0:
      //reset timers and starting position
      timeTrajStepStart = timeTraj;
      stepStartP = initialP;
      trajStep = trajStep + 1;

      //calcualate timeTrajStepFinish for the next step
      vNext = maxVel;
      timeTrajStepFinish = abs(30.0/vNext); //for straight line distance/velocity
      break;

    //drive forward 30cm
    case 1:
      PandVdes = GenStraight((timeTraj-timeTrajStepStart), stepStartP, vNext);

      //end condition
      if (timeTraj-timeTrajStepStart >= timeTrajStepFinish){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenStraight(timeTrajStepFinish, stepStartP, vNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;
        //calcualate timeTrajStepFinish for the next step
        arcRadiusNext = 60.0;
        vNext = maxVel;
        timeTrajStepFinish = abs(45.0/180.0*pi*arcRadiusNext/vNext); //for arc, arc angle/180*pi*radius/velocity
      }
      break;

    //turn in left in 45deg w rad of 60cm
    case 2:
      PandVdes = GenArc((timeTraj-timeTrajStepStart), stepStartP, vNext, arcRadiusNext);

      //end condition
      if (timeTraj-timeTrajStepStart >= timeTrajStepFinish){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenArc(timeTrajStepFinish, stepStartP, vNext, arcRadiusNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;

        //calcualate timeTrajStepFinish for the next step
        arcRadiusNext = -60.0;
        vNext = maxVel;
        timeTrajStepFinish = abs(45.0/180.0*pi*arcRadiusNext/vNext); //for arc, arc angle/180*pi*radius/velocity
      }
      break;

    //turn in right in 45deg w rad of 60cm
    case 3:
      PandVdes = GenArc((timeTraj-timeTrajStepStart), stepStartP, vNext, arcRadiusNext);

      //end condition
      if (timeTraj-timeTrajStepStart >= timeTrajStepFinish){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenArc(timeTrajStepFinish, stepStartP, vNext, arcRadiusNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;

        //calcualate timeTrajStepFinish for the next step
        vNext = maxVel;
        timeTrajStepFinish = abs((86.54-5)/vNext); //for straight line distance/velocity
        
      }
      break;
    
    case 4: //drive most the distance to distance sensor point (stop 5 theoretical cm short)
      PandVdes = GenStraight((timeTraj-timeTrajStepStart), stepStartP, vNext);
      //prime iir filter for next case
      readDistanceSensor();
      //end condition
      if (timeTraj-timeTrajStepStart >= timeTrajStepFinish){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenStraight(timeTrajStepFinish, stepStartP, vNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;

        //calcualate timeTrajStepFinish for the next step
        vNext = 10.0; //slower for this to make sure we dont run past distance target
        timeTrajStepFinish = abs(10/vNext); //for straight line distance/velocity
        
      }
      break;

    case 5: //drive until distance sensor trips or 10cm (5cm past theoretical)
      PandVdes = GenStraight((timeTraj-timeTrajStepStart), stepStartP, vNext);

      //end condition
      readDistanceSensor();
      if ((timeTraj-timeTrajStepStart >= timeTrajStepFinish) || (distVal>73)){ //stop if distance sensor trips //75 is was found experimetnaly (73 to account for iir delay)
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenStraight(timeTrajStepFinish, stepStartP, vNext);
        stepStartP = PandVdes.p;
        //override x codinate with theoretical since sensor says so
        actualP.x = 205.84;
        stepStartP.x = actualP.x;
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;
        
        //calcualate timeTrajStepFinish for the next step
        arcRadiusNext = 12.5;
        vNext = 10.0;
        timeTrajStepFinish = abs((90.0+30)/180.0*pi*arcRadiusNext/vNext); //for arc, arc angle/180*pi*radius/velocity
      }
      break;

    //turn left 90 deg (theoretical) w r=12.5 or until line sensor reads certain value (not yet implemented)
    case 6:
      PandVdes = GenArc((timeTraj-timeTrajStepStart), stepStartP, vNext, arcRadiusNext);

      //get data from reflectance sensor
      readReflectanceSensor();
      readDistanceSensor(); //also prime distance sensor iir filter

      //end condition //if 30 deg past theoretical or line sensor is valid and correct)
      if ((timeTraj-timeTrajStepStart >= timeTrajStepFinish) || (onLine && (lineError<=0))){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenArc(timeTrajStepFinish*90.0/120.0, stepStartP, vNext, arcRadiusNext); //90/120 to use theoretical to set new position
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;

        //comand starting line following
        followingLine = true;

        //calcualate timeTrajStepFinish for the next step (backup in case distance fails
        vNext = 10; //used by line following outside
        timeTrajStepFinish = abs((20.5+10.0)/vNext); //for straight line distance/velocity
      }
      break;

    case 7: //drive 20.5cm until distance sensor trip also line follow
      //generated values dont get used because line following
      PandVdes = GenStraight((timeTraj-timeTrajStepStart), stepStartP, vNext);

      //end condition
      readDistanceSensor();
      //73 is temporary, find a value that stops 6.6 cm from button.
      if ((timeTraj-timeTrajStepStart >= timeTrajStepFinish) ||  (distVal>73)){  //stop if distance sensor trips //75 is was found experimetnaly (73 to account for iir delay)
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenStraight(timeTrajStepFinish*20.5/30.5, stepStartP, vNext); //uses theoretical
        stepStartP = PandVdes.p;
        //override x, y and theta codinate, known from line and ditance sensor
        actualP.x = 218.34;
        actualP.y = 83.336;
        actualP.theta = 90.0;
        stepStartP = actualP;
        
        //disable line following
        followingLine = false;

        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;

        //calcualate timeTrajStepFinish for the next step
        arcRadiusNext = 15.0;
        vNext = 10.0;
        timeTrajStepFinish = abs(10.0/180.0*pi*arcRadiusNext/vNext); //for arc, arc angle/180*pi*radius/velocity
      }
      break;

    //turn left 10 deg w r=15
    case 8:
      PandVdes = GenArc((timeTraj-timeTrajStepStart), stepStartP, vNext, arcRadiusNext);

      //drop conveyor
      conveyorPower = -400;

      //end condition //if 30 deg past theoretical or line sensor is valid and correct)
      if ((timeTraj-timeTrajStepStart >= timeTrajStepFinish) || (onLine && (lineError<=0))){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenArc(timeTrajStepFinish, stepStartP, vNext, arcRadiusNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;

        //calcualate timeTrajStepFinish for the next step
        vNext = 10;
        timeTrajStepFinish = abs((4.0+4.0)/vNext); //4cm theoretical, 4 more to slip wheels a bit after hitting pushblocks //for straight line distance/velocity
      }
      break;

    case 9: //drive most the distance to distance sensor point (stop 5 theoretical cm short)
      PandVdes = GenStraight((timeTraj-timeTrajStepStart), stepStartP, vNext);
      //prime iir filter for next case
      readDistanceSensor();
      //end condition
      if (timeTraj-timeTrajStepStart >= timeTrajStepFinish){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenStraight(timeTrajStepFinish, stepStartP, vNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;

        //put here if you want to stop
        //set velocities to zero to stop
        PandVdes.v = 0;
        PandVdes.w = 0;
      }
      break;

    //dont forget to update this number in main tab
    case 10: // end of trajectory //uses the fact that traj step = 5 to signal that its finished the trajectory
      //doesnt actualy get here
      break;
    
    default:
      Serial.println("trajectory error, be woo be woo be woo");
      break;
  }
}

PandV GenStraight(double tLocal, Pose pInit, double vLocal){
  //calculates ramsete vars relative to a 0,0,0 start point for a staight line
  //tLocal = time (sec) since start of this specific line
  //pInit, initial pose
  //v = speed of trajectory (center of robot) in whatever direciton its headed.

  //theta = 0 means positive x direction, theta = 90 means positive y direciton
  PandV PandVlocal;
  //set vals for the stuff that doesnt change
  PandVlocal.p.theta = pInit.theta;
  PandVlocal.v = vLocal;
  PandVlocal.w = 0;

  PandVlocal.p.x = pInit.x + tLocal*vLocal*cos(pInit.theta);
  PandVlocal.p.y = pInit.y + tLocal*vLocal*sin(pInit.theta);

  return PandVlocal;
}

PandV GenArc(double tLocal, Pose pInit, double vLocal, double rLocal){
  //calculates ramsete vars relative to a 0,0,0 start point for a staight line
  //tLocal = time (sec) since start of this specific line
  //pInit = initial position (at start of trajectory)
  //vLocal = speed of trajectory (center of robot) in whatever direciton its headed.
  //double rLocal = radius of arc, positive is center point on left, negative is cp on right

  //theta = 0 means positive x direction, theta = 90 means positive y direciton
  PandV PandVlocal;
  //set vals for the stuff that doesnt change
  PandVlocal.v = vLocal;
  // w is independent of time
  PandVlocal.w = vLocal/rLocal;

  PandVlocal.p.theta = pInit.theta + PandVlocal.w*tLocal;
  double relX = rLocal*sin(vLocal/rLocal*tLocal);
  double relY = -rLocal*cos(vLocal/rLocal*tLocal) + rLocal;

  //do rotation matrix and inital cord offsets
  PandVlocal.p.x = pInit.x + relX * cos(pInit.theta) - relY * sin(pInit.theta);
  PandVlocal.p.y = pInit.y + relX * sin(pInit.theta) + relY * cos(pInit.theta);

  return PandVlocal;
}