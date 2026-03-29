

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
      timeTrajStepFinish = abs(20.0/vNext); //for straight line distance/velocity
      break;

    //drive forward 20cm
    case 1:
      PandVdes = GenStraight((timeTraj-timeTrajStepStart), stepStartP, vNext);

      //end condition
      if (timeTraj >= timeTrajStepFinish){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenStraight(timeTrajStepFinish, stepStartP, vNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;
        //calcualate timeTrajStepFinish for the next step
        arcRadiusNext = 20;
        vNext = maxVel;
        timeTrajStepFinish = abs(90.0/180.0*pi*arcRadiusNext/vNext); //for arc, arc angle/180*pi*radius/velocity
      }
      break;

    //turn in left in 90deg w rad of 20cm
    case 2:
      PandVdes = GenArc((timeTraj-timeTrajStepStart), stepStartP, vNext, arcRadiusNext);

      //end condition
      if (timeTraj >= timeTrajStepFinish){
        //calculate what position step finished at to feed to next step start time
        //recalculating at at the theorectical time avoids error propogating through the steps.
        PandVdes = GenArc(timeTrajStepFinish, stepStartP, vNext, arcRadiusNext);
        stepStartP = PandVdes.p;
        
        trajStep = trajStep + 1;
        
        //set the time the next trajectory step starts
        timeTrajStepStart = timeTraj;
        //calcualate timeTrajStepFinish for the next step
        arcRadiusNext = 20;
        vNext = maxVel;
        //will depend on what the next trajectory step is if there is any
        //timeTrajStepFinish = timeTrajStepStart + abs(90.0/180.0*pi*arcRadiusNext/vNext); //for arc, arc angle/180*pi*radius/velocity
      }
      break;

    case 3: // end of trajectory //uses the fact that traj step = 3 to signal that its finished the trajectory
      PandVdes.v = 0;
      PandVdes.w = 0;
    
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

  PandVlocal.p.x = pInit.x + tLocal*vLocal*cos(pInit.theta/180*pi);
  PandVlocal.p.y = pInit.y + tLocal*vLocal*sin(pInit.theta/180*pi);

  return PandVlocal;
}

PandV GenArc(double tLocal, Pose pInit, double vLocal, double rLocal){
  //calculates ramsete vars relative to a 0,0,0 start point for a staight line
  //tLocal = time (sec) since start of this specific line
  //pInit = initial position (at start of trajectory)
  //vLocal = speed of trajectory (center of robot) in whatever direciton its headed.
  //double rLocal = radius of arc, positive is cetner point on right, negative is cp on left

  //theta = 0 means positive x direction, theta = 90 means positive y direciton
  PandV PandVlocal;
  //set vals for the stuff that doesnt change
  PandVlocal.v = vLocal;
  // w is independent of time
  PandVlocal.w = (vLocal/rLocal)*180.0/pi;

  PandVlocal.p.theta = pInit.theta + PandVlocal.w*tLocal;
  double relX = sin(vLocal/rLocal*tLocal);
  double relY = cos(vLocal/rLocal*tLocal) - rLocal;

  //do rotation matrix and inital cord offsets
  PandVlocal.p.x = pInit.x + relX * cos(PandVlocal.p.theta*pi/180.0) - relY * sin(PandVlocal.p.theta*pi/180.0);
  PandVlocal.p.y = pInit.y + relX * sin(PandVlocal.p.theta*pi/180.0) + relY * cos(PandVlocal.p.theta*pi/180.0);

  return PandVlocal;
}