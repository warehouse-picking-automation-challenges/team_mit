// Author: Alberto (adapted from Robbie Paolini)

#include "teleop.hpp"

void updateCartesian(const robot_comm::robot_CartesianLogConstPtr& msg)
{
  if (!readingCartesian)
  {
    newCartesian[0] = msg->x;
    newCartesian[1] = msg->y;
    newCartesian[2] = msg->z;
    newCartesian[3] = msg->q0;
    newCartesian[4] = msg->qx;
    newCartesian[5] = msg->qy;
    newCartesian[6] = msg->qz;
    updatedCartesian = true;
  }
}

void updateJoints(const robot_comm::robot_JointsLogConstPtr& msg)
{
  if (!readingJoints)
  {
    newJoints[0] = msg->j1;
    newJoints[1] = msg->j2;
    newJoints[2] = msg->j3;
    newJoints[3] = msg->j4;
    newJoints[4] = msg->j5;
    newJoints[5] = msg->j6;
    updatedJoints = true;
  }
}

void updateOther()
{
  if (!readingOther)
    {
      robot.GetSpeed(newSpeed[0], newSpeed[1]);
      robot.GetWorkObject(newWorkObj);
      robot.GetTool(newTool);
      robot.GetInertia(newInertia);
      updatedOther = true;
    }
}

bool initialize()
{
  //robot.subscribe(nodePtr);

  if (!robot.Ping())
  {
    ROS_WARN("Robot not ready!");
    return false;
  }
  
  if (!robot.SetComm(NON_BLOCKING))
  {
    ROS_WARN("Unable to set communication with the robot.");
    return false;
  }

  if (!robot.SetZone(ROBOT_ZONE))
  {
    ROS_WARN("Unable to set the zone of robot!");
    return false;
  }

  if (!robot.SetSpeed(ROBOT_SPEED_TCP,ROBOT_SPEED_ORI))
  {
    ROS_WARN("Unable to set the speed of the robot!");
    return false;
  }

  updatedCartesian = false;
  updatedJoints = false;
  updatedOther = false;
  readingCartesian = false;
  readingJoints = false;
  readingOther = false;

  // Start with Coarse motion mode
  linStep = LIN_STEP_3;
  angStep = ANG_STEP_3;
  
  //Start logging state of the robot
  robot.subscribeCartesian(nodePtr, 100, &updateCartesian);
  robot.subscribeJoints(nodePtr, 100, &updateJoints);
  updateOther();

  //Initialize the visualization of the console
  exitProgram = false;
  enabled = false;
  console_init();
  console_update_motion_mode(linStep);

  last_key = NO_KEY;

  return true;
}

void updateState()
{
  if (updatedCartesian)
  {
    readingCartesian = true; //Thread safe
    memcpy(curCartesian, newCartesian, 7*sizeof(double));
    readingCartesian = false;
    console_update_cartesian(curCartesian);
    updatedCartesian = false;
  }
  if (updatedJoints)
  {
    readingJoints = true; //Thread safe
    memcpy(curJoints, newJoints, 6*sizeof(double));
    readingJoints = false;
    console_update_joints(curJoints);
    updatedJoints = false;
  }
  if (updatedOther)
  {
    readingOther = true; //Thread safe
    memcpy(curWorkObj, newWorkObj, 7*sizeof(double));
    memcpy(curTool, newTool, 7*sizeof(double));
    memcpy(curInertia, newInertia, 7*sizeof(double));
    memcpy(curSpeed, newSpeed, 2*sizeof(double));
    console_update_workObj(curWorkObj);
    console_update_tool(curTool);
    console_update_inertia(curInertia);
    console_update_speed(curSpeed);

    readingOther = false;
    updatedOther = false;
  }
}

void executeUserCommand()
{
  KEY_CMDS cur_key = console_get_key();

  // If we are no longer pressing the same key, make sure we 
  // reflect this in the console
  if (cur_key != last_key)
    console_clear_key(last_key);

  // If this is an enable key, setup communication,
  // stop any movement, and we're done. 
  if (cur_key == ENABLE)
  {
    if (!enabled)
    {
      enabled = true;
      console_update_enabled(enabled);
      robot.SetComm(NON_BLOCKING);
      robot.SetZone(ROBOT_ZONE);
      robot.Stop();
    }
    last_key = ENABLE;
    return;
  }

  // If this is a disable key, stop movement, and we're done
  if (cur_key == DISABLE)
  {
    if (enabled)
    {
      enabled = false;
      console_update_enabled(enabled);
      robot.Stop();
    }
    last_key = DISABLE;
    return;
  }

  // If we're not enabled, and the user pressed something that's not quit, 
  // alert them that they're not doing anything useful
  if (!enabled)
  {
    if (cur_key == NO_KEY)
    {
      last_key = NO_KEY;
      return;
    }
    if (cur_key != QUIT)
    {
      console_flash();
      last_key = cur_key;
      return;
    }
  }

  //Before serving any motion command we update the current state to
  //possible new readings.
  updateState();

  switch (cur_key)
  {
    // Incremental cartesian Moves
    case DEC_X:
        robot.SetCartesian(curCartesian[0] - linStep, curCartesian[1], curCartesian[2], curCartesian[3],
            curCartesian[4], curCartesian[5], curCartesian[6]);
        break;
    case DEC_Y:
        robot.SetCartesian(curCartesian[0], curCartesian[1] - linStep, curCartesian[2], curCartesian[3],
            curCartesian[4], curCartesian[5], curCartesian[6]);
        break;
    case DEC_Z:
      robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2] - linStep, curCartesian[3],
                          curCartesian[4], curCartesian[5], curCartesian[6]);
      break;
    case INC_X:
      robot.SetCartesian(curCartesian[0] + linStep, curCartesian[1], curCartesian[2], curCartesian[3],
                          curCartesian[4], curCartesian[5], curCartesian[6]);
      break;
    case INC_Y:
      robot.SetCartesian(curCartesian[0], curCartesian[1] + linStep, curCartesian[2], curCartesian[3],
                          curCartesian[4], curCartesian[5], curCartesian[6]);
      break;
    case INC_Z:
      robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2] + linStep, curCartesian[3],
                          curCartesian[4], curCartesian[5], curCartesian[6]);
      break;

    // Incremental Cartesian Rotations
    case DEC_RX:
      {
        Quaternion orient;
        orient[0] = curCartesian[3];
        orient[1] = curCartesian[4];
        orient[2] = curCartesian[5];
	orient[3] = curCartesian[6];

	RotMat r;
	r.rotX(-angStep*DEG2RAD);
        Quaternion increment = r.getQuaternion();

        orient = increment ^ orient;
        robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2], orient[0],
            orient[1], orient[2], orient[3]);
        break;
      }
    case DEC_RY:
      {
        Quaternion orient;
        orient[0] = curCartesian[3];
        orient[1] = curCartesian[4];
        orient[2] = curCartesian[5];
	orient[3] = curCartesian[6];

	RotMat r;
	r.rotY(-angStep*DEG2RAD);
        Quaternion increment = r.getQuaternion();

        orient = increment ^ orient;
        robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2], orient[0],
            orient[1], orient[2], orient[3]);
        break;
      }
    case DEC_RZ:
      {
         Quaternion orient;
        orient[0] = curCartesian[3];
        orient[1] = curCartesian[4];
        orient[2] = curCartesian[5];
	orient[3] = curCartesian[6];

	RotMat r;
	r.rotZ(-angStep*DEG2RAD);
        Quaternion increment = r.getQuaternion();

        orient = increment ^ orient;
        robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2], orient[0],
            orient[1], orient[2], orient[3]);
        break;
     }
    case INC_RX:
      {
        Quaternion orient;
        orient[0] = curCartesian[3];
        orient[1] = curCartesian[4];
        orient[2] = curCartesian[5];
	orient[3] = curCartesian[6];

	RotMat r;
	r.rotX(angStep*DEG2RAD);
        Quaternion increment = r.getQuaternion();

        orient = increment ^ orient;
        robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2], orient[0],
            orient[1], orient[2], orient[3]);
        break;
      }
    case INC_RY:
      {
        Quaternion orient;
        orient[0] = curCartesian[3];
        orient[1] = curCartesian[4];
        orient[2] = curCartesian[5];
	orient[3] = curCartesian[6];

	RotMat r;
	r.rotY(angStep*DEG2RAD);
        Quaternion increment = r.getQuaternion();

        orient = increment ^ orient;
        robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2], orient[0],
            orient[1], orient[2], orient[3]);
        break;
      }
    case INC_RZ:
      {
        Quaternion orient;
        orient[0] = curCartesian[3];
        orient[1] = curCartesian[4];
        orient[2] = curCartesian[5];
	orient[3] = curCartesian[6];

	RotMat r;
	r.rotZ(angStep*DEG2RAD);
        Quaternion increment = r.getQuaternion();

        orient = increment ^ orient;
        robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2], orient[0],
            orient[1], orient[2], orient[3]);
        break;
      }      

    // Joint Moves
    case DEC_J1:
      robot.SetJoints(curJoints[0] - angStep, curJoints[1], curJoints[2], 
                      curJoints[3], curJoints[4], curJoints[5]);
      break;
    case DEC_J2:
      robot.SetJoints(curJoints[0], curJoints[1] - angStep, curJoints[2], 
                      curJoints[3], curJoints[4], curJoints[5]);
      break;
    case DEC_J3:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2] - angStep, 
                      curJoints[3], curJoints[4], curJoints[5]);
      break;
    case DEC_J4:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2], 
                      curJoints[3] - angStep, curJoints[4], curJoints[5]);
      break;
    case DEC_J5:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2], 
                      curJoints[3], curJoints[4] - angStep, curJoints[5]);
      break;
    case DEC_J6:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2], 
                      curJoints[3], curJoints[4], curJoints[5] - angStep);
      break;
    case INC_J1:
      robot.SetJoints(curJoints[0] + angStep, curJoints[1], curJoints[2], 
                      curJoints[3], curJoints[4], curJoints[5]);
      break;
    case INC_J2:
      robot.SetJoints(curJoints[0], curJoints[1] + angStep, curJoints[2], 
                      curJoints[3], curJoints[4], curJoints[5]);
      break;
    case INC_J3:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2] + angStep, 
                      curJoints[3], curJoints[4], curJoints[5]);
      break;
    case INC_J4:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2], 
                      curJoints[3] + angStep, curJoints[4], curJoints[5]);
      break;
    case INC_J5:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2], 
                      curJoints[3], curJoints[4] + angStep, curJoints[5]);
      break;
    case INC_J6:
      robot.SetJoints(curJoints[0], curJoints[1], curJoints[2], 
                      curJoints[3], curJoints[4], curJoints[5] + angStep);
      break;

    // Align the tool to nearest cartesian orthant
    case ALIGN:
      {
        robot.Stop();
        Quaternion cur_q(curCartesian+3);

        // Find the direction the tool is currently pointing
        Vec dir = (cur_q^Quaternion("0 0 0 1")^(cur_q.conjugate())).getVector();

        // If it's pointing along the z axis already, we're done
        if ((dir[0]*dir[0] + dir[1]*dir[1]) > 0.00001)
        {
          // Now, find the quaternion that rotates the tool to where it's 
          // currently pointing (with no rotation)
          Vec temp = Vec("0 0 1", 3)^dir;
          Vec n_hat = temp / temp.norm();
          double sin_th = temp.norm();
          double cos_th = sqrt(1 - sin_th*sin_th);
          if (temp[2] < 0)
            cos_th *= -1;
          double cos_th2 = sqrt(0.5*(1 + cos_th));
          double sin_th2 = sqrt(0.5*(1 - cos_th));
          Quaternion q_z;
          q_z[0] = cos_th2;
          q_z[1] = sin_th2 * n_hat[0]; 
          q_z[2] = sin_th2 * n_hat[1]; 
          q_z[3] = sin_th2 * n_hat[2]; 

          // Now, express the rotation about the z-axis of the tool 
          // as a quaternion
          Quaternion q_r = (q_z.conjugate()) ^ cur_q;

          // Now, find the closest axis to rotate our tool towards
          // (This is done by finding the largest dot product)
          double max_dot_prod = -1;
          int closest_idx = -1;
          for (int i=0; i<NUM_ALIGN_DIRS; i++)
          {
            double dp = align_vecs[i] * dir;
            if (dp > max_dot_prod)
            {
              max_dot_prod = dp;
              closest_idx = i;
            }
          }

          // Now, our goal orientation is the closest aligned location, 
          // which is then rotated by the current tool rotation
          Quaternion new_q = align_quats[closest_idx] ^ q_r;

          // Set this new position
          robot.SetComm(BLOCKING);
          robot.SetCartesian(curCartesian[0], curCartesian[1], curCartesian[2], 
              new_q[0], new_q[1], new_q[2], new_q[3]);
        }

        robot.SetComm(NON_BLOCKING);
        break;
      }

    // Speed Adjustments
    case DEC_TCP:
      curSpeed[0] -= linStep;
      if (curSpeed[0] < 0)
        curSpeed[0] = 0;
      robot.SetSpeed(curSpeed[0], curSpeed[1]);
      console_update_speed(curSpeed);
      break;
      
    case INC_TCP:
      curSpeed[0] += linStep;
      if (curSpeed[0] < 0)
        curSpeed[0] = 0;
      robot.SetSpeed(curSpeed[0], curSpeed[1]);
      console_update_speed(curSpeed);
      break;
      
    case DEC_ORI:
      curSpeed[1] -= angStep;
      if (curSpeed[1] < 0)
        curSpeed[1] = 0;
      robot.SetSpeed(curSpeed[0], curSpeed[1]);
      console_update_speed(curSpeed);
      break;
      
    case INC_ORI:
      curSpeed[1] += angStep;

      if (curSpeed[1] < 0)
        curSpeed[1] = 0;
      
      robot.SetSpeed(curSpeed[0], curSpeed[1]);
      console_update_speed(curSpeed);
      break;
      
  // Larger motions.       
    case COARSE:
        switch (linStep)
        {
            case LIN_STEP_1:
                linStep = LIN_STEP_2;
                angStep = ANG_STEP_2;
                break;
            case LIN_STEP_2:
                linStep = LIN_STEP_3;
                angStep = ANG_STEP_3;
                break;
            default:
                break;
        }
        console_update_motion_mode(linStep);
        break;
      
  // Smaller Motions.    
    case FINE:
        switch (linStep)
        {
            case LIN_STEP_3:
                linStep = LIN_STEP_2;
                angStep = ANG_STEP_2;
                break;
            case LIN_STEP_2:
                linStep = LIN_STEP_1;
                angStep = ANG_STEP_1;
                break;
            default:
                break;
        }
        console_update_motion_mode(linStep);
        break;

  // Stop the robot and quit the console.
    case QUIT:
      robot.Stop();
      exitProgram = true;
      break;

    // If the user just lifted off of a key, make sure we immediately stop the robot
    case NO_KEY:
      if (last_key != NO_KEY)
        robot.Stop();
      break;
    
    case UNKNOWN_KEY:
      // If the user pressed a random key, let them know
      console_flash();
      break;

    default:
      //Something went wrong
      console_flash();
      break;
  }

  // Remember the last key, so we can erase it when it's no longer pressed.
  last_key = cur_key;
}


///////////////
// MAIN LOOP //
///////////////
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ABBteleoperator");
  ros::NodeHandle node;
  ros::Rate loop_rate(TELEOP_RATE);
  nodePtr = &node;
  int counter = 0;
  RobotComm robot_(&node,"1");
  robot = robot_;
  
  // Initialize the robot, subscribe to services, and create the console
  if (!initialize())
  {
    ROS_WARN("Unable to initialize teleop. Exiting...");
    exitProgram = true;
    exit(0);
  }

  // In the loop we keep track of robot updates and execute user commands
  ros::AsyncSpinner spinner(1);
  spinner.start(); // This handles ros requests in background and asynchronously.
  while (ros::ok() && !exitProgram)
  {
    updateState();
    executeUserCommand();
    loop_rate.sleep();
    //Once a second we check for posible updates on the speed, tool,
    //workbject, and zone of the robot.
    if(counter>TELEOP_RATE)
      {
        updateOther();
        counter=0;
      }
    counter++;
  }
  console_close();
  return 0;
}

