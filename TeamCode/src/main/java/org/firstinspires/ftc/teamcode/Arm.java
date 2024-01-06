package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.State;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Arm extends BlocksOpModeCompanion {
    
    /* Declare our class variables. */

    // Our run states for our arm
    // NOTE:  A target pose must be set for the arm to move.
    static private enum State {
        INIT,
        WAITING_FOR_COMMAND,
        MOVING_TO_FLOOR_POSE, 
        MOVING_TO_TRAVEL_POSE,
        MOVING_TO_BACKDROP_POSE,
        MOVING_TO_UP_POSE,         
        STOP_MOVING,
    }

    static State armState = State.INIT;
    
    // Constructor of arm joints   
    // ArmJoint(String _servoName, double initPosition, double Max, double Min, double Increment)
    static private ArmJoint shoulder = null;
    static private ArmJoint elbow = null; 
    static private ArmJoint wrist = null; 
    
    // Initialize poses: Init, Up, Floor, Travel, Backdrop 
    // Constructor and initialization of poses  
    static private Pose poseInit = null;
    static private Pose poseUp = null;    
    static private Pose poseFloor = null;   
    static private Pose poseTravel = null;   
    static private Pose poseBackdrop = null;   
    
    static private double numberStepsBetweenPoses = 50.0;
    
    // Declare and set our run timer
    static ElapsedTime timerOfARunState = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final int maxRunTime = 30000;   // Max run time for a state is 5 seconds.
    
  @ExportToBlocks (
    heading = "Run Arm Based on State",
    color = 255,
    comment = "Runs arm based on the state.",
    tooltip = "States are changed by calling a pose or stop."
  )
  /**
   * Runs an interation of the drivetrain based on it's current state
   * Running the drive motors in a STATE mode to allow other actions to happen on the robot.
   */
  public static void runArmIteration() {
      
    boolean shoulderMovingToTarget = false;
    boolean elbowMovingToTarget = false;
    boolean wristMovingToTarget = false;
 
    switch (armState) {
      
      case INIT: {       
          
        telemetry.addData("Arm: ", "INIT");        
        break;
      }

      case WAITING_FOR_COMMAND: {
          
        telemetry.addData("Arm: ", "WAITING_FOR_COMMAND");
        break;
      }

      case MOVING_TO_UP_POSE: {
                
        // Check to see if we have timed out on the movement request
        if ( timerOfARunState.milliseconds() > maxRunTime) {
            
            telemetry.addData("Arm: ", "TIMED OUT WHILE  MOVING TO UP POSE");
                    
            // Stop arm motions            
            stopMovement();
            
            // Change our state            
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        } 
        
        // Let's move to target that has already been set
        shoulderMovingToTarget = shoulder.movingtoTarget();
        elbowMovingToTarget = elbow.movingtoTarget(); 
        wristMovingToTarget = wrist.movingtoTarget();

        telemetry.addData("Arm: ", " MOVING TO UP POSE");
        // keeps looping while we are still moving joints to their target position.
        
        telemetry.addData("UP Pose, Shoulder target", poseUp.getPoseShoulderTarget());
        telemetry.addData("UP Pose, Shoulder increment", shoulder.getCurrentIncrement());        
        telemetry.addData("UP Pose, Shoulder current position", shoulder.getCurrentPosition());
        
        telemetry.addData("UP Pose, Elbow target", poseUp.getPoseElbowTarget());
        telemetry.addData("UP Pose, Shoulder increment", elbow.getCurrentIncrement());  
        telemetry.addData("UP Pose, Elbow current position", elbow.getCurrentPosition());        
        
        telemetry.addData("UP Pose, Wrist target",  poseUp.getPoseWristTarget());  
        telemetry.addData("UP Pose, Wrist increment", wrist.getCurrentIncrement());          
        telemetry.addData("UP Pose, Wrist current position", wrist.getCurrentPosition());   
        
        telemetry.update();
        
        // Check to see if all arm joints are done        
        if (! shoulderMovingToTarget && ! elbowMovingToTarget && ! wristMovingToTarget) {
            
            telemetry.addData("Arm: ", "DONE  MOVING TO UP POSE");
            
            // Stop arm motions
            stopMovement();
            
            // Change our state
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        }                  
                
        // Break out of this case of moving to UP pose
        break;
        
      }
      
      case MOVING_TO_FLOOR_POSE: {
                
        // Check to see if we have timed out on the movement request
        if ( timerOfARunState.milliseconds() > maxRunTime) {
            
            telemetry.addData("Arm: ", "TIMED OUT WHILE MOVING TO FLOOR POSE");
                    
            // Stop arm motions            
            stopMovement();
            
            // Change our state            
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        } 
        
        // Let's move to target that has already been set
        shoulderMovingToTarget = shoulder.movingtoTarget();
        elbowMovingToTarget = elbow.movingtoTarget(); 
        wristMovingToTarget = wrist.movingtoTarget();

        telemetry.addData("Arm: ", " MOVING TO FLOOR POSE");
        // keeps looping while we are still moving joints to their target position.
        
        telemetry.addData("FLOOR Pose, Shoulder target", poseFloor.getPoseShoulderTarget());
        telemetry.addData("FLOOR Pose, Shoulder increment", shoulder.getCurrentIncrement());        
        telemetry.addData("FLOOR Pose, Shoulder current position", shoulder.getCurrentPosition());
        
        telemetry.addData("FLOOR Pose, Elbow target", poseFloor.getPoseElbowTarget());
        telemetry.addData("FLOOR Pose, Shoulder increment", elbow.getCurrentIncrement());  
        telemetry.addData("FLOOR Pose, Elbow current position", elbow.getCurrentPosition());        
        
        telemetry.addData("FLOOR Pose, Wrist target",  poseFloor.getPoseWristTarget());  
        telemetry.addData("FLOOR Pose, Wrist increment", wrist.getCurrentIncrement());          
        telemetry.addData("FLOOR Pose, Wrist current position", wrist.getCurrentPosition());   
        
        telemetry.update();
        
        // Check to see if all arm joints are done        
        if (! shoulderMovingToTarget && ! elbowMovingToTarget && ! wristMovingToTarget) {
            
            telemetry.addData("Arm: ", "DONE MOVING TO FLOOR POSE");
            
            // Stop arm motions
            stopMovement();
            
            // Change our state
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        }                  
                
        // Break out of this case of moving to Floor pose
        break;
        
      }
      
      case MOVING_TO_TRAVEL_POSE: {
                
        // Check to see if we have timed out on the movement request
        if ( timerOfARunState.milliseconds() > maxRunTime) {
            
            telemetry.addData("Arm: ", "TIMED OUT WHILE MOVING TRAVEL POSE");
                    
            // Stop arm motions            
            stopMovement();
            
            // Change our state            
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        } 
        
        // Let's move to target that has already been set
        shoulderMovingToTarget = shoulder.movingtoTarget();
        elbowMovingToTarget = elbow.movingtoTarget(); 
        wristMovingToTarget = wrist.movingtoTarget();

        telemetry.addData("Arm: ", "MOVING TRAVEL POSE");
        // keeps looping while we are still moving joints to their target position.
        
        telemetry.addData("TRAVEL Pose, Shoulder target", poseTravel.getPoseShoulderTarget());
        telemetry.addData("TRAVEL Pose, Shoulder increment", shoulder.getCurrentIncrement());        
        telemetry.addData("TRAVEL Pose, Shoulder current position", shoulder.getCurrentPosition());
        
        telemetry.addData("TRAVEL Pose, Elbow target", poseTravel.getPoseElbowTarget());
        telemetry.addData("TRAVEL Pose, Shoulder increment", elbow.getCurrentIncrement());  
        telemetry.addData("TRAVEL Pose, Elbow current position", elbow.getCurrentPosition());        
        
        telemetry.addData("TRAVEL Pose, Wrist target",  poseTravel.getPoseWristTarget());  
        telemetry.addData("TRAVEL Pose, Wrist increment", wrist.getCurrentIncrement());          
        telemetry.addData("TRAVEL Pose, Wrist current position", wrist.getCurrentPosition());   
        
        telemetry.update();
        
        // Check to see if all arm joints are done        
        if (! shoulderMovingToTarget && ! elbowMovingToTarget && ! wristMovingToTarget) {
            
            telemetry.addData("Arm: ", "DONE MOVING TRAVEL POSE");
            
            // Stop arm motions
            stopMovement();
            
            // Change our state
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        }                  
                
        // Break out of this case of moving to Travel pose
        break;
        
      }
      
      case MOVING_TO_BACKDROP_POSE: {
                
        // Check to see if we have timed out on the movement request
        if ( timerOfARunState.milliseconds() > maxRunTime) {
            
            telemetry.addData("Arm: ", "TIMED OUT WHILE MOVING BACKDROP POSE");
                    
            // Stop arm motions            
            stopMovement();
            
            // Change our state            
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        } 
        
        // Let's move to target that has already been set
        shoulderMovingToTarget = shoulder.movingtoTarget();
        elbowMovingToTarget = elbow.movingtoTarget(); 
        wristMovingToTarget = wrist.movingtoTarget();

        telemetry.addData("Arm: ", "MOVING BACKDROP POSE");
        // keeps looping while we are still moving joints to their target position.
        
        telemetry.addData("BACKDROP Pose, Shoulder target", poseBackdrop.getPoseShoulderTarget());
        telemetry.addData("BACKDROP Pose, Shoulder increment", shoulder.getCurrentIncrement());        
        telemetry.addData("BACKDROP Pose, Shoulder current position", shoulder.getCurrentPosition());
        
        telemetry.addData("BACKDROP Pose, Elbow target", poseBackdrop.getPoseElbowTarget());
        telemetry.addData("BACKDROP Pose, Shoulder increment", elbow.getCurrentIncrement());  
        telemetry.addData("BACKDROP Pose, Elbow current position", elbow.getCurrentPosition());        
        
        telemetry.addData("BACKDROP Pose, Wrist target",  poseBackdrop.getPoseWristTarget());  
        telemetry.addData("BACKDROP Pose, Wrist increment", wrist.getCurrentIncrement());          
        telemetry.addData("BACKDROP Pose, Wrist current position", wrist.getCurrentPosition());   
        
        telemetry.update();
        
        // Check to see if all arm joints are done        
        if (! shoulderMovingToTarget && ! elbowMovingToTarget && ! wristMovingToTarget) {
            
            telemetry.addData("Arm: ", "DONE MOVING BACKDROP POSE");
            
            // Stop arm motions
            stopMovement();
            
            // Change our state
            armState = State.WAITING_FOR_COMMAND;
            
            break;
        }                  
                
        // Break out of this case of moving to Backdrop pose
        break;
        
      }
      
     }
      
    }


    @ExportToBlocks (
        heading = "Arm",
        color = 255,
        comment = "Returns whether we are waiting for a command.",
        tooltip = "If waiting a long time, then check the maxRunTime setting."
    )
    /** Returns whether we are waiting for a command.
     */
    public static boolean waitingForCommand() {
        
        if (armState == State.WAITING_FOR_COMMAND) 
            return true;
        else    
            return false;
    
    }  // end method waitingForCommand()

    @ExportToBlocks (
        heading = "Arm: Initialize",
        color = 255,
        comment = "Initialize Arm and Poses",
        tooltip = "Initialize Arm and Poses",
        parameterLabels = {"Shoulder Motor Name",
                           "Elbow Motor Name",
                           "Wrist Motor Name"              
        }
    )
    /** Initialize arm and poses
     */
    static public void initArm(String _shoulderName, String _elbowName, String _wristName) {
       
        // Constructor of arm joints   
        // ArmJoint(String _servoName, double initPosition, double Min, double Max, double Increment)
        shoulder = new ArmJoint(_shoulderName, hardwareMap.get(Servo.class, _shoulderName), 0.25, 0.25, 0.8, 0.001); 
        elbow = new ArmJoint(_elbowName,hardwareMap.get(Servo.class, _elbowName), 1, 0, 1, 0.001); 
        wrist = new ArmJoint(_wristName, hardwareMap.get(Servo.class, _wristName), 1, 0, 1, 0.001); 
    
        // Initialize all of our servos on the arm.  DOES NOT include the gripper.
        shoulder.init();       
        elbow.init();       
        wrist.init();    
        
        // Initialize poses: Floor, Travel, Backdrop 
        // Constructor and initialization of pose  
        // public Pose(String _name, double _shoulder, double _elbow, double _wrist)        
        poseInit = new Pose("Initial", 0.25, 1.0, 1.0);
        poseUp = new Pose("Up", 0.5, 0.5, 0.5);    
        poseFloor = new Pose("Floor", 0.521, 0.038, 0.632);    
        poseTravel = new Pose("Travel", 0.26, 0.998, 0.551);    
        poseBackdrop = new Pose("Backdrop", 0.529, 0.587, 0.258);    
    
        // Ready to receive commands
        armState = State.WAITING_FOR_COMMAND;    
    
    }  // end method initArm()
     
    
    @ExportToBlocks (
        heading = "Arm: Move To UP Pose",
        color = 255,
        comment = "Moves the arm to an UP pose.",
        tooltip = "Stop by quitting program.",
        parameterLabels = {}
    )
    /**
    */
    static public void moveToPose_UP() {
        
        // Set arm joints to pose target positions
        shoulder.setTargetPosition(poseUp.getPoseShoulderTarget(), numberStepsBetweenPoses); 
        elbow.setTargetPosition(poseUp.getPoseElbowTarget(), numberStepsBetweenPoses); 
        wrist.setTargetPosition(poseUp.getPoseWristTarget(), numberStepsBetweenPoses); 
        
        // Start moving shoulder joint, and then enter the control loop
        shoulder.movingtoTarget();

        // Set the state that was requested
        armState = State.MOVING_TO_UP_POSE;
        timerOfARunState.reset();
        
        //  Our RunArmIteration() takes over from here.  Put if in your main loop.
        
    }   // end method moveToPose_UP()

    
    @ExportToBlocks (
        heading = "Arm: Move To FLOOR Pose",
        color = 255,
        comment = "Moves the arm to an FLOOR pose.",
        tooltip = "Stop by quitting program.",
        parameterLabels = {}
    )
    /**
    */
    static public void moveToPose_FLOOR() {
        
        // Set arm joints to pose target positions
        shoulder.setTargetPosition(poseFloor.getPoseShoulderTarget(), numberStepsBetweenPoses); 
        elbow.setTargetPosition(poseFloor.getPoseElbowTarget(), numberStepsBetweenPoses); 
        wrist.setTargetPosition(poseFloor.getPoseWristTarget(), numberStepsBetweenPoses); 
        
        // Start moving shoulder joint, and then enter the control loop
        shoulder.movingtoTarget();

        // Set the state that was requested
        armState = State.MOVING_TO_FLOOR_POSE;
        timerOfARunState.reset();
        
        //  Our RunArmIteration() takes over from here. Put if in your main loop.
        
    }   // end method moveToPose_FLOOR()

    
    @ExportToBlocks (
        heading = "Arm: Move To TRAVEL Pose",
        color = 255,
        comment = "Moves the arm to an TRAVEL pose.",
        tooltip = "Stop by quitting program.",
        parameterLabels = {}
    )
    /**
    */
    static public void moveToPose_TRAVEL() {
        
        // Set arm joints to pose target positions
        shoulder.setTargetPosition(poseTravel.getPoseShoulderTarget(), numberStepsBetweenPoses); 
        elbow.setTargetPosition(poseTravel.getPoseElbowTarget(), numberStepsBetweenPoses); 
        wrist.setTargetPosition(poseTravel.getPoseWristTarget(), numberStepsBetweenPoses); 
        
        // Start moving shoulder joint, and then enter the control loop
        shoulder.movingtoTarget();

        // Set the state that was requested
        armState = State.MOVING_TO_TRAVEL_POSE;
        timerOfARunState.reset();
        
        //  Our RunArmIteration() takes over from here. Put if in your main loop.
        
    }   // end method moveToPose_TRAVEL()

    
    @ExportToBlocks (
        heading = "Arm: Move To BACKDROP Pose",
        color = 255,
        comment = "Moves the arm to an BACKDROP pose.",
        tooltip = "Stop by quitting program.",
        parameterLabels = {}
    )
    /**
    */
    static public void moveToPose_BACKDROP() {
        
        // Set arm joints to pose target positions
        shoulder.setTargetPosition(poseBackdrop.getPoseShoulderTarget(), numberStepsBetweenPoses); 
        elbow.setTargetPosition(poseBackdrop.getPoseElbowTarget(), numberStepsBetweenPoses); 
        wrist.setTargetPosition(poseBackdrop.getPoseWristTarget(), numberStepsBetweenPoses); 
        
        // Start moving shoulder joint, and then enter the control loop
        shoulder.movingtoTarget();

        // Set the state that was requested
        armState = State.MOVING_TO_BACKDROP_POSE;
        timerOfARunState.reset();
        
        //  Our RunArmIteration() takes over from here. Put if in your main loop.
        
    }   // end method moveToPose_BACKDROP()

    @ExportToBlocks (
        heading = "Arm: Stop Movement",
        color = 255,
        comment = "Stops movement of arm",
        tooltip = "If current position will cause further damage then STOP PROGRAM.",
        parameterLabels = {}
    )
    /**
     * Stops all arm movement
     */
    static public void stopMovement() {
        
        // Stop all motion & sets to WAITING_FOR_COMMAND
        shoulder.setTargetPosition(shoulder.getCurrentPosition(), 1.0);
        shoulder.movingtoTarget();
        
        elbow.setTargetPosition(elbow.getCurrentPosition(), 1.0);
        elbow.movingtoTarget();
        
        wrist.setTargetPosition(wrist.getCurrentPosition(), 1.0);
        wrist.movingtoTarget();
        
        armState = State.WAITING_FOR_COMMAND;
        
        telemetry.addData("Arm: ", "SHOULD NOT BE MOVING");
        
    }   // end method stopMovement()

    @ExportToBlocks (
        heading = "Arm: Shoulder By Increment",
        color = 255,
        comment = "Moves the shoulder by the increment requested",
        tooltip = "Should be checking for min / max positions.",
        parameterLabels = {"Negative or positive increment"}
    )
    /**
     * Moves the shoulder by an increment
     */
    static public void moveShoulderByIncrement(double _increment) {
        
        // Moves joint by increment & sets to WAITING_FOR_COMMAND
        shoulder.moveByIncrement(_increment);
        
        armState = State.WAITING_FOR_COMMAND;
        
        telemetry.addData("Arm: ", "Moving shoulder by an increment");
        
    }   // end method moveShoulderByIncrement()


    @ExportToBlocks (
        heading = "Arm: Elbow By Increment",
        color = 255,
        comment = "Moves the elbow by the increment requested",
        tooltip = "Should be checking for min / max positions.",
        parameterLabels = {"Negative or positive increment"}
    )
    /**
     * Moves the elbow by an increment
     */
    static public void moveElbowByIncrement(double _increment) {
        
        // Moves joint by increment & sets to WAITING_FOR_COMMAND
        elbow.moveByIncrement(_increment);
        
        armState = State.WAITING_FOR_COMMAND;
        
        telemetry.addData("Arm: ", "Moving elbow by an increment");
        
    }   // end method moveElbowByIncrement()


    @ExportToBlocks (
        heading = "Arm: Wrist By Increment",
        color = 255,
        comment = "Moves the wrist by the increment requested",
        tooltip = "Should be checking for min / max positions.",
        parameterLabels = {"Negative or positive increment"}
    )
    /**
     * Moves the wrist by an increment
     */
    static public void moveWristByIncrement(double _increment) {
        
        // Moves joint by increment & sets to WAITING_FOR_COMMAND
        wrist.moveByIncrement(_increment);
        
        armState = State.WAITING_FOR_COMMAND;
        
        telemetry.addData("Arm: ", "Moving wrist by an increment");
        
    }   // end method moveWristByIncrement()

    @ExportToBlocks (
        heading = "Arm: Release Arm Joints",
        color = 255,
        comment = "Releases each of the arm joints from a position",
        tooltip = "This will make the arm collapse."
    )
    /**
     * Releases all the arm joints
     **/
    static public void releaseArmJoints() {
        
        // Moves joint by increment & sets to WAITING_FOR_COMMAND
        wrist.release();
        
        armState = State.WAITING_FOR_COMMAND;
        
        telemetry.addData("Arm: ", "Releasing all the arm joints");
        
    }   // end method releaseArmJoints()

}
