package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.State;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
          
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.LED;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class DrivetrainMecanumWithSmarts extends BlocksOpModeCompanion {

    /* Declare our class variables. */
    static private DcMotor  driveLeftFrontHW   = null;
    static private DcMotor  driveRightBackHW   = null;
    
    static private DcMotor  driveRightFrontHW  = null;
    static private DcMotor  driveLeftBackHW    = null;
    
    // Variables for the hardware configuration names of our drive motors
    static private String _driveLeftFrontName, _driveLeftBackName, _driveRightFrontName, _driveRightBackName;
    
    // Initialize our speed levels
    // powerLevel: Range 0-1.00
    static private double[] powerLevel = {0.15, 0.50};    

    // Define IMU
    static IMU imu; 
    static YawPitchRollAngles orientation;
    static AngularVelocity angularVelocity;
    static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    static RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    static private double          robotHeading  = 0;
    static private double          headingOffset = 0;
    static private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    static private double  targetHeading    = 0;
    static private double  targetDistance   = 0;
    static private double  driveSpeed       = 0;
    static private double  maxDriveSpeed    = 0;
    static private double  maxTurnSpeed     = 0;
    static private double  turnSpeed        = 0;
    static private double  speedLeftFront   = 0;
    static private double  speedRightBack   = 0;
    static private double  speedRightFront  = 0;
    static private double  speedLeftBack    = 0;
    static private int     targetLeftFront  = 0;
    static private int     targetRightBack  = 0;
    static private int     targetRightFront = 0;
    static private int     targetLeftBack   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 280.0 ;   // 560 REV HD HEX 20:1 Planetary 300 RPM 28x20 (eg: 537.7 for GoBILDA 312 RPM Yellow Jacket)
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 2.95 ;     // For figuring circumference;  75 mm REV mecanum wheels
    static final double     MECANUM_FACTOR_FORWARD = 1.0;     // Adjustment for theoretical calculations for FORWARD
    static final double     MECANUM_FACTOR_BACKWARD = 1.0;     // Adjustment for theoretical calculations for BACKWARD
    static final double     MECANUM_FACTOR_LEFT    = 1.2;     // Adjustment for theoretical calculations for LEFT 
    static final double     MECANUM_FACTOR_RIGHT    = 1.2;     // Adjustment for theoretical calculations for RIGHT 
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // Run states for our drivetrain
    // NOTE:  A target level must be set for the lift to raise or lower
    static private enum State {
      INIT,
      WAITING_FOR_COMMAND,
      DRIVING_STRAIGHT, 
      DRIVING_LEFT,
      TURNING_TO_HEADING,
      HOLDING_HEADING,
      MOVE_TO_OBJECT,
      MOVE_TOWARDS_OBJECT,
      MOVE_AWAY_FROM_OBJECT
    }
  
    static State drivetrainState = State.WAITING_FOR_COMMAND;
    static ElapsedTime timerOfARunState = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final int maxRunTime = 5000;   // Max run time for a state is 5 seconds.
    static ElapsedTime timerHoldHeading = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static int timeToHoldHeading = 0;
    
    private static DistanceSensor sensor2MDistance;
    static double distanceCurrent;
    static int distanceObjectSeen = 0;
    static int distanceFromObject = 170;
    static int distanceToClearObject = 300;
    
    
  @ExportToBlocks (
    heading = "Run Drivetrain Based on State",
    color = 255,
    comment = "Runs drivetrain based on the state.",
    tooltip = "States are changed by calling a movement or heading."
  )
  /**
   * Runs an interation of the drivetrain based on it's current state
   * Running the drive motors in a STATE mode to allow other actions to happen on the robot.
   */
  public static void runDrivetrainIteration() {
 
    switch (drivetrainState) {
      
      case INIT: {       
          
        telemetry.addData("Movement: ", "INIT");        
        break;
      }

      case WAITING_FOR_COMMAND: {
          
        telemetry.addData("Movement: ", "WAITING_FOR_COMMAND");
        break;
      }

      case DRIVING_STRAIGHT: {
                
        telemetry.addData("Movement: ", "DRIVING STRAIGHT");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime && 
           (driveLeftFrontHW.isBusy() && driveRightBackHW.isBusy() && driveRightFrontHW.isBusy() && driveLeftBackHW.isBusy())) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(targetHeading, maxDriveSpeed, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (targetDistance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

      case DRIVING_LEFT: {
                
        telemetry.addData("Movement: ", "DRIVING SIDEWAYS");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime &&
           (driveLeftFrontHW.isBusy() && driveRightBackHW.isBusy() && driveRightFrontHW.isBusy() && driveLeftBackHW.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(targetHeading, maxDriveSpeed, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (targetDistance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveLeft(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
        } else {    
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }

        break;
      }

      case TURNING_TO_HEADING: {
                
        telemetry.addData("Movement: ", "TURNING TO HEADING");
        
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(targetHeading, maxTurnSpeed, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        if (timerOfARunState.milliseconds() < maxRunTime && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(targetHeading, maxTurnSpeed, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
            
        } else {
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }

        break;
      }

      case HOLDING_HEADING: {
                
        telemetry.addData("Movement: ", "HOLDING HEADING");

        // keep looping while we have time remaining.
        if (timerOfARunState.milliseconds() < maxRunTime && timerHoldHeading.milliseconds() < timeToHoldHeading) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(targetHeading, maxTurnSpeed, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
            
        } else {
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }

        break;
      }
      
      case MOVE_TO_OBJECT: {
                
        telemetry.addData("Movement: ", "MOVING FORWARD TO AN OBJECT");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime && distanceCurrent > distanceObjectSeen) {

            // Grab the latest distance reading at the bottom of the lift to see if anything there
            distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

            telemetry.addData("Current distance: ", distanceCurrent);
            telemetry.addData("Look for object at distance: ", distanceObjectSeen);

            // Start moving the robot
            moveRobot(driveSpeed, turnSpeed); 

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

      case MOVE_TOWARDS_OBJECT: {
                
        telemetry.addData("Movement: ", "MOVING TOWARDS OBJECT");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime && distanceCurrent > distanceFromObject) {

            // Grab the latest distance reading at the bottom of the lift to see if anything there
            distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

            telemetry.addData("Current distance: ", distanceCurrent);
            telemetry.addData("Look for object at distance: ", distanceFromObject);

            // Start moving the robot
            moveLeft(driveSpeed, turnSpeed); 

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

      case MOVE_AWAY_FROM_OBJECT: {
                
        telemetry.addData("Movement: ", "MOVING AWAY FROM OBJECT");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime && distanceCurrent < distanceToClearObject) {

            // Grab the latest distance reading at the bottom of the lift to see if anything there
            distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

            telemetry.addData("Current distance: ", distanceCurrent);
            telemetry.addData("Look for object at distance: ", distanceToClearObject);

            // Start moving the robot
            moveLeft(driveSpeed, turnSpeed); 

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

    }

  }

    @ExportToBlocks (
        heading = "Drivetrain",
        color = 255,
        comment = "Returns whether we are waiting for a command.",
        tooltip = "If waiting a long time, then check the maxRunTime setting."
    )
    /** Returns whether we are waiting for a command.
     */
    public static boolean waitingForCommand() {
        if (drivetrainState == State.WAITING_FOR_COMMAND) 
            return true;
        else    
            return false;
    
    }  // end method waitingForCommand()

    @ExportToBlocks (
        heading = "Drivetrain",
        color = 255,
        comment = "Initialize Drivetrain and Pose",
        tooltip = "Initialize Drivetrain and Pose",
        parameterLabels = {"Left Front Drive Motor Name",
                           "Left Back Drive Motor Name",
                           "Right Front Drive Motor Name",
                           "Right Back Drive Motor Name",
                           "2M Distance Sensor"
        }
    )
    /** Initialize drivetrain and pose
     */
    static public void initDriveTrainWithSmarts(String driveLeftFrontName, String driveLeftBackName, String driveRightFrontName, String driveRightBackName, String sensor2MDistanceName) {

       // Save the hardware configuration names of our drive motors
       _driveLeftFrontName = driveLeftFrontName;
       _driveLeftBackName = driveLeftBackName;
       _driveRightFrontName = driveRightFrontName;
       _driveRightBackName = driveRightBackName;
       
       // Initialize to Config A where our Floor pose is at the Front
       // Set our drive to the default A configuration
       setDriveToAConfig();       
       
       // Initialize our drive train motors
       initDrivetrainMotors();
       
       // Initialize our IMU for pose information
       initIMU();
       telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
       
       // Set the encoders for closed loop speed control, and reset the heading.
       driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       driveLeftBackHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       driveRightFrontHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       driveRightBackHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       
           
       // Let's get a hardware handle on our distance sensor at the bottom of our lift
       sensor2MDistance = hardwareMap.get(DistanceSensor.class, sensor2MDistanceName);

       resetHeading();

    }  // end method initDriveTrain()
     
    // **********  HIGH Level driving functions.  ********************
    
    @ExportToBlocks (
        heading = "Drivetrain: Smarts",
        color = 255,
        comment = "Drives Straight either Forward or Reverse.",
        tooltip = "Reverse movement is obtained by setting a negative distance (not speed).",
        parameterLabels = {"Max Drive Speed", "Distance", "Heading"}
    )
    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param requestedDistance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param requestedHeading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    static public void driveStraight(double requestedMaxDriveSpeed,
                              double requestedDistance,
                              double requestedHeading) {
        
        // Friction adjustment based on direction for mecanum drive
        double adjustmentMecanum = 1.0;    
        targetDistance = requestedDistance;
        targetHeading = requestedHeading;
        
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        maxDriveSpeed = Math.abs(requestedMaxDriveSpeed);
        
        // if driving in reverse, the motor correction also needs to be reversed
        if (targetDistance < 0)
            adjustmentMecanum = MECANUM_FACTOR_BACKWARD;
        else
            adjustmentMecanum = MECANUM_FACTOR_FORWARD;        

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode

        // Resetting encoders for all drive motors
        driveLeftFrontHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBackHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightFrontHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBackHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setting motors to brake if no power
        driveLeftFrontHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftBackHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightFrontHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightBackHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(adjustmentMecanum * targetDistance * COUNTS_PER_INCH);
        targetLeftFront = driveLeftFrontHW.getCurrentPosition() + moveCounts;
        targetRightBack = driveRightBackHW.getCurrentPosition() + moveCounts;
        targetLeftBack = driveLeftBackHW.getCurrentPosition() + moveCounts;            
        targetRightFront = driveRightFrontHW.getCurrentPosition() + moveCounts;


        // Set Target FIRST, then turn on RUN_TO_POSITION
        driveLeftFrontHW.setTargetPosition(targetLeftFront);
        driveRightBackHW.setTargetPosition(targetRightBack);

        driveRightFrontHW.setTargetPosition(targetRightFront);
        driveLeftBackHW.setTargetPosition(targetLeftBack);

        // Now RUN_TO_POSITION
        driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        driveRightFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeftBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving straight, and then enter the control loop
        moveRobot(maxDriveSpeed, 0);

        // Set the state that was requested
        drivetrainState = State.DRIVING_STRAIGHT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.
        
    }   // end method driveStraight()

    @ExportToBlocks (
        heading = "Drivetrain: Smarts",
        color = 255,
        comment = "Drives Left or Right.",
        tooltip = "Right movement is obtained by setting a negative distance (not speed).",
        parameterLabels = {"Max Drive Speed", "Distance", "Heading"}
    )
    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param requestedDistance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param requestedHeading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    static public void driveLeft(double requestedMaxDriveSpeed,
                              double requestedDistance,
                              double requestedHeading) {

        // Friction adjustment based on direction for mecanum drive
        double adjustmentMecanum = 1.0;               
        
        targetDistance = requestedDistance;
        targetHeading = requestedHeading;
        
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        maxDriveSpeed = Math.abs(requestedMaxDriveSpeed);
        
        // if driving in reverse, the motor correction also needs to be reversed
        if (targetDistance < 0)
            adjustmentMecanum = MECANUM_FACTOR_RIGHT;
        else
            adjustmentMecanum = MECANUM_FACTOR_LEFT;        

        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(adjustmentMecanum * Math.abs(targetDistance) * COUNTS_PER_INCH);
            
        if (targetDistance > 0) {
            targetLeftFront = driveLeftFrontHW.getCurrentPosition() - moveCounts;
            targetRightBack = driveRightBackHW.getCurrentPosition() - moveCounts;
            
            targetRightFront = driveRightFrontHW.getCurrentPosition() + moveCounts;
            targetLeftBack = driveLeftBackHW.getCurrentPosition() + moveCounts;
              
        } else {
                
            targetLeftFront = driveLeftFrontHW.getCurrentPosition() + moveCounts;
            targetRightBack = driveRightBackHW.getCurrentPosition() + moveCounts;
            
            targetRightFront = driveRightFrontHW.getCurrentPosition() - moveCounts;
            targetLeftBack = driveLeftBackHW.getCurrentPosition() - moveCounts;
               
        }

        // Set Target FIRST, then turn on RUN_TO_POSITION
        driveLeftFrontHW.setTargetPosition(targetLeftFront);
        driveRightBackHW.setTargetPosition(targetRightBack);

        driveRightFrontHW.setTargetPosition(targetRightFront);
        driveLeftBackHW.setTargetPosition(targetLeftBack);

        // Now RUN_TO_POSITION
        driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        driveRightFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeftBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving straight, and then enter the control loop
        moveLeft(maxDriveSpeed, 0);
        
        // Set the state that was requested
        drivetrainState = State.DRIVING_LEFT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method driveLeft()
    
    @ExportToBlocks (
        heading = "Drivetrain: Smarts",
        color = 255,
        comment = "Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.",
        tooltip = "0 = fwd. +ve is CCW from fwd. -ve is CW from forward.",
        parameterLabels = {"Max Turn Speed", "Heading"}
    )
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param requestedTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    static public void turnToHeading(double requestedTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, requestedTurnSpeed, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        if ((Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Set the state that was requested
            drivetrainState = State.TURNING_TO_HEADING;
            timerOfARunState.reset();
            
        } else {
            // Not within the heading threshold, so not going to set to action
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        //  Our RunDrivetrainIteration() takes over from here.
    }

    @ExportToBlocks (
        heading = "Drivetrain: Smarts",
        color = 255,
        comment = "Obtain & hold a heading for a finite amount of time.",
        tooltip = "This function is useful for giving the robot a moment to stabilize it's heading between movements.",
        parameterLabels = {"Max Turn Speed", "Heading", "Hold Time"}
    )
    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in milliseconds) to hold the specified heading.
     */
    static public void holdHeading(double requestedTurnSpeed, double heading, int holdTime) {

        timeToHoldHeading = holdTime;
        timerHoldHeading.reset();

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, requestedTurnSpeed, P_DRIVE_GAIN);
        
        if (timerHoldHeading.milliseconds() < timeToHoldHeading) {

            // Set the state that was requested
            drivetrainState = State.HOLDING_HEADING;
            timerOfARunState.reset();
            
        } else {
            // Not within the heading threshold, so not going to set to action
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        
    }

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Aligns robot to an object",
        tooltip = "Robot can see the object. Pass true if moving forward",
        parameterLabels = {"Are we moving forward?", "Distance Object is Seen"}
    )
    /**
     * Moves the robot forward backward to align with the closest object
     */
    public static void moveToObject(boolean movingForward, int distanceSeen) {
        
        distanceObjectSeen = distanceSeen;
        
        // Set the required driving speed 
        if (movingForward)
            driveSpeed = powerLevel[0];
        else
            driveSpeed = -powerLevel[0];
        
        turnSpeed = 0.0;
        
        // Grab the latest distance reading at the bottom of the lift to see if anything there
        distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

        telemetry.addData("Current distance: ", distanceCurrent);
        telemetry.addData("Distance When Object Seen: ", distanceObjectSeen);
        
        // Start moving the robot
        moveRobot(driveSpeed, turnSpeed); 

        // Set the state that was requested
        drivetrainState = State.MOVE_TO_OBJECT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method moveToObject()


    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Moves robot towards an object.",
        tooltip = "Will stop when at distance provided. Pass true if moving forward",
        parameterLabels = {"Are we moving left", "Distance From Object To Stop"}
    )
    /**
     * Moves the robot left or right towards an object
     */
    public static void moveTowardsObject(boolean movingLeft, int distanceToStop) {
        
        distanceFromObject = distanceToStop;
        
        // Set the required driving speed 
        if (movingLeft)
            driveSpeed = powerLevel[0];
        else
            driveSpeed = -powerLevel[0];
        
        turnSpeed = 0.0;
        
        // Grab the latest distance reading at the bottom of the lift to see if anything there
        distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

        telemetry.addData("Current distance: ", distanceCurrent);
        telemetry.addData("Distance To Stop from Object: ", distanceFromObject);

        // Start moving the robot
        //moveLeftWOPositionAuto(driveSpeed); 

        // Set the state that was requested
        drivetrainState = State.MOVE_TOWARDS_OBJECT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method moveTowardsObject()


    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Moves back away from an object",
        tooltip = "Robot can see the object. Pass true if moving left",
        parameterLabels = {"Are we moving left", "Distance To Clear Object"}
    )
    /**
     * Moves the robot away from an object
     */
    public static void moveAwayFromObject(boolean movingLeft, int distanceToClear) {
        
        distanceToClearObject = distanceToClear;
        
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        if (movingLeft)
            driveSpeed = powerLevel[0];
        else
            driveSpeed = -powerLevel[0];
        
        turnSpeed = 0.0;
        
        // Grab the latest distance reading at the bottom of the lift to see if anything there
        distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

        telemetry.addData("Current distance: ", distanceCurrent);
        telemetry.addData("Current distance: ", distanceToClearObject);

        // Start moving the robot
        //moveLeftWOPositionAuto(driveSpeed); 

        // Set the state that was requested
        drivetrainState = State.MOVE_AWAY_FROM_OBJECT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method moveAwayFromObject()


    // **********  LOW Level initialization functions.  ********************

    /** Initialize variables for our drivetrain motors:
     *    > Current speed
     *    > Set next power threshold
     *    > Motor direction
     *    > Resetting encoder values
     *    > Set run mode of motors to not use encoders
     *    > Set motors to brake if no power
     */
    static private void initDrivetrainMotors() {
        
        // All drive motor handles below were set in the setDriveToXConfig() method
        
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode

        // Resetting encoders for all drive motors
        driveLeftFrontHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBackHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightFrontHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBackHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setting motors to brake if no power
        driveLeftFrontHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftBackHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightFrontHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightBackHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }  // end method initDrivetrainMotors()

    /** Initialize IMU and its parameters:
     *    > Angles
     *    > Gravity
     */
    static private void initIMU() {

        imu = hardwareMap.get(IMU.class, "imu");
        /* The next two lines define Hub orientation.
        * See example SensorIMUOrthogonal.java
        */
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);      
        
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    
    }  // end method initIMU()

    @ExportToBlocks (
        heading = "Drivetrain: Configurations",
        color = 255,
        comment = "Set drive to A Config.",
        tooltip = "A Config is where the front / forward is where the gripper is on the Floor pose."
    )
    // **********  Drivetrain Configuration  ********************
    /** Set our drive movements to either an A or B layout
     *  Where A relates to our Floor gripper position as the front / forward
     *  This function is for the A layout and is the default
     */
    static public void setDriveToAConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFrontHW = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveLeftBackHW = hardwareMap.get(DcMotor.class, _driveLeftBackName);
        driveRightBackHW = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveRightFrontHW = hardwareMap.get(DcMotor.class, _driveRightFrontName);

        // Reverse direction for our physically inverted motors
        driveLeftFrontHW.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeftBackHW.setDirection(DcMotorSimple.Direction.REVERSE);

        // Explicitly set forward direction for our other motors
        driveRightFrontHW.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRightBackHW.setDirection(DcMotorSimple.Direction.FORWARD);
        
    } // end method setDriveAConfig()
    
    @ExportToBlocks (
        heading = "Drivetrain: Configurations",
        color = 255,
        comment = "Set drive to B Config.",
        tooltip = "B Config is where the front / forward is backdrop directional pose."
    )
    // **********  Drivetrain Configuration  ********************
    /** Set our drive movements to either an A or B layout
     *  Where A relates to our Floor gripper position as the front / forward
     *  This function is for the A layout and is the default
     */
    static public void setDriveToBConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFrontHW = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveLeftBackHW = hardwareMap.get(DcMotor.class, _driveRightFrontName);
        driveRightBackHW = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveRightFrontHW = hardwareMap.get(DcMotor.class, _driveLeftBackName);

        // Set the direction appropriately for expected movements
        driveLeftBackHW.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightBackHW.setDirection(DcMotorSimple.Direction.FORWARD);

        driveRightFrontHW.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeftFrontHW.setDirection(DcMotorSimple.Direction.REVERSE);
        
    } // end method setDriveToBConfig()
    
    @ExportToBlocks (
        heading = "Drivetrain: Movement with provided XYT",
        color = 255,
        comment = "Moves based on axial(Y,forward/backward), lateral(X, side-to-side) and yaw (turning)",
        tooltip = "Robot can see the object. Pass true if moving left",
        parameterLabels = {"Axial (forward or backward)", "Lateral (left or right)", "Yaw (turning)"}
    )
    /**
     * A function to use in teleop to pass the gamepad values and drive
     */
    public static void DriveXYT(double axial, double lateral, double yaw) {
        double max;

        // Combine the requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        speedLeftFront = axial + lateral + yaw;
        speedRightFront = (axial - lateral) - yaw;
        speedLeftBack = (axial - lateral) + yaw;
        speedRightBack = (axial + lateral) - yaw;
        
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(speedLeftFront), Math.abs(speedRightFront), Math.abs(speedLeftBack), Math.abs(speedRightBack)));
        if (max > 1) {
            speedLeftFront = speedLeftFront / max;
            speedRightFront = speedRightFront / max;
            speedLeftBack = speedLeftBack / max;
            speedRightBack  = speedRightBack  / max;
        }
        
        // Send calculated power to wheels.
        driveLeftFrontHW.setPower(speedLeftFront);
        driveRightFrontHW.setPower(speedRightFront);
        driveLeftBackHW.setPower(speedLeftBack);
        driveRightBackHW.setPower(speedRightBack);

    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Stops all drivetrain movement
     */
    static public void stopDrivetrain() {
        
        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightBackHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        driveRightFrontHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLeftBackHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("Movement: ", "NOT MOVING");
        
    }   // end method stopDrivetrain()


    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param desiredTurnSpeed      Turning power needed to get to required heading. 
	 * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     */
    static public double getSteeringCorrection(double desiredHeading, double desiredTurnSpeed, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        maxTurnSpeed = desiredTurnSpeed; 

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    static public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        speedLeftFront  = drive - turn;
        speedRightBack  = drive - turn;

        speedRightFront = drive + turn;
        speedLeftBack = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(speedLeftFront), Math.abs(speedRightBack)), Math.max(Math.abs(speedRightFront), Math.abs(speedLeftBack)));
        if (max > 1.0)
        {
            speedLeftFront /= max;
            speedRightBack /= max;
            
            speedRightFront /= max;
            speedLeftBack /= max;
        }

        driveLeftFrontHW.setPower(speedLeftFront);
        driveRightBackHW.setPower(speedRightBack);

        driveRightFrontHW.setPower(speedRightFront);
        driveLeftBackHW.setPower(speedLeftBack);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    //  ADDED FOR MECANUM By Coach Breton
    static public void moveLeft(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        speedLeftFront  = drive + turn;
        speedRightBack  = drive + turn;

        speedRightFront = drive - turn;
        speedLeftBack = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(speedLeftFront), Math.abs(speedRightBack)), Math.max(Math.abs(speedRightFront), Math.abs(speedLeftBack)));
        if (max > 1.0)
        {
            speedLeftFront /= max;
            speedRightBack /= max;
            
            speedRightFront /= max;
            speedLeftBack /= max;
        }

        driveLeftFrontHW.setPower(speedLeftFront);
        driveRightBackHW.setPower(speedRightBack);

        driveRightFrontHW.setPower(speedRightFront);
        driveLeftBackHW.setPower(speedLeftBack);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    //  ADDED FOR MECANUM By Coach Breton
    static public void turnRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        // Turning by setting same speed for wheels on the same sides
        speedLeftFront  = drive - turn;
        speedLeftBack  = drive - turn;

        speedRightFront = drive + turn;
        speedRightBack = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(speedLeftFront), Math.abs(speedRightBack)), Math.max(Math.abs(speedRightFront), Math.abs(speedLeftBack)));
        if (max > 1.0)
        {
            speedLeftFront /= max;
            speedRightBack /= max;
            
            speedRightFront /= max;
            speedLeftBack /= max;
        }

        // Turning by setting same speed for wheels on the same sides
        driveLeftFrontHW.setPower(speedLeftFront);
        driveLeftBackHW.setPower(speedLeftBack);

        driveRightFrontHW.setPower(speedRightFront);
        driveRightBackHW.setPower(speedRightBack);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    static private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos LF:RB ,  RF:LB",  "%7d:%7d  ,  %7d:%7d", targetLeftFront, targetRightBack, targetRightFront, targetLeftBack);
            telemetry.addData("Actual Pos LF:RB ,  RF:LB",  "%7d:%7d  ,  %7d:%7d",  
                               driveLeftFrontHW.getCurrentPosition(), driveRightBackHW.getCurrentPosition(),
                               driveRightFrontHW.getCurrentPosition(), driveLeftBackHW.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds  LF:RB ,  RF:LB.", "%5.2f : %5.2f  ,  %5.2f : %5.2f", speedLeftFront, speedRightBack, speedRightFront, speedLeftBack);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    static public double getRawHeading() {
        YawPitchRollAngles angles   = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the "offset" heading back to zero
     */
    static public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
