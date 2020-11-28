package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Enums.RingCollectionState;
import org.firstinspires.ftc.teamcode.Enums.ShooterState;
import org.firstinspires.ftc.teamcode.Enums.WobbleTargetZone;
import org.firstinspires.ftc.teamcode.Test.BasicAutonomous_II;

import java.util.List;

@Autonomous(name="Blue_Left_Line_HGWB", group="Test")

// This opmode EXTENDS BASICAUTONOMOUS and actually does the same thing as BasicAutonomous
// The goal here was to extend a base class with all the methods and prove it works just the same.

// Place robot on the left most blue line when facing the goal. Robot should be placed such that
// as it drives straight ahead it will not hit the stack of rings. So basically center the robot on
// the seam between the first and second floor tile. Which is an inch or to to the right of the blue line.

// Alignment Position LH Blue Line when facing the goal
//    B     X       X       X
//    B     X       X       X
//    B     X       X       X
//    B     X       X       X
//    B     X       X       X

public class BLUE_Left_Line_Auto_HGWB extends BasicAutonomous {

    private static  final double       extraRingShootTimeAllowed    = 2; //  timer for shooting the single ring unique to this opMode
    private static  final double       autoShootTimeAllowed = 4;//
    private static final  double       autoRingCollectTimeAllowed = 1.5; // time allowed to let the single ring to get picked up
    private static final double        shooterStartUpTimeAllowed = 1.25;

    @Override
    public void runOpMode() {


        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();

            // See BasicAutonomous for details on camera zoom settings.
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }

        // Call init methods in the various subsystems
        // if "null exception" occurs it is probably because the hardware init is not called below.
        drivetrain.init(hardwareMap);
        wobble.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        elevator.init(hardwareMap);


        // move implements to start position. Note, 18x18x18 inch cube has to be maintained
        // until start is pressed. Position servos and motors here so human error and set-up is not
        // as critical. Team needs to focus on robot alignment to the field.

        shooter.shooterReload(); // reload = flipper back, stacker mostly down, shooter off
        // nothing here for wobble goal yet. Gravity will take care of most of it.
        // the wobble gripper is automatically opened during the wobble init.

        // Gyro set-up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Init gyro parameters then calibrate
        drivetrain.imu.initialize(parameters);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        // Encoder rest is handled in the Drivetrain init in Drivetrain class

        // Calibrate gyro

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && drivetrain.imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", drivetrain.imu.getCalibrationStatus().toString());
        /** Wait for the game to begin */
        telemetry.addData("Square", Square);

        telemetry.update();

        /////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////
        tfTime.reset(); //  reset the TF timer
        while (tfTime.time() < tfSenseTime && opModeIsActive()) { // need to let TF find the target so timer runs to let it do this
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        ///
                        StackSize = recognition.getLabel();
                        //telemetry.addData("Target", Target);
                        if (StackSize == "Quad") {
                            Square = WobbleTargetZone.BLUE_C;
                            telemetry.addData("Square", Square);
                        } else if (StackSize == "Single") {
                            Square = WobbleTargetZone.BLUE_B;
                            telemetry.addData("Square", Square);

                        }

                    }
                    telemetry.update();
                }
            }
            if (tfod != null) {
                tfod.shutdown();
            }
        }
        // Pick up the Wobble Goal before moving.
        // Sleep statements help let things settle before moving on.
        wobble.GripperOpen();
        wobble.ArmExtend();
        sleep(500);
        wobble.GripperClose();
        sleep(500);
        wobble.ArmCarryWobble();
        sleep(250);

        // After picking up the wobble goal the robot always goes to the same spot to shoot the 3 preloaded rings.
        // After delivering the rings, the switch case has the appropriate drive path to the identified Target Zone.

        drivetime.reset(); // reset because time starts when TF starts and time is up before we can call gyroDrive
        // Drive paths are initially all the same to get to the shooter location
        gyroDrive(DRIVE_SPEED, 57.0, 0.0, 10);
        gyroTurn(TURN_SPEED,-10,3);
        mShooterState = ShooterState.STATE_SHOOTER_ACTIVE;
        shooterStartUp(mShooterState, shooterStartUpTimeAllowed);
        shoot3Rings(mShooterState, autoShootTimeAllowed);   // call method to start shooter and launch 3 rings. pass shooter state in case it is needed
        drivetime.reset(); // reset because time starts when TF starts and time is up before we can call gyroDrive

        // Switch manages the 3 different Target Zone objectives based on the number of rings stacked up
        // Ring stack is none, one or 4 rings tall and is determined by a randomization process.
        // Robot has to read the stack height and set the Target Zone square state based on Vuforia/ Tensor Flow detection
        switch(Square){
            case BLUE_A: // no rings. 3 tiles (24 inches per tile) forward and one tile to the left from start
                telemetry.addData("Going to BLUE A", "Target Zone");
                gyroTurn(TURN_SPEED*.5,20,3);
                gyroDrive(DRIVE_SPEED, 6.0, 20.0, 5);
                sleep(750);
                wobble.GripperOpen();
                sleep(500);
                wobble.ArmExtend();
                drivetime.reset();
                gyroTurn(TURN_SPEED *.5, 25,2);
                gyroDrive(DRIVE_SPEED, -44.0, 25, 5);
                wobble.ArmContract();

                break;
            case BLUE_B: // one ring  4 tiles straight ahead
                telemetry.addData("Going to BLUE B", "Target Zone");
                gyroDrive(DRIVE_SPEED, 25.0, -20.0, 5);
                sleep(750);
                wobble.GripperOpen();
                sleep(500);
                wobble.ArmExtend();
                drivetime.reset();
                gyroDrive(DRIVE_SPEED, -13.0, -15, 5);
                wobble.ArmContract();
                gyroTurn(TURN_SPEED *.5, 190,3); // turn towards ring stack
                gyroDrive(DRIVE_SPEED, 9,190,3); //drive to ring stack w/ gyro ri get correct heading
                mRingCollectionState = RingCollectionState.COLLECT; // change collector state to get ready to pick up rings
                collectRingsInAuto_A(mRingCollectionState, autoRingCollectTimeAllowed);// switch to method to drive and collect (no encoders)
                drivetime.reset();
                gyroTurn(TURN_SPEED * .75, -1,3); // rotate back towards goal
                gyroDrive(DRIVE_SPEED,32,-1,3); // drive to goal at prescribed heading
                mShooterState = ShooterState.STATE_SHOOTER_ACTIVE; // set shooter to active again
                shoot3Rings(mShooterState, extraRingShootTimeAllowed);   // call method to start shooter and launch rings. Time will only let 1 or 2 out this time
                gyroDrive(DRIVE_SPEED,6,-1,3); // make final drive to the line and park
                wobble.ArmContract();

                break;
            case BLUE_C: // four rings. 5 tiles forward and one tile to the left.
                telemetry.addData("Going to RED C", "Target Zone");
                gyroTurn(TURN_SPEED,10,3);
                gyroDrive(DRIVE_SPEED, 48, 10.0, 5);
                sleep(1000);
                wobble.GripperOpen();
                wobble.ArmExtend();
                sleep(1000);
                drivetime.reset();
                gyroDrive(DRIVE_SPEED, -43.0, 20, 5);
                gyroDrive(.95, -23.0, 20, 5);
                gyroDrive(DRIVE_SPEED, 26.0, 20, 5);
                //gyroDrive(DRIVE_SPEED, 2.0, 10, 5);
                gyroTurn(TURN_SPEED,-180,3);
                wobble.ArmContract();

                break;
        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }






}
