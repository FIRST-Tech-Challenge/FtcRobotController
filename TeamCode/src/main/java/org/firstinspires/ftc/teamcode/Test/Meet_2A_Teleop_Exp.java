package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Autonomous.BasicAutonomous;
import org.firstinspires.ftc.teamcode.Enums.DriveSpeedState;
import org.firstinspires.ftc.teamcode.Enums.RingCollectionState;
import org.firstinspires.ftc.teamcode.Enums.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Debouce;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain_v3;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Meet 2A Teleop Exp", group="Test")
//@Disabled // Leave disabled until ready to test


public class Meet_2A_Teleop_Exp extends BasicAutonomous {

    public Drivetrain_v3 drivetrain  = new Drivetrain_v3(true);   // Use subsystem Drivetrain
    public static final double DRIVE_SPEED = 0.80;     // Nominal speed for better accuracy.
    private Debouce mdebounce = new Debouce();
    private DriveSpeedState  currDriveState;

    RingCollectionState mRingCollectionState = RingCollectionState.OFF;
    //ShooterState mShooterState = ShooterState.STATE_SHOOTER_OFF; // default condition, this is needed to keep shooter on for a Linear Opmode

    // VuForia

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    // New Key created in 2020
    private static final String VUFORIA_KEY = "AZYEAT//////AAABmVQTIdrDekmFijIfmSRrV0lMe8Ecw4JdEXCVLgGS4LYCWT6vjXm57dCd1kTEqxKQPvbsorc32jUUmotoZT/NHLZeL0XOP1d1WuRDkadO2zIdRhED9NPsq3fh36bkbz2stnDiIOXlrOaIEbNetPG6b4INIOJ7B8oauCAAYjTY4ycZj6hfkS8NSp2QqVyYSZ3+dRVZiSHSU+nWObQyZoT24wJGhAbH3Y9BI8JdlizcQGjGzlqLfzUS8fIiQlB+9AAAEUAKkyKqg0dIcSFB6Rj+MCQ3kPrJ8VpAxUGXZ84Zxa7CbtKn79+cmLbs18FIu706qObLUtZbbDCCDdSv6DlBVfzrkzgcC4WytmaogFryoGWN";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 70.5f * mmPerInch;
    private static final float quadField  = 35.5f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override
    public void runOpMode() {
        double drive;
        double turn;
        double left;
        double right;
        double max;
        double speedfactor = 0.5;

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        currDriveState = DriveSpeedState.DRIVE_FAST; // initialize robot to FAST

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsUltimateGoal.activate();

        // Call init methods in the various subsystems
        // if "null exception" occurs it is probably because the hardware init is not called below.
        drivetrain.init(hardwareMap);
        wobble.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        elevator.init(hardwareMap);

        shooter.shooterReload(); // reload = flipper back, stacker mostly down, shooter off

        // Gyro set-up
        BNO055IMU.Parameters gyroparameters = new BNO055IMU.Parameters();

        gyroparameters.mode = BNO055IMU.SensorMode.IMU;
        gyroparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroparameters.loggingEnabled = false;

        // Init gyro parameters then calibrate
        drivetrain.imu.initialize(gyroparameters);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        // Encoder rest is handled in the Drivetrain init in Drivetrain class

        // Calibrate gyro
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !drivetrain.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", drivetrain.imu.getCalibrationStatus().toString());
        /** Wait for the game to begin */

        telemetry.update();

        /////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////


        while (opModeIsActive()){
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            //========================================
            // GAME PAD 1
            //========================================
            // left joystick is assigned to drive speed
            drive = -gamepad1.left_stick_y;
            // right joystick is for turning
            turn  =  gamepad1.right_stick_x;
            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max; // does this to stay within the limit and keeps the ratio the same
                right /= max;
            }
            if (gamepad1.left_bumper && mRingCollectionState == RingCollectionState.OFF) {
                    shooter.flipperBackward();
                    shooter.stackerMoveToMidLoad();
                    mRingCollectionState = RingCollectionState.COLLECT;
                    telemetry.addData("Collector State", mRingCollectionState);
                    mdebounce.debounce(175); // need to pause for a few ms to let drive release the button

                }
            if (gamepad1.left_bumper && mRingCollectionState == RingCollectionState.COLLECT) {
                    shooter.flipperBackward();
                    shooter.stackerMoveToMidLoad();
                    mRingCollectionState = RingCollectionState.OFF;
                    telemetry.addData("Collector State", mRingCollectionState);
                    mdebounce.debounce(175);
                }


            if (gamepad1.right_bumper && mRingCollectionState == RingCollectionState.OFF) {
                    shooter.flipperBackward();
                    shooter.stackerMoveToReload();
                    mRingCollectionState = RingCollectionState.EJECT;
                    telemetry.addData("Collector State", mRingCollectionState);
                    mdebounce.debounce(175);

                }

            if (gamepad1.right_bumper && mRingCollectionState == RingCollectionState.EJECT) {
                    shooter.flipperBackward();
                    shooter.stackerMoveToReload();
                    mRingCollectionState = RingCollectionState.OFF;
                    telemetry.addData("Collector State", mRingCollectionState);
                    mdebounce.debounce(175);

                }

            if (gamepad1.x) {
                    //shooter.shooterReload();
                    shooter.stackerMoveToReload();
                    telemetry.addData("Stacker Reset", "Complete ");

                }
            if (gamepad1.y) {
                    shooter.shootOneRingHigh();
                  //shooter.shootMiddleGoal();
                    mRingCollectionState = RingCollectionState.OFF;

                    telemetry.addData("Shooter High", "Complete ");
                }

            if (gamepad1.a) {
                    shooter.shooterReload();
                    //shooter.shooterOff();
                    telemetry.addData("Shooter High", "Complete ");
                }
            if (gamepad1.b) {
                    shooter.stackerMoveToShoot();
                    mRingCollectionState = RingCollectionState.OFF;
                    telemetry.addData("Stacker Ready to Shoot", "Complete ");
                }
            if (gamepad1.left_trigger > 0.25) {
                    shooter.flipperForward();
                    mdebounce.debounce(700);
                   telemetry.addData("Flipper Fwd", "Complete ");
                    shooter.flipperBackward();
                    mdebounce.debounce(700);
                }
            if (gamepad1.right_trigger > 0.25) {
                    //shooter.flipperBackward();
                    //telemetry.addData("Flipper Back", "Complete ");
                    shooter.shootonePowerShots();
                    telemetry.addData("SHooter Low for Power Shots", "Complete ");
            }


            if (gamepad1.left_stick_button)
            {
                currDriveState = DriveSpeedState.DRIVE_FAST;
            }
            if (gamepad1.right_stick_button)
            {
                currDriveState =  DriveSpeedState.DRIVE_SLOW;
            }

            // Wobble Controls

            if (gamepad1.dpad_left) {
                wobble.GripperOpen();
                wobble.ArmExtend();
                telemetry.addData("Ready to rab Wobble", "Complete ");
            }

            if (gamepad1.dpad_up){
                wobble.GripperClose();
                sleep(500);
                wobble.ArmCarryWobble();
                telemetry.addData("Carrying Wobble", "Complete ");
            }
            if (gamepad1.dpad_right) {
                wobble.GripperOpen();
                wobble.ArmExtend();
                telemetry.addData("Dropping Wobble", "Complete ");
            }
            if (gamepad1.dpad_down) {
                wobble.ArmContract();
                wobble.GripperOpen();
                wobble.LiftLower();
                telemetry.addData("Reset Wobble", "Complete ");
            }
            if (gamepad1.back){
                wobble.LiftRise();
            }


            //========================================
            // GAME PAD 2
            //========================================
            if (gamepad2.x) {
                gyroTurn(TURN_SPEED,0,3);

            }

            //========================================
            // Switch Cases
            //========================================
            switch(currDriveState) {

                case DRIVE_FAST:
                    telemetry.addData("Drive Speed",currDriveState);
                    drivetrain.leftFront.setPower(left);
                    drivetrain.rightFront.setPower(right);
                    //leftFront.setPower(left);
                    //rightFront.setPower(right);

                    // Send telemetry message to signify robot running;
                    telemetry.addData("left",  "%.2f", left);
                    telemetry.addData("right", "%.2f", right);
                    break;

                case DRIVE_SLOW:
                    telemetry.addData("Drive Speed",currDriveState);
                    drivetrain.leftFront.setPower(left*speedfactor);
                    drivetrain.rightFront.setPower(right*speedfactor);
                    //leftFront.setPower(left*speedfactor);
                    //rightFront.setPower(right*speedfactor);

                    // Send telemetry message to signify robot running;
                    telemetry.addData("left",  "%.2f", left);
                    telemetry.addData("right", "%.2f", right);
                    break;
            }
            switch(mRingCollectionState) {

                case OFF:
                    telemetry.addData("Collector State",mRingCollectionState);
                    intake.Intakeoff();;
                    elevator.Elevatoroff();

                    break;

                case COLLECT:
                    telemetry.addData("Collector State",mRingCollectionState);
                    intake.Intakeon();;
                    elevator.ElevatorSpeedfast();
                    break;

                case EJECT:
                    telemetry.addData("Collector State",mRingCollectionState);
                    intake.IntakeReverse();;
                    elevator.Elevatorbackup();
                    break;
            }


            telemetry.update();
        } // while active bracket


        } // runopmode bracket




    } // class bracket






