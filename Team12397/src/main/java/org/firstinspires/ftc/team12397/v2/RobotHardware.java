package org.firstinspires.ftc.team12397.v2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    private LinearOpMode myOpMode = null;

    public ElapsedTime runtime = new ElapsedTime();

    public IMU imu = null;

    public double drivePower=0;
    public double strafePower=0;
    public double turnPower=0;

    public double leftFrontPower = 0;
    public double leftBackPower = 0;
    public double rightFrontPower = 0;
    public double rightBackPower = 0;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    public DcMotorEx rotateMotor = null;
    public DcMotorEx slideExtender = null;

    private Servo clawPinch = null; // closes/opens  .5 close | 0 open
    public Servo clawYaw = null; // rotates the claw around a vertical axis 0 default | .6? turn around
    public Servo clawPitch = null; // rotates the claw around a horizontal axis .5 center

    public int leftFrontTarget;
    public int leftBackTarget;
    public int rightFrontTarget;
    public int rightBackTarget;

    public WebcamName camera = null;
  
    // wheel ticks:
    public final double COUNTS_PER_MOTOR_REV = 537.7;
    public final double WHEEL_DIAMETER_INCHES = 3.77953;
    public final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // rotator ticks:
    public final double ROTATE_SLIDE_TICKS_PER_DEGREE = (28.0 * 50.9 / 360.0) * (100.0 / 30.0);
    public final double ROTATION_START = 0.0 * ROTATE_SLIDE_TICKS_PER_DEGREE;
    public final double ROTATION_90 = 90 * ROTATE_SLIDE_TICKS_PER_DEGREE;
    public final double ROTATION_MINIMUM = -565/ROTATE_SLIDE_TICKS_PER_DEGREE;
    public final double ROTATION_MAXIMUM = 715/ROTATE_SLIDE_TICKS_PER_DEGREE;

    // extender ticks:
    public final double EXTEND_SLIDE_TICKS_PER_REV = (((((1+(46./17))) * (1+(46./11))) * 28));
    public final double EXTEND_SLIDE_TICKS_PER_INCH = EXTEND_SLIDE_TICKS_PER_REV/ (112/25.4); // 112: https://www.gobilda.com/3407-series-hub-mount-winch-pulley-dual-spool-112mm-circumference/
  
    // mm / 25.4 = in
    public final double EXTENDER_SLIDE_MAXIMUM_TICKS = EXTEND_SLIDE_TICKS_PER_INCH*17.5;
    // end of ticks...

    IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));


    public RobotHardware(LinearOpMode OpMode) {myOpMode = OpMode;}
/**
  Initialize all the robot's hardware.
  This method must be called ONCE when the OpMode is initialized.
**/
    public void init() {
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "right_back");

        slideExtender = myOpMode.hardwareMap.get(DcMotorEx.class, "slide_extender");
        rotateMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "rotate_motor");

        camera = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        // motor directions...
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        rotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideExtender.setDirection(DcMotorSimple.Direction.FORWARD);

        // encoder resets
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotateMotor.setTargetPosition(0);
        rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      
        slideExtender.setTargetPosition(0);
        slideExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // zero power behavior...
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        clawPinch = myOpMode.hardwareMap.get(Servo.class, "claw_pinch");
        clawYaw = myOpMode.hardwareMap.get(Servo.class, "claw_yaw");
        clawPitch = myOpMode.hardwareMap.get(Servo.class, "claw_pitch");


        imu = myOpMode.hardwareMap.get(IMU .class, "imu");
        imu.initialize(parameters);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     *
     * @param Drive Fwd/Rev driving power(-1.0 to 1.0) +ve is forward
     * @param Strafe Right/Left turning power(-1.0 to 1.0) +ve is CW
     * @param Turn
     */
    public void driveRobotCentric(double Drive,double Strafe, double Turn) {


        //Combine drive and turn for blended motion.
        double leftFrontPower = Drive + Strafe + Turn;
        double leftBackPower = Drive - Strafe + Turn;
        double rightFrontPower = Drive - Strafe - Turn;
        double rightBackPower = Drive + Strafe - Turn;

        //Scale the values so neither exceed +/-1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void driveFieldCentric(double drive, double strafe, double turn) {
        //make the parameters store the value assigned when calling the method
        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);


        // Combine drive and Turn for blended motion.
        leftFrontPower = driveRotation + strafeRotation + turn;
        leftBackPower = driveRotation - strafeRotation + turn;
        rightFrontPower = driveRotation - strafeRotation - turn;
        rightBackPower = driveRotation + strafeRotation -turn;

        // Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) , Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFront.setPower(leftFrontWheel);
        leftBack.setPower(leftBackWheel);
        rightFront.setPower(rightFrontWheel);
        rightBack.setPower(rightBackWheel);
    }

    public void driveEncoder(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches){
        // drives only while myOpMode is active
        if(myOpMode.opModeIsActive()){
          
            //determine new target position
            leftFrontTarget = leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            rightBackTarget = rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(leftFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            rightBack.setTargetPosition(rightBackTarget);

            //turn on RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the time and start motion

            runtime.reset();
            setDrivePower(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

            while ((myOpMode.opModeIsActive() &&
                    (leftFront.isBusy() && leftBack.isBusy() &&
                            rightFront.isBusy() && rightBack.isBusy()))){

                //display it for driver

                myOpMode.telemetry.addData("Running to ", " %7d :%7d :%7d :%7d",
                        leftFront, leftBack, rightFront, rightBack);
                myOpMode.telemetry.addData("Currently at ", "%7d ;%7d :%7d :%7d",
                        leftFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                        rightFront.getCurrentPosition(), rightBack.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            setDrivePower(0, 0, 0, 0 );
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    /**
     *
     * This is NOT relative. Absolute distance from the starting point
     * @param inches inches from the retracted position: another term could be absolute inches.
     */
    public void setExtenderPosition(double inches){
        // ensure requested position is not negative distance or overextending.
        inches = Math.min(inches, 17.5);
        inches = Math.max(inches, 0);

        slideExtender.setTargetPosition((int) (inches*EXTEND_SLIDE_TICKS_PER_INCH));
        slideExtender.setVelocity(2500);
        slideExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     *
     * This is NOT relative. Absolute distance from the starting point
     * @param rotateDegrees degrees from the starting position: another term could be absolute rotation.
     */
    public void RotateSlides(double rotateDegrees){
        rotateDegrees = Math.min(rotateDegrees, ROTATION_MAXIMUM);
        rotateDegrees = Math.max(rotateDegrees, ROTATION_MINIMUM);

        rotateMotor.setTargetPosition((int) (rotateDegrees * ROTATE_SLIDE_TICKS_PER_DEGREE));
        rotateMotor.setVelocity(2500);
        rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // intention: testing
    // waiting on hardware implementation...
    public void setServoPosition(int servoNum, double position){
        switch (servoNum) {
            case 0:
                clawPinch.setPosition(position);
                break;
            case 1:
                clawYaw.setPosition(position);
                break;
            case 2:
                clawPitch.setPosition(position);
                break;
        }
    }

}