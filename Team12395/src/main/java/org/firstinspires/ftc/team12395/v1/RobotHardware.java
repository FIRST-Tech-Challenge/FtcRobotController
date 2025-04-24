package org.firstinspires.ftc.team12395.v1;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    public ElapsedTime runtime = new ElapsedTime();

    public IMU imu = null;

    public double drivePower = 0;
    public double strafePower = 0;
    public double turnPower = 0;

    public double leftFrontPower = 0;
    public double leftBackPower = 0;
    public double rightFrontPower = 0;
    public double rightBackPower = 0;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor slideMotorL = null;
    private DcMotor slideMotorR = null;

    private Servo horizontal1 = null;
    private Servo lextend = null;
    private Servo rextend = null;
    private Servo leftOutTake = null;
    private Servo rightOutTake = null;
    private Servo inClaw = null;
    private Servo outClaw = null;
    private Servo rotClaw = null;

    public int leftFrontTarget;
    public int leftBackTarget;
    public int rightFrontTarget;
    public int rightBackTarget;

    public double COUNTS_PER_MOTOR_REV = 537.7;
    public double WHEEL_DIAMETER_INCHES = 3.77953;
    public double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);



    public double SLIDE_TICKS_PER_DEGREE = 28.0 * 19.2 / 360.0;

    public double SLIDE_START = 0.0 * SLIDE_TICKS_PER_DEGREE;
    public double SLIDE_HIGH_RUNG = 82 * SLIDE_TICKS_PER_DEGREE;
    public double SLIDE_HIGH_BASKET = 2100 * SLIDE_TICKS_PER_DEGREE;
    public double slidePosition = (int)SLIDE_START;

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));


    public RobotHardware(LinearOpMode OpMode) {myOpMode = OpMode;}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        slideMotorL = myOpMode.hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorR = myOpMode.hardwareMap.get(DcMotor.class, "slideMotorR");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point on opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives mat require direction flips.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        slideMotorR.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotorL.setTargetPosition(0);
        slideMotorR.setTargetPosition(0);
//        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Define and initialize ALL installed servos.
        horizontal1 = myOpMode.hardwareMap.get(Servo.class, "horizontal1");
        lextend = myOpMode.hardwareMap.get(Servo.class, "lextend");
        rextend = myOpMode.hardwareMap.get(Servo.class, "rextend");
        leftOutTake = myOpMode.hardwareMap.get(Servo.class, "leftOutTake");
        rightOutTake = myOpMode.hardwareMap.get(Servo.class, "rightOutTake");

        inClaw = myOpMode.hardwareMap.get(Servo.class, "inClaw");
        outClaw = myOpMode.hardwareMap.get(Servo.class, "outClaw");
        rotClaw = myOpMode.hardwareMap.get(Servo.class, "rotClaw");

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);



        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power(-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power(-1.0 to 1.0) +ve is CW
     * @param Strafe
     */
    public void driveRobotCentric(double Drive,double Strafe, double Turn){
        //Combine drive and turn for blended motion.
        double leftFrontPower = Drive + Strafe + Turn;
        double leftBackPower = Drive - Strafe + Turn;
        double rightFrontPower = Drive - Strafe - Turn;
        double rightBackPower = Drive + Strafe -  Turn;

        //Scale the values so neither exceed +/-1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0)
        {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void driveFieldCentric(double drive, double strafe, double turn){
        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);



        leftFrontPower = driveRotation + strafeRotation + turn;
        leftBackPower = driveRotation - strafeRotation + turn;
        rightFrontPower = driveRotation - strafeRotation - turn;
        rightBackPower = driveRotation + strafeRotation - turn;

        max = Math.max(Math.abs(leftBackPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if(max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);


    }

    public void driveEncoder(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches){
        // drives only while myOpMode is active
        if(myOpMode.opModeIsActive()){


            //determine new target position
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(leftFrontTarget - 1);
            leftBackDrive.setTargetPosition(leftBackTarget - 1 );
            rightFrontDrive.setTargetPosition(rightFrontTarget - 1);
            rightBackDrive.setTargetPosition(rightBackTarget - 1 );

            //turn on RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the time and start motion

            runtime.reset();
            setDrivePower(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

            while ((myOpMode.opModeIsActive() &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                            rightFrontDrive.isBusy() && rightBackDrive.isBusy()))){

                //display it for driver

                myOpMode.telemetry.addData("Running to ", " %7d :%7d :%7d :%7d",
                        leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                myOpMode.telemetry.addData("Currently at ", "%7d ;%7d :%7d :%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            setDrivePower(0, 0, 0, 0 );
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(200); //optional pause after each move
        }
    }



    public void SetSlidePosition(double slidePosition){
        slideMotorL.setTargetPosition((int)(slidePosition));
        slideMotorR.setTargetPosition((int)(slidePosition));

        ((DcMotorEx) slideMotorL).setVelocity(2500);
        ((DcMotorEx) slideMotorR).setVelocity(2500);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }



    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     * the extends are what extend the slides
     * @param offset
     */
    public void setIntakePosition(double offset){
        //whatever value you subtract from lextend should be added to rextend and vise versa
        if (offset == 1) {
            //extended
            lextend.setPosition(1);
            rextend.setPosition(0);
        } else if (offset == 0) {
            //retracted
            lextend.setPosition(0.8);
            rextend.setPosition(0.2 * 360 / 300);
        } else if (offset == 2){
            lextend.setPosition(0.97);
            rextend.setPosition(.135 * 360 / 300);
        }
    }

    public void setHorizontalPosition(double offset) {
        if (offset == 0) {
            horizontal1.setPosition(0.24);
            //b is pressed
        } else if (offset == 1) {
            //x is pressed
            horizontal1.setPosition(0.79);
        }
    }


    //changes here affect both duo and solo , add to left and right.
    public void setVerticalPower(double power) {
        if (power == 1) {
            // a is pressed
            leftOutTake.setPosition(.34);
            rightOutTake.setPosition(.66);
        }else if (power == 0){
            // y is pressed
            leftOutTake.setPosition(.622);
            rightOutTake.setPosition(.378);
        }else if (power == 2){
            leftOutTake.setPosition(.9);
            rightOutTake.setPosition(0.1);
        } else if (power == 3){
            leftOutTake.setPosition(.8);
            rightOutTake.setPosition(.2);
        }

    }

    public void setInClawPosition(double power){
        if(power == 1){
            //closed
            inClaw.setPosition(0.4);
        }else if(power == 0){
            inClaw.setPosition(0);
        }
    }

    public void  setInClawRotation(double power){
        if(power == 0){
            rotClaw.setPosition(.27);
        } else if(power == 1){
            rotClaw.setPosition(.59);
        }
    }

    public void setOutClawPosition(double power){
        if(power == 1){
            //button pressed
            outClaw.setPosition(1);
        }else if(power == 0){
            outClaw.setPosition(.7);
        }
    }
}


