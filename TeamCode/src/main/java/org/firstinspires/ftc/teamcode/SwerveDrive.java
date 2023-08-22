
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This OpMode executes a Swerve Drive control TeleOp for a two wheel/pod differential swerve drive robot
 * The code is structured as an Iterative OpMode
 * In this mode, the Right joysticks controls the direction of motion and speed, while the robot maintains orientation
 * and the left joystick-x changes the orientation (rotation) of the robot
 */
@TeleOp
public class SwerveDrive extends OpMode{
    // Constants
    static final double MAX_MTR_SPEED = 0.7;  // full speed  = 1.0. Backed off to prevent damage while developing
    static final double TICKS_PER_180_TURN = 433.5; // turn both motors this amount for 180 deg turn, was 458.62
    static final double TICKS_PER_INCH = 24.2; // CHECK VALUE.  real close ~25.3
    static final double TICKS_PER_RAD = 47.7*TICKS_PER_INCH/(2.0*Math.PI);
    static final double INITIAL_WHEEL_ANGLE = (Math.PI/2.0); // Wheel angle in radians

    /* Declare OpMode members. */
    double priorTime = 0.0; // for logging loop times

    // HARDWARE
    // DcMotors
    DcMotor motor1a = null;
    DcMotor motor1b = null;
    DcMotor motor2c = null;
    DcMotor motor2d = null;
    // IMU (inertia measurement unit, SDK 8.1 and later method of using IMU
    IMU imu;

    // Potentiometer Class
    PotClass pots;

    // ROBOT GLOBALS
    // target positions for robot
    double currentPodAngles = INITIAL_WHEEL_ANGLE;  // the angle of the pods relative to the x axis of the robot
    double targetPodAngles = INITIAL_WHEEL_ANGLE;
    int currentS = 0;  // the robot travel, in ticks
    double currentRobotAngle = 0; // the angle of the robot

    // target positions for each motor, in ticks
    int targetPosA, targetPosB, targetPosC, targetPosD;
    Orientation or; // robots orientation (x,y,z angles) from IMU on Hub
    double globalIMUHeading;
    int IMUHeadingUpdateTicks = 0;

    //int headingTicks = 0; // heading correction ticks
    int robotHeadingTicks = 0; // heading setting ticks


    /*
     * Code to run ONCE when the driver touches INIT
     */
    @Override
    public void init() {
        int turnTicks,turnTicks2;
        double deltaAngle;

        // Define and Initialize Motors
        motor1a = hardwareMap.get(DcMotor.class, "a");
        motor1b = hardwareMap.get(DcMotor.class, "b");
        motor2c = hardwareMap.get(DcMotor.class, "c");
        motor2d = hardwareMap.get(DcMotor.class, "d");

        motor1a.setDirection(DcMotor.Direction.FORWARD);
        motor1b.setDirection(DcMotor.Direction.REVERSE); // Why??

        motor2c.setDirection(DcMotor.Direction.FORWARD);
        motor2d.setDirection(DcMotor.Direction.FORWARD);

        pots = new PotClass();
        pots.initPots(hardwareMap);

        pots.getAngleFromPots(true,0); // find out where the wheel are pointed

        deltaAngle = pots.getShortestTurnAngle(pots.angle1,INITIAL_WHEEL_ANGLE);
        turnTicks =  (int)((deltaAngle / Math.PI) * TICKS_PER_180_TURN);
        targetPosA = turnTicks;
        targetPosB = turnTicks;

        deltaAngle = pots.getShortestTurnAngle(pots.angle2,INITIAL_WHEEL_ANGLE);
        turnTicks2 =  (int)((deltaAngle/ Math.PI) * TICKS_PER_180_TURN);
        targetPosC = turnTicks2;
        targetPosD = turnTicks2;

        setRunToPos();

        // map IMU
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.resetDeviceConfigurationForOpMode();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.secondAngle; // save the starting heading

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");

        // Write to log file
        RobotLog.d("SRA-start-SWERVE DRIVE = %.05f",getRuntime());
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        targetPosA=0;
        targetPosB=0;
        targetPosC=0;
        targetPosD=0;
        setRunToPos();
    }

    void setRunToPos() {
        motor1a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2c.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1a.setPower(MAX_MTR_SPEED);
        motor1b.setPower(MAX_MTR_SPEED);
        motor2c.setPower(MAX_MTR_SPEED);
        motor2d.setPower(MAX_MTR_SPEED);
        setMotorPositions();
        motor1a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setMotorPositions() {
        motor1a.setTargetPosition(targetPosA);
        motor1b.setTargetPosition(targetPosB);
        motor2c.setTargetPosition(targetPosC);
        motor2d.setTargetPosition(targetPosD);
    }

    @Override
    public void loop() {
        double gpRightX, gpRightY, gpLeftX;
        double loopTime;
        int PODturnTicks;
        double fwdSpeed;
        double deltaHeading;
        double deltaAngle;
        boolean goHeadingUpdate;
        YawPitchRollAngles ypa;

        gpRightX = gamepad1.right_stick_x; // used for wheel direction
        gpRightY = gamepad1.right_stick_y; // used for wheel direction

        gpLeftX = gamepad1.left_stick_x; // used for robot orientation

        fwdSpeed = Math.hypot(gpRightX,gpRightY); // used for robot speed

        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        ypa = imu.getRobotYawPitchRollAngles();
        //deltaHeading = pots.getShortestTurnAngle(globalIMUHeading,or.secondAngle);

        if (fwdSpeed>0.1) { // detect Right joystick movement, update angle
            targetPodAngles = Math.atan2(-gpRightY,-gpRightX); // atan2(y,x) = radians
            if (fwdSpeed>0.2) {
                currentS -= fwdSpeed*TICKS_PER_INCH;  // update distance
            }
        }
        deltaAngle= pots.getShortestTurnAngle(currentPodAngles,targetPodAngles);
        currentPodAngles += deltaAngle;
        currentPodAngles = pots.moduloAngle(currentPodAngles);

        PODturnTicks =  (int)(((currentPodAngles-INITIAL_WHEEL_ANGLE) / Math.PI) * TICKS_PER_180_TURN);

        // check if the wheel angle supports a heading change
        goHeadingUpdate = ((Math.abs(targetPodAngles)>= Math.PI/2.5) && (Math.abs(targetPodAngles)<=Math.PI/1.5));
        if(goHeadingUpdate) {
            //IMUHeadingUpdateTicks -= (int) (deltaHeading * 10); // Kp = 10 (P only feedback)

            currentRobotAngle += (gpLeftX/10.0); // update the robot angle in radians
            robotHeadingTicks = (int) (currentRobotAngle*TICKS_PER_RAD); // update robot heading in ticks
        } else {
            IMUHeadingUpdateTicks = 0;
        }

        targetPosA = currentS + PODturnTicks - robotHeadingTicks + IMUHeadingUpdateTicks;
        targetPosB = -currentS + PODturnTicks + robotHeadingTicks - IMUHeadingUpdateTicks;
        targetPosC = currentS + PODturnTicks + robotHeadingTicks - IMUHeadingUpdateTicks;
        targetPosD = -currentS + PODturnTicks - robotHeadingTicks + IMUHeadingUpdateTicks;

        setMotorPositions();

        telemetry.addData("Right stick y", gpRightY);
        telemetry.addData("Right stick x", gpRightX);
        telemetry.addData("Left stick x", gpLeftX);
        //telemetry.addData("Right Trigger =",fwdSpeed);
        telemetry.addData("current Pod Angle =","%.03f", currentPodAngles);
        telemetry.addData("Target Pod Angle =","%.03f", targetPodAngles);
        telemetry.addData("Robot Angle =","%.03f, TICKS = %d", currentRobotAngle,robotHeadingTicks);
        //telemetry.addData("TICKS A =", "%d, B = %d",targetPosA,targetPosB);
        //telemetry.addData("TICKS C =", "%d, D = %d",targetPosC,targetPosD);
        //telemetry.addData("POTENTIOMETER 1 =", "%.03f, 2= %.03f",potentiometer1.getVoltage(),potentiometer2.getVoltage());
        //telemetry.addData("POTENTIOMETER 3 =", "%.03f, 4= %.03f",potentiometer3.getVoltage(),potentiometer4.getVoltage());
        //telemetry.addData("deltaHeading IMU=","%.03f",deltaHeading);
        telemetry.update();

        loopTime = getRuntime() - priorTime;
        priorTime = getRuntime();

        RobotLog.d("SRA-loop-time = %.05f, loop = %.05f",priorTime,loopTime);
        RobotLog.d("SRA-loop-gamepad = %.03f, %.03f",gamepad1.right_stick_x,gamepad1.right_stick_y);
//        RobotLog.d("SRA-loop-ENCODERS = %d, %d, %d, %d", motor1a.getCurrentPosition(), motor1b.getCurrentPosition(), motor2c.getCurrentPosition(), motor2d.getCurrentPosition());
//        RobotLog.d("SRA-loop-POD 2 = %d, %d", motor2c.getCurrentPosition(), motor2d.getCurrentPosition());
//        RobotLog.d("SRA-loop-TARGET = %d, %d, %d, %d",targetPosA,targetPosB,targetPosC,targetPosD);
        RobotLog.d("SRA-IMU Angles RobAngle = %.03f, %.03f, %.03f, %.03f",currentRobotAngle,or.firstAngle,or.secondAngle,or.thirdAngle);
        RobotLog.d("SRA-IMU YPA RobAngle = %.03f, Y= %.03f, P=%.03f, R=%.03f",currentRobotAngle,ypa.getYaw(AngleUnit.RADIANS),ypa.getPitch(AngleUnit.RADIANS),ypa.getPitch(AngleUnit.RADIANS));
    }

    @Override
    public void stop() {
        RobotLog.d("SRA-end-SWERVE DRIVE = %.05f",getRuntime());
    }
}
