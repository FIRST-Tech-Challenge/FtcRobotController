
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This OpMode executes a Motorcycle Mode control TeleOp for a two wheel/pod differential swerve drive robot
 * The code is structured as an Iterative OpMode
 * In this mode, the Right joysticks controls the direction of motion and steering
 */

@TeleOp
public class MotorcycleMode extends OpMode{
    // Constants
    static final double TICK_SPEED = 10.0; // gamepad joystick tick increment (speed) multiplier for linear movement.
    static final double TICK_TURN = 170.0; // gamepad joystick tick multiplier for rotation. 220 ticks approximately turns wheels 90 degrees
    static final double MAX_MTR_SPEED = 0.8;  // full speed  = 1.0. Backed off to prevent damage while developing
    static final double TICKS_PER_180_TURN = 433.5; // turn both motors this amount for 180 deg turn, was 458.62

    static final double INITIAL_WHEEL_ANGLE = 0.0; // Wheel initial angle, in radians

    /* Declare OpMode members. */
    double priorTime = 0.0; // for logging loop times

    // HARDWARE
    // DcMotors
    DcMotor motor1a = null;
    DcMotor motor1b = null;
    DcMotor motor2c = null;
    DcMotor motor2d = null;
    // Potentiometer Class
    PotClass pots;

    int targetPosition = 0; // motorcycle mode translation, in ticks
    double turnPod1 = 0; // used for both pod angles in translateMode
    double turnPod2=0;
    // target positions for each motor, in ticks
    int targetPosA, targetPosB, targetPosC, targetPosD;
    //double speed; // the robot speed

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

        pots.getAngleFromPots(false,0); // find out where the wheel are pointed

        deltaAngle = pots.getShortestTurnAngle(pots.angle1,INITIAL_WHEEL_ANGLE);
        turnTicks =  (int)((deltaAngle / Math.PI) * TICKS_PER_180_TURN);
        targetPosA = turnTicks;
        targetPosB = turnTicks;

        deltaAngle = pots.getShortestTurnAngle(pots.angle2,INITIAL_WHEEL_ANGLE);
        turnTicks2 =  (int)((deltaAngle/ Math.PI) * TICKS_PER_180_TURN);
        targetPosC = turnTicks2;
        targetPosD = turnTicks2;

        setRunToPos();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");

        // Write to log file
        RobotLog.d("SRA-start-Motorcycle_Mode = %.05f",getRuntime());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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
    void setMotorMotorcycleMode() {
        // used in motorcycle mode
        targetPosA = targetPosition + (int)(turnPod1* TICK_TURN);
        targetPosB = -targetPosition + (int)(turnPod1* TICK_TURN);
        targetPosC = targetPosition + (int)(turnPod2* TICK_TURN);
        targetPosD = -targetPosition + (int)(turnPod2* TICK_TURN);
        //
        setMotorPositions();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double gpLeftX, gpRightY;
        double loopTime;
       double podAngle;

        gpLeftX = gamepad1.left_stick_x;
        gpRightY = gamepad1.right_stick_y;
        gpRightY = gpRightY*gpRightY*gpRightY;  // cube

        podAngle = gpLeftX;
        turnPod1 = podAngle;
        turnPod2 = -podAngle;

        targetPosition += (int) (gpRightY * TICK_SPEED);
        setMotorMotorcycleMode();

        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.update();

        loopTime = getRuntime() - priorTime;
        priorTime = getRuntime();

        RobotLog.d("SRA-loop-MOTORCYCLE = %.05f, loop = %.05f",priorTime,loopTime);
        //RobotLog.d("SRA-loop-gamepad = %.03f, %.03f",gamepad1.right_stick_x,gamepad1.right_stick_y);
        //RobotLog.d("SRA-loop-ENCODERS = %d, %d, %d, %d", motor1a.getCurrentPosition(), motor1b.getCurrentPosition(), motor2c.getCurrentPosition(), motor2d.getCurrentPosition());
        //RobotLog.d("SRA-loop-TARGET = %d, %d, %d, %d",targetPosA,targetPosB,targetPosC,targetPosD);
    }

    @Override
    public void stop() {
        RobotLog.d("SRA-end-MOTORCYCLE_MODE = %.05f",getRuntime());
    }
}
