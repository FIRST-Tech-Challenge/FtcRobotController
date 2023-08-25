
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This OpMode executes a Swerve Drive Potentiometer Recording Auto for a two wheel/pod differential swerve drive robot
 * The code is structured as an Iterative OpMode
 */
@TeleOp
public class PotentiometerRecord extends OpMode{
    // Constants
    static final double MAX_MTR_SPEED = 0.7;  // full speed  = 1.0. Backed off to prevent damage while developing
    static final double TICKS_PER_180_TURN = 435; // turn both motors this amount for 180 deg turn
    static final double INITIAL_WHEEL_ANGLE = 0.0; // Wheel angle in radians
    static final boolean CW = true; // If true, wheel rotation is CW, else CCW

    /* Declare OpMode members. */
    double priorTime = 0.0; // for logging loop times

    // HARDWARE
    // DcMotors
    DcMotor motor1a = null;
    DcMotor motor1b = null;
    DcMotor motor2c = null;
    DcMotor motor2d = null;

    // Potentiometer Class
    public TwoFullRotationPotClass pots;

    // ROBOT GLOBALS
    // target positions for robot
    double currentAngle = INITIAL_WHEEL_ANGLE;  // the robot angle

    // target positions for each motor, in ticks
    int targetPosA, targetPosB, targetPosC, targetPosD;

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

        pots = new TwoFullRotationPotClass();
        pots.initPots(hardwareMap);

        pots.getAngleFromPots(true,INITIAL_WHEEL_ANGLE); // find out where the wheel are pointed

        deltaAngle = pots.getShortestTurnAngle(pots.angle1,INITIAL_WHEEL_ANGLE);
        turnTicks =  (int)((deltaAngle / Math.PI) * TICKS_PER_180_TURN);
        targetPosA = turnTicks;
        targetPosB = turnTicks;

        deltaAngle = pots.getShortestTurnAngle(pots.angle2,INITIAL_WHEEL_ANGLE);
        turnTicks2 =  (int)((deltaAngle/ Math.PI) * TICKS_PER_180_TURN);
        targetPosC = turnTicks2;
        targetPosD = turnTicks2;

        setMotorPositions(); // Turn to the initial position

        setRunToPos();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");

        // Write to log file
        RobotLog.d("SRA-start-PotentiometerRecord = %.05f",getRuntime());
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
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
        double loopTime;
        int ticks;
        boolean log;

        if (CW) {
            if(currentAngle<=(4.1*Math.PI)) currentAngle += 0.03;  // spin a bit more than 2 revs, increment angle
            log = ((currentAngle>=(2.0*Math.PI)) && (currentAngle<=(4.0*Math.PI))); // only log 2nd rotation
        } else {
            if(currentAngle>=(-4.1*Math.PI)) currentAngle -= 0.03;  // spin a bit more than 2 revs, increment angle
            log = ((currentAngle<=(-2.0*Math.PI)) && (currentAngle>=(-4.0*Math.PI))); // only log 2nd rotation
        }

        ticks = (int)((currentAngle/Math.PI) * TICKS_PER_180_TURN);
        targetPosA = ticks;
        targetPosB = ticks;
        targetPosC = ticks;
        targetPosD = ticks;
        setMotorPositions();

        pots.getAngleFromPots(log,pots.moduloAngle(currentAngle));
        telemetry.addData("CURRENT ANG (rad)","=%.03f",currentAngle);
        telemetry.addData("POT 1 ANGLE (rad)","=%.03f",pots.angle1);
        telemetry.addData("POT 2 ANGLE (rad)","=%.03f",pots.angle2);
        telemetry.update();

        loopTime = getRuntime() - priorTime;
        priorTime = getRuntime();

        RobotLog.d("SRA-loop-time = %.05f, loop = %.05f",priorTime,loopTime);
        //RobotLog.d("SRA-loop-ENCODERS = %d, %d, %d, %d", motor1a.getCurrentPosition(), motor1b.getCurrentPosition(), motor2c.getCurrentPosition(), motor2d.getCurrentPosition());
        //RobotLog.d("SRA-loop-TARGET = %d, %d, %d, %d",targetPosA,targetPosB,targetPosC,targetPosD);
    }

    @Override
    public void stop() {
        RobotLog.d("SRA-end-PotentiometerRecord = %.05f",getRuntime());
    }
}
