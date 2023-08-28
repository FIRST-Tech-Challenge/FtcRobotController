package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This OpMode executes a Motorcycle Mode control TeleOp for a two wheel/pod differential swerve drive robot
 * The code is structured as an Iterative OpMode
 * In this mode, the Right joysticks controls the direction of motion and steering
 */
@TeleOp
public class MotorcycleMode extends OpMode{
    // Constants
    static final double TICK_SPEED = 8.0; // gamepad joystick tick increment (speed) multiplier for linear movement.
    static final double INITIAL_WHEEL_ANGLE = 0.0; // Wheel initial angle, in radians

    /* Declare OpMode members. */
    double priorTime = 0.0; // for logging loop times

    TwoWheelDiffSwerveClass drive;

    // Potentiometer Class
    TwoFullRotationPotClass pots;

    @Override
    public void init() {
        drive = new TwoWheelDiffSwerveClass();
        drive.initDrive(hardwareMap);

        pots = new TwoFullRotationPotClass();
        pots.initPots(hardwareMap);
        pots.getAngleFromPots(false,0); // find out where the wheels are pointed

        drive.initWheelAngles(pots.angle1, pots.angle2,INITIAL_WHEEL_ANGLE,INITIAL_WHEEL_ANGLE);  // set the wheels to desired angle

        // Send telemetry message to signify robot waiting;
        telemetry.addData("INITIAL POT 1 ="," %.05f, POT 2 = %.05f",pots.angle1,pots.angle2);
        pots.getAngleFromPots(false,0); // find out where the wheels are pointed
        telemetry.addData("NEW POT 1 ="," %.05f, POT 2 = %.05f",pots.angle1,pots.angle2);
        telemetry.addData(">", "Robot Ready.  Press Play.");

        // Write to log file
        RobotLog.d("SRA-start-Motorcycle_Mode = %.05f",getRuntime());
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        drive.setMotorsPower(1.0); // full speed  = 1.0. Backed off to prevent damage while developing
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double gpLeftX, gpRightY;
        double throttle;
        double loopTime;
        double podAngle;

        gpLeftX = gamepad1.left_stick_x;
        gpRightY = gamepad1.right_stick_y;
        gpRightY = gpRightY*gpRightY*gpRightY;  // cube the input to make it smoother

        podAngle = gpLeftX;  // 1 radian ~ 60 degrees, about right for max wheel turn

        throttle = gamepad1.right_trigger;

        drive.setRobotTranslation((int) ((1+throttle)* gpRightY * TICK_SPEED));
        drive.setMotorPositions(podAngle,-podAngle,0.0); // Moves the robot

        telemetry.addData("Right stick y = SPEED = ", gpRightY);
        telemetry.addData("Left stick x = TURN = ", gpLeftX);
        telemetry.addData("Right Trigger = THROTTLE = ", throttle);
        telemetry.update();

        loopTime = getRuntime() - priorTime;
        priorTime = getRuntime();

        RobotLog.d("SRA-loop-MOTORCYCLE = %.05f, loop = %.05f,targetPos = %d,podAngle = %.04f",priorTime,loopTime,drive.targetPosition,podAngle);
        //RobotLog.d("SRA-loop-gamepad = %.03f, %.03f",gamepad1.right_stick_x,gamepad1.right_stick_y);
        //RobotLog.d("SRA-loop-ENCODERS = %d, %d, %d, %d", motor1a.getCurrentPosition(), motor1b.getCurrentPosition(), motor2c.getCurrentPosition(), motor2d.getCurrentPosition());
        //RobotLog.d("SRA-loop-TARGET = %d, %d, %d, %d",targetPosA,targetPosB,targetPosC,targetPosD);
    }

    @Override
    public void stop() {
        RobotLog.d("SRA-end-MOTORCYCLE_MODE = %.05f",getRuntime());
    }
}
