
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TwoWheelDiffSwerveClass.TICKS_PER_HALF_WHEEL_TURN;

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
    static final double INITIAL_WHEEL_ANGLE = 0.0; // Wheel initial angle, in radians

    /* Declare OpMode members. */
    double priorTime = 0.0; // for logging loop times

    TwoWheelDiffSwerveClass drive;

    // Potentiometer Class
    TwoFullRotationPotClass pots;


    @Override
    public void init() {
        double deltaAngle1;
        double deltaAngle2;

        drive = new TwoWheelDiffSwerveClass();
        drive.initDrive(hardwareMap);

        pots = new TwoFullRotationPotClass();
        pots.initPots(hardwareMap);

        pots.getAngleFromPots(false,0); // find out where the wheel are pointed

        deltaAngle1 = drive.getShortestTurnAngle(pots.angle1,INITIAL_WHEEL_ANGLE);
        deltaAngle2 = drive.getShortestTurnAngle(pots.angle2,INITIAL_WHEEL_ANGLE);
        drive.initWheelAngles(deltaAngle1,deltaAngle2);  // set the wheels to desired angle

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double gpLeftX, gpRightY;
        double loopTime;
        double podAngle;
        double turnPod1, turnPod2;

        gpLeftX = gamepad1.left_stick_x;
        gpRightY = gamepad1.right_stick_y;
        gpRightY = gpRightY*gpRightY*gpRightY;  // cube the input to make it smoother

        podAngle = gpLeftX;
        turnPod1 = podAngle;
        turnPod2 = -podAngle;

        drive.setRobotTranslation((int) (gpRightY * TICK_SPEED));
        drive.setMotorPositions((int)(turnPod1* TICK_TURN),(int)(turnPod2* TICK_TURN),0);

        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
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
