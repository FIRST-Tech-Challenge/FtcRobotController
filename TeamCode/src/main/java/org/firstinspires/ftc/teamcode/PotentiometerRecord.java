
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
    static final double INITIAL_WHEEL_ANGLE = 0.0; // Wheel angle in radians
    static final boolean CW = true; // If true, wheel rotation is CW, else CCW

    /* Declare OpMode members. */
    double priorTime = 0.0; // for logging loop times

    TwoWheelDiffSwerveClass drive;

    // Potentiometer Class
    public TwoFullRotationPotClass pots;

    // ROBOT GLOBALS
    // target positions for robot
    double currentAngle = INITIAL_WHEEL_ANGLE;  // the robot angle

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
        telemetry.addData(">", "Robot Ready.  Press Play.");

        // Write to log file
        RobotLog.d("SRA-start-PotentiometerRecord = %.05f",getRuntime());
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        drive.setMotorsPower(0.8); // full speed  = 1.0. Backed off to prevent damage while developing
        drive.setRobotTranslation(0);
    }

    @Override
    public void loop() {
        double loopTime;
        boolean log;

        if (CW) {
            if(currentAngle<=(4.1*Math.PI)) currentAngle += 0.03;  // spin a bit more than 2 revs, increment angle
            log = ((currentAngle>=(2.0*Math.PI)) && (currentAngle<=(4.0*Math.PI))); // only log 2nd rotation
        } else {
            if(currentAngle>=(-4.1*Math.PI)) currentAngle -= 0.03;  // spin a bit more than 2 revs, increment angle
            log = ((currentAngle<=(-2.0*Math.PI)) && (currentAngle>=(-4.0*Math.PI))); // only log 2nd rotation
        }
        drive.setMotorPositions(currentAngle,currentAngle,0.0); // Moves the robot

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
