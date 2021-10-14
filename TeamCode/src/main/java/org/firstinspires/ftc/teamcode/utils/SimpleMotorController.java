package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Represents a simple control plane for autonomous motors on the robot.
 * @author Thomas Ricci
 */
@Deprecated
public class SimpleMotorController {

    private final Telemetry TELEMETRY;
    private final ElapsedTime TIME = new ElapsedTime();

    /**
     * Creates a new control plane for the motors of the robot.
     * @param telemetry The telemetry object to log to.
     */
    public SimpleMotorController(Telemetry telemetry) {
        TELEMETRY = telemetry;
    }

    /**
     * Attempts to move a single motor a specific distance within a given amount of time.
     * @param motor The motor to move.
     * @param distance The distance the motor should spin, or move, by.
     * @param speed The speed at which the motor should move.
     * @param timeout The time in seconds in which the motor will give up moving if it hasn't completed moving.
     */
    public void moveSingleMotor(Motor motor, double distance, double speed, double timeout) {
        DcMotor dcMotor = motor.getDcMotor();
        dcMotor.setTargetPosition(dcMotor.getCurrentPosition() + (int)(distance * motor.getCountsPerInch()));
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TIME.reset();
        dcMotor.setPower(Math.abs(speed));
        while(TIME.seconds() < timeout && dcMotor.isBusy()) {
            TELEMETRY.update();
        }
        dcMotor.setPower(0);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TELEMETRY.addData("Run complete and motor ready for next run",  "Position is " + dcMotor.getCurrentPosition());
        TELEMETRY.update();
    }

    /**
     * Attempts to move multiple motors a specific distance in parallel within a given amount of time.
     * @param motors The motors to move.
     * @param distance The distance the motors should spin, or move, by.
     * @param speed The speed at which the motors should move.
     * @param timeout The time in seconds in which the motors will give up moving if they haven't completed moving.
     */
    public void moveGroupedMotors(Motor[] motors, double distance, double speed, double timeout) {
        for(Motor motor : motors) {
            DcMotor dcMotor = motor.getDcMotor();
            dcMotor.setTargetPosition(dcMotor.getCurrentPosition() + (int)(distance * motor.getCountsPerInch()));
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Reset the timeout time and start motion
        TIME.reset();
        for(Motor motor : motors) {
            DcMotor dcMotor = motor.getDcMotor();
            dcMotor.setPower(Math.abs(speed));
        }
        while(TIME.seconds() < timeout && motorsAreBusy(motors)) {
            TELEMETRY.update();
        }

        // Cleanup motors and prep for next motion
        for(Motor motor : motors) {
            DcMotor dcMotor = motor.getDcMotor();
            dcMotor.setPower(0);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        TELEMETRY.addData("Run complete and motor ready for next run",  "Current position of first motor:" + motors[0].getDcMotor().getCurrentPosition());
        TELEMETRY.update();
    }

    /**
     * Attempts to move multiple motors specific distances in parallel within a given amount of time.
     * @param motors The motors to move.
     * @param distances The distances the motors should spin, or move, by. The values at each array index correlate with the motors indexes, meaning distances[0] defines the distance motors[0] will move.
     * @param speed The speed at which the motors should move.
     * @param timeout The time in seconds in which the motors will give up moving if they haven't completed moving.
     */
    public void moveMultipleMotors(Motor[] motors, double[] distances, double speed, double timeout) {
        for(int i = 0; i < motors.length; i++) {
            Motor motor = motors[i];
            DcMotor dcMotor = motor.getDcMotor();
            dcMotor.setTargetPosition(dcMotor.getCurrentPosition() + (int)(distances[i] * motor.getCountsPerInch()));
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Reset the timeout time and start motion
        TIME.reset();
        for(Motor motor : motors) {
            DcMotor dcMotor = motor.getDcMotor();
            dcMotor.setPower(Math.abs(speed));
        }
        while(TIME.seconds() < timeout && motorsAreBusy(motors)) {
            TELEMETRY.update();
        }

        // Cleanup motors and prep for next motion
        for(Motor motor : motors) {
            DcMotor dcMotor = motor.getDcMotor();
            dcMotor.setPower(0);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        TELEMETRY.addData("Run complete and motor ready for next run",  "Current position of first motor: " + motors[0].getDcMotor().getCurrentPosition());
        TELEMETRY.update();
    }

    /**
     * Attempts to kill the motor even if it's still running.
     * @param motor The motor to kill.
     */
    public void killMotor(Motor motor) {
        DcMotor dcMotor = motor.getDcMotor();
        if(dcMotor.getPower() > 0) {
            dcMotor.setPower(0);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TELEMETRY.addData("Motor has been killed successfully and is ready for next run", "Current position: " + dcMotor.getCurrentPosition());
        }else{
            TELEMETRY.addData("Motor was not running, so it was not killed", "Current position: " + dcMotor.getCurrentPosition());
        }
        TELEMETRY.update();
    }

    /**
     * Attempts to kill multiple motors even if they're still running.
     * @param motors The motors to kill.
     */
    public void killMotors(Motor[] motors) {
        for(Motor motor : motors) {
            DcMotor dcMotor = motor.getDcMotor();
            if(dcMotor.getPower() > 0) {
                dcMotor.setPower(0);
                dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                TELEMETRY.addData("Motor has been killed successfully and is ready for next run", "Current position: " + dcMotor.getCurrentPosition());
            }else{
                TELEMETRY.addData("Motor was not running, so it was not killed", "Current position: " + dcMotor.getCurrentPosition());
            }
            TELEMETRY.update();
        }
        TELEMETRY.addData("All motors killed if possible and ready for next run", "Current position of first motor: " + motors[0].getDcMotor().getCurrentPosition());
        TELEMETRY.update();
    }

    private boolean motorsAreBusy(Motor[] motors) {
        boolean busy = false;
        for(Motor motor : motors) {
            if(!busy && motor.getDcMotor().isBusy()) {
                busy = true;
            }
        }
        return busy;
    }

}
