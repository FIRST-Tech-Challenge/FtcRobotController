package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DriveTrain {
    private Robot robot = null;
    private volatile Gamepad gamepad = null;

    private double normal_speed = 0.8;
    private double medium_speed = 0.3;
    private double slow_speed = 0.2;
    private boolean verbose = false;

    public DriveTrain(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    private void calculateSpeed(double drive, double strafe, double twist, double speed_factor, double speeds[]) {
        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        speeds[0] = (drive + strafe + twist);
        speeds[1] = (drive - strafe - twist);
        speeds[2] = (drive - strafe + twist);
        speeds[3] = (drive + strafe - twist);

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        speeds[0] *= speed_factor;
        speeds[1] *= speed_factor;
        speeds[2] *= speed_factor;
        speeds[3] *= speed_factor;

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
    }

    public void stopDrive() {
        //sampleDrive.setMotorPowers(double FL, double BL, double BR, double FR);
        robot.sampleDrive.setMotorPowers(0, 0, 0, 0);
    }

    private void drive(double left_y, double left_x, double right_x, double speed_factor) {
        double[] speeds = new double[4];

        if (left_y == 0 && left_x == 0 && right_x == 0) {
            stopDrive();
            return;
        }

        calculateSpeed(left_y, -left_x, -right_x, speed_factor, speeds);
        log("SPEED0:", speeds[0]);
        log("SPEED1:", speeds[1]);
        log("SPEED2:", speeds[2]);
        log("SPEED3:", speeds[3]);
        //sampleDrive.setMotorPowers(double FL, double BL, double BR, double FR);
        robot.sampleDrive.setMotorPowers(-speeds[0], -speeds[2], -speeds[3], -speeds[1]);
        /*
        robot.motorFL.setPower(speeds[0]);
        robot.motorFR.setPower(speeds[1]);
        robot.motorBL.setPower(speeds[2]);
        robot.motorBR.setPower(speeds[3]);
         */
    }

    public void drive_normal() {
        drive(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, normal_speed);
    }

    public void drive_medium() {
        drive(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, medium_speed);
    }

    public void drive_slow() {


        double[] speeds = new double[4];

        double y = 0;
        double x = 0;

        if (gamepad.dpad_up) {
            y = -0.2;
        } else if (gamepad.dpad_down) {
            y = 0.2;
        } else if (gamepad.dpad_left) {
            x = 0.36;
        } else if (gamepad.dpad_right) {
            x = -0.36;
        }

        if (x == 0 && y == 0) {
            return;
        }
        drive(y, x, 0.0, normal_speed);
    }

    private void log(String s, Double d) {
        if (verbose) {
            robot.telemetry.addData(s, d);
        }
    }

    private void logUpdate() {
        if (verbose) {
            robot.telemetry.update();
        }
    }

    public void drive() {

        if (gamepad.left_stick_y != 0 || gamepad.left_stick_x != 0 || gamepad.right_stick_x != 0) {
            if (gamepad.left_trigger > 0 || gamepad.right_trigger > 0) {
                log("DriveTrain: Medium Speed: ", medium_speed);
                drive_medium();
            } else {
                log("DriveTrain: Normal Speed:", normal_speed);
                drive_normal();
            }
        } else if (gamepad.dpad_left) {
            robot.sampleDrive.turn(Math.toRadians(90));
        } else if (gamepad.dpad_right) {
            robot.sampleDrive.turn(Math.toRadians(-90));
        } else if (gamepad.dpad_down || gamepad.dpad_up) {
            robot.sampleDrive.turn(Math.toRadians(180));
            //drive_slow();
        } else {
            log("DriveTrain: StopDrive", 0.0);
            stopDrive();
        }
        logUpdate();
    }

    public void timed_drive(double y, double x, double rx, double speed, double time) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();

        // while (opModeIsActive() &&  runTime.time() > time) {
        while (runTime.time() < time) {
            drive(y, x, 0, speed);
            log("Time:", runTime.time());
            logUpdate();
        }
        stopDrive();
        log("Done Time:", runTime.time());
        logUpdate();
    }


    /*
    static final int ticks_per_mm = 2000 / 48;  //2000 ticks per one rev(48mm) of Gobilda pod
    static final double autonomous_power = 0.1;

    private void drive_forward_reverse(int distance, boolean reverse) {
        int target_ticks = ticks_per_mm * distance;
        int cur_ticks = 0;
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (true) {
            cur_ticks = robot.motorFR.getCurrentPosition();
            if (Math.abs(cur_ticks) > target_ticks) {
                break;
            }
            drive((reverse) ? autonomous_power: -autonomous_power, 0, 0, 1.0);
            log("Driving Cur ticks:", (double)cur_ticks);
            logUpdate();
        }
        stopDrive();
        log("Drive Done Cur ticks:", (double) robot.motorFR.getCurrentPosition());
        logUpdate();
    }

    public void drive_forward(int distance) {
        drive_forward_reverse(distance, false);
    }
    public void drive_reverse(int distance) {
        drive_forward_reverse(distance, true);
    }

    private void strafe_left_right(int distance, boolean left) {
        int target_ticks = ticks_per_mm * distance;
        int cur_ticks;
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // while (opModeIsActive() &&  runTime.time() > time) {
        while (true) {
            cur_ticks = robot.motorFL.getCurrentPosition();
            if (Math.abs(cur_ticks) > target_ticks) {
                break;
            }
            drive(0, (left) ? -autonomous_power: autonomous_power, 0, 1.0);
            log("Strafe Cur ticks:", (double)cur_ticks);
            log("Strafe Cur ticks:", (double)Math.abs(cur_ticks));
            log("Strafe target ticks:", (double)target_ticks);
            logUpdate();
        }
        stopDrive();
        log("Drive Done Cur ticks:", (double) robot.motorFL.getCurrentPosition());
        logUpdate();
    }

    public void strafe_left(int distance) {
        strafe_left_right(distance, true);
    }
    public void strafe_right(int distance) {
        strafe_left_right(distance, false);
    }

    private void twist_left_right(double degrees, boolean left) {
        robot.resetYaw();
        double cur_degrees;

        // while (opModeIsActive() &&  runTime.time() > time) {
        while (true) {
            cur_degrees = robot.getYaw();
            if (cur_degrees > degrees) {
                break;
            }
            drive(0, 0, (left) ? -autonomous_power: autonomous_power, 1.0);
        }
        stopDrive();
        log("CUR DEGREES", cur_degrees);
        logUpdate();
    }

    public void twist_left(double degrees) {
        twist_left_right(degrees, true);
    }
    public void twist_right(double degrees) {
        twist_left_right(degrees, false);
    }
     */

}


