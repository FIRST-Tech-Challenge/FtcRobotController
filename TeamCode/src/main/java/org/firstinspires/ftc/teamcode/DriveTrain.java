package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain {
    private Robot robot;
    private volatile Gamepad gamepad;

    private double normal_speed = 1.0;
    private double medium_speed = 0.5;
    private double slow_speed = 0.2;

    public DriveTrain(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    private void calculateSpeed(double drive, double strafe, double twist, double speed_factor, double speeds[])
    {
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
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
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

    private void drive(double speed_factor)
    {
        double[] speeds = new double[4];

        if (gamepad.left_stick_y == 0 && gamepad.left_stick_x == 0 && gamepad.right_stick_x == 0) {
            return;
        }

        calculateSpeed(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, speed_factor, speeds);
        robot.motorFL.setPower(speeds[0]);
        robot.motorFR.setPower(speeds[1]);
        robot.motorBL.setPower(speeds[2]);
        robot.motorBR.setPower(speeds[3]);
    }

    public void drive_normal()
    {
        drive(normal_speed);
    }

    public void drive_medium()
    {
        drive(medium_speed);
    }

    public void drive_slow()
    {
        double[] speeds = new double[4];

        double y = 0;
        double x = 0;

        if (gamepad.dpad_up) {
            y = 0.2;
        } else if (gamepad.dpad_down) {
            y = -0.2;
        } else if (gamepad.dpad_left) {
            x = -0.4;
        } else if (gamepad.dpad_right) {
            x = 0.4;
        }

        if (x == 0 && y == 0) {
            return;
        }

        calculateSpeed(y, x, 0, normal_speed, speeds);
        robot.motorFL.setPower(speeds[0]);
        robot.motorFR.setPower(speeds[1]);
        robot.motorBL.setPower(speeds[2]);
        robot.motorBR.setPower(speeds[3]);
    }

    public void drive() {
        if (gamepad.left_stick_y != 0 || gamepad.left_stick_x != 0 || gamepad.right_stick_x != 0) {
            if (gamepad.left_trigger > 0 || gamepad.right_trigger > 0) {
                drive_medium();
            } else {
                drive_normal();
            }
        } else if (gamepad.dpad_left || gamepad.dpad_right || gamepad.dpad_down || gamepad.dpad_up) {
            drive_slow();
        }
    }
}
