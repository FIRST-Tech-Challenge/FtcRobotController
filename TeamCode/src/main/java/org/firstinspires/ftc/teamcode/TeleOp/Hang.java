package org.firstinspires.ftc.teamcode.TeleOp;

public class Hang {

    public final RobotHardware robot;

    // Constructor that takes RobotHardware as a parameter
    public Hang(RobotHardware robot) {
        this.robot = robot;
    }

    // Hang Programming

    public void controlHang(boolean dpad_down, boolean dpad_up) {
        if (dpad_down) {
            setHangPower(1); // Hang Down
        } else if (dpad_up) {
            setHangPower(-1); // Hang Up
        }
    }

    private void setHangPower(double power) {
        robot.leftHang.setPower(power);
        robot.rightHang.setPower(power);
        try {
            Thread.sleep(100); // Short pause for smooth movement
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        robot.leftHang.setPower(0);
        robot.rightHang.setPower(0);
    }
}