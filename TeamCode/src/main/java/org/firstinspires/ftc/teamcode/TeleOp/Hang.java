package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOp.RobotHardware;


public class Hang {

    public final RobotHardware robot;
    private final LinearOpMode opMode;

    // Constructor that takes RobotHardware as a parameter
    public Hang(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    // Hang Programming

    public void controlHang(boolean dpad_down, boolean dpad_up) {
        if (dpad_down) {
            setHangPower(1, 100); // Hang Down
        } else if (dpad_up) {
            setHangPower(-1, 100); // Hang Up
        }
    }

    public void setHangPower(double power, int time) {
        robot.leftHang.setPower(power);
        robot.rightHang.setPower(power);
        opMode.sleep(time);
        robot.leftHang.setPower(0);
        robot.rightHang.setPower(0);
    }
}