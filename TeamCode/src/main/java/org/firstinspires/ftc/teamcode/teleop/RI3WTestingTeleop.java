package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name="RI3W Testing Teleop", group="11588")
public class RI3WTestingTeleop extends OpMode {
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    double turningPower = 0;
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap);
    }

    @Override
    public void loop() {
        gamepad.update();
        if (gamepad.a.isPressed()) {
            robot.frontRight.setPower(1);
        } else {
            robot.frontRight.setPower(0);
        }
        if (gamepad.b.isPressed()) {
            robot.backRight.setPower(1);
        } else {
            robot.backRight.setPower(0);
        }
        if (gamepad.x.isPressed()) {
            robot.backLeft.setPower(1);
        } else {
            robot.backLeft.setPower(0);
        }
        if (gamepad.y.isPressed()) {
            robot.frontLeft.setPower(1);
        } else {
            robot.frontLeft.setPower(0);
        }
//        telemetry.addData("Front Left Power", robot.frontLeft.getPower());
//        telemetry.addData("Front Right Power", robot.frontRight.getPower());
//        telemetry.addData("Back Left Power", robot.backLeft.getPower());
//        telemetry.addData("Back Right Power", robot.backRight.getPower());
//        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
//        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
//        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
//        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
//        telemetry.addData("Robot Angle", robot.getAngle());
        telemetry.update();
    }
}
