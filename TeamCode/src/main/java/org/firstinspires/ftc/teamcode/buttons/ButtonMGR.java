package org.firstinspires.ftc.teamcode.buttons;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.buttons.dpad.Left;
import org.firstinspires.ftc.teamcode.buttons.dpad.Right;
import org.firstinspires.ftc.teamcode.buttons.dpad.Up;
import org.firstinspires.ftc.teamcode.driver.EncoderDrive;
import org.firstinspires.ftc.teamcode.imgproc.ImgProc;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Objects;

public class ButtonMGR {
    public Robot robot;
    public ImgProc image;
    private Telemetry telemetry;
    public EncoderDrive encoder;

    public ButtonMGR(Telemetry telemetry, Robot robot) {
        this.telemetry = telemetry;
        encoder = new EncoderDrive(robot);
        image = new ImgProc();
        this.robot = robot;
    }

    public void CheckAndExecute() {
        Gamepad gamepad = robot.gamepad1;
        if (gamepad.dpad_up) {
            this.dpadUp();
        }

        if (gamepad.dpad_right) {
            this.dpadRight();
        }

        if (gamepad.dpad_left) {
            this.dpadLeft();
        }

        if (gamepad.left_bumper) {
            this.leftBumper();
        }

        gamepad = robot.gamepad2;
        if (gamepad.dpad_up) {
            this.dpadUp();
        }

        if (gamepad.dpad_right) {
            this.dpadRight();
        }

        if (gamepad.dpad_left) {
            this.dpadLeft();
        }

        if (gamepad.left_bumper) {
            this.leftBumper();
        }
    }

    private void dpadUp() {
        this.encoder = new Up(this.robot).encoder;
    }

    private void dpadRight() {
        this.image = new ImgProc();
        new Right(this.telemetry, this.image);
    }

    private void dpadLeft() {
        new Left(this.image);
    }

    private void leftBumper() {
        new org.firstinspires.ftc.teamcode.buttons.bumpers.Left(this.encoder);
    }
}
