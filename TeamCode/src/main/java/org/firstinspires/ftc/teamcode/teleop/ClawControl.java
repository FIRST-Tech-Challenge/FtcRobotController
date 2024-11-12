package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Claw;

@TeleOp(name = "ClawControl")
public class ClawControl extends OpMode {
    Claw claw;

    @Override
    public void init() {
        this.claw = new Claw(
            hardwareMap.get(Servo.class, "ClawRotLeft"),
            hardwareMap.get(Servo.class, "ClawRotRight"),
            hardwareMap.get(Servo.class, "ClawArmLeft"),
            hardwareMap.get(Servo.class, "ClawArmRight"));
    }

    @Override
    public void loop() {
        if (this.gamepad2.left_bumper) {
            this.claw.openLeft();
        } else {
            this.claw.closeLeft();
        }
        if (this.gamepad2.right_bumper) {
            this.claw.openRight();
        } else {
            this.claw.closeRight();
        }
        if (!this.gamepad2.guide) {
            if (this.gamepad2.y) {
                this.claw.setRotate(0.5);
            } else if (this.gamepad2.b) {
                this.claw.setRotate(0);
            } else {
                this.claw.rotate(this.gamepad2.right_stick_y * 0.01);
            }
        }
        this.telemetry.addData("LeftRot", this.claw.leftRot.getPosition());
        this.telemetry.addData("RightRot", this.claw.rightRot.getPosition());
        this.telemetry.addData("LeftClaw", this.claw.leftClaw.getPosition());
        this.telemetry.addData("RightClaw", this.claw.rightClaw.getPosition());
        this.telemetry.update();
    }
}
