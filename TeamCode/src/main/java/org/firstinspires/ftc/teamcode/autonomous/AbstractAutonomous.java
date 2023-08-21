package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.UpdatedClawbotHardware;

public abstract class AbstractAutonomous extends LinearOpMode {
    public UpdatedClawbotHardware robot = new UpdatedClawbotHardware();
    public ElapsedTime timer = new ElapsedTime();

    public void move() {
        move(5);
    }

    public void move(int seconds) {
        timer.reset();
        robot.leftDrive.setPower(.75);
        robot.rightDrive.setPower(.75);
        while (timer.seconds() < seconds) {

        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public void openClaw() {
        robot.claw.setPosition(UpdatedClawbotHardware.CLAW_OPEN_POSITION);
    }

    public void closeClaw() {
        robot.claw.setPosition(UpdatedClawbotHardware.CLAW_CLOSED_POSITION);
    }

    public void turnLeft() {
        turnLeft(3);
    }

    public void turnLeft(int seconds) {
        timer.reset();
        robot.rightDrive.setPower(.75);
        while (timer.seconds() < seconds) {

        }
        robot.rightDrive.setPower(0);
    }

    public void turnRight() {
        turnLeft(3);
    }

    public void turnRight(int seconds) {
        timer.reset();
        robot.leftDrive.setPower(.75);
        while (timer.seconds() < seconds) {

        }
        robot.leftDrive.setPower(0);
    }

}
