package org.firstinspires.ftc.teamcode.teleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commons.RobotHardware;

public class Claw {

    public final RobotHardware robot;
    public final LinearOpMode opMode;

    // Constructor that takes RobotHardware as a parameter
    public Claw(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    public void controlClaw(boolean buttonA, boolean buttonY, boolean buttonX, boolean buttonB, boolean rightStickButton, boolean leftBumper, boolean rightBumper) {
            if (buttonA) {
                setClawPower(1,100);
            } else if (buttonY) {
                setClawPower(-1,100);
            }

            if (buttonX) {
                setClawRotation(0.5, 0.41); // Halfway
            } else if (buttonB) {
                setClawRotation(1, 0.09); // Down
            } else if (rightStickButton) {
                setClawRotation(0.2, 0.71); // Up
            }

            if (leftBumper) {
                setClawServo(0.3, 1);
            } else if (rightBumper) {
                setClawServo(1, 0.3);
            }
        }

        public void setClawServo ( double leftPos, double rightPos){

            robot.rightClawServo.setPosition(rightPos);
            robot.leftClawServo.setPosition(leftPos);
        }

        public void setClawPower ( double power, int time) {
            robot.leftClaw.setPower(power);
            robot.rightClaw.setPower(power);
            opMode.sleep(time);
            robot.leftClaw.setPower(0);
            robot.rightClaw.setPower(0);
        }

        public void setClawRotation ( double leftPos, double rightPos){
            robot.leftRotation.setPosition(leftPos);
            robot.rightRotation.setPosition(rightPos);
        }
    }
