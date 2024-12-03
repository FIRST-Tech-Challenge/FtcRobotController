package org.firstinspires.ftc.teamcode.teleOp;

import org.firstinspires.ftc.teamcode.commons.RobotHardware;

public class Claw {

    public final RobotHardware robot;

    // Constructor that takes RobotHardware as a parameter
    public Claw(RobotHardware robot) {
        this.robot = robot;
    }

    public void controlClaw(boolean buttonA, boolean buttonY, boolean buttonX, boolean buttonB, boolean rightStickButton, boolean leftBumper, boolean rightBumper) {
            if (buttonA) {
                setClawPower(1);
            } else if (buttonY) {
                setClawPower(-1);
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

        private void setClawServo ( double leftPos, double rightPos){

            robot.rightClawServo.setPosition(rightPos);
            robot.leftClawServo.setPosition(leftPos);
        }

        private void setClawPower ( double power) {
            robot.leftClaw.setPower(power);
            robot.rightClaw.setPower(power);
            try {
                Thread.sleep(100); // Short pause for smooth movement
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            robot.leftClaw.setPower(0);
            robot.rightClaw.setPower(0);
        }

        private void setClawRotation ( double leftPos, double rightPos){
            robot.leftRotation.setPosition(leftPos);
            robot.rightRotation.setPosition(rightPos);
        }
    }
