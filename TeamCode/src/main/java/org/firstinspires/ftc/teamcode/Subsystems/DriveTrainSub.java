package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainSub {
    public DcMotor leftFront, rightFront, rightRear, leftRear;
    public Gamepad gamepad;
    public HardwareMap hardwareMap;
    double y = -gamepad1.left_stick_y;
    double x = gamepad1.left_stick_x;
    double rx = gamepad1.right_stick_x;

    public DriveTrainSub(OpMode opMode) {
            this.gamepad = opMode.gamepad1;
            this.hardwareMap = opMode.hardwareMap;

            leftFront = (DcMotor) hardwareMap.get("LeftFront");
            rightFront = (DcMotor) hardwareMap.get("RightFront");
            rightRear = (DcMotor) hardwareMap.get("RightRear");
            leftRear = (DcMotor) hardwareMap.get("LeftRear");
        }

        public void teleOp() {
            if (gamepad1.right_bumper) {
                poweringMotors(0.5);
            }
            else poweringMotors(1);
        }
        public void poweringMotors(double multiplier) {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator) * multiplier;
            double backLeftPower = ((y - x + rx) / denominator) * multiplier;
            double frontRightPower = ((y - x - rx) / denominator) * multiplier;
            double backRightPower = ((y + x - rx) / denominator) * multiplier;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
        }
    }
