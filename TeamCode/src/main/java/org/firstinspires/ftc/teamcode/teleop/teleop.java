/*
 Made by Aryan Sinha,
 FTC team 202101101
  */

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.Hardware2;


/**
 * This class handles the manual driving.
 * @author aryansinha
 * also in a less prominent fashion
 * @soon-to-be-author author karthikperi
 */
@TeleOp(name="Basic: Fat OpMode", group="Linear Opmode")
public class teleop extends BaseOpMode {
    private final Hardware2 robot = new Hardware2(false);

    /**
     * {@inheritDoc}
     */

    public void runOpMode() throws InterruptedException {
        robot.initTeleOpIMU(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        final ElapsedTime runtime = new ElapsedTime();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn, -0.8, 0.8) ;
            double rightPower = Range.clip(drive - turn, -0.8, 0.8) ;

            robot.frontRightMotor.setPower(rightPower);
            robot.backRightMotor.setPower(rightPower);
            robot.frontLeftMotor.setPower(leftPower);
            robot.backLeftMotor.setPower(leftPower);

            if (gamepad2.left_bumper) {
                robot.getLeftClaw().setPosition(-1);
                robot.getRightClaw().setPosition(1);
            }
            else if (gamepad2.right_bumper) {
                robot.getLeftClaw().setPosition(0.3);
                robot.getRightClaw().setPosition(-0.3);
            }

            if (gamepad2.left_trigger > 0.2) {
                robot.getArm().setPower(-0.7);
            } else if (gamepad2.right_trigger > 0.3) {
                robot.getArm().setPower(0.5);
            } else {
                robot.getArm().setPower(0);
            }


            if (gamepad1.x) {
                robot.frontRightMotor.setPower(1);
                robot.backRightMotor.setPower(-1);
                robot.frontLeftMotor.setPower(-1);
                robot.backLeftMotor.setPower(1);
            } else if (gamepad1.b) {
                robot.frontRightMotor.setPower(-1);
                robot.backRightMotor.setPower(1);
                robot.frontLeftMotor.setPower(1);
                robot.backLeftMotor.setPower(-1);
            }
        }
    }

    /**
     * {@inheritDoc}
     * @return
     */
    public Hardware2 getRobot() {
        return robot;
    }
}