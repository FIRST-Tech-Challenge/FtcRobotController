/*
Made by Aryan Sinha,
FTC team 202101101
 */

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.common.utils.DriveUtils.logData;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.HardwareNew;
import org.firstinspires.ftc.teamcode.common.BaseNewOpMode;


/**
 * This class handles the manual driving.
 * @soon-to-be-author karthikperi
 * @author aryansinha
 */
@TeleOp(name="Basic: Obese OpMode", group="Linear Opmode")
public class ManualGameOp extends BaseNewOpMode {
    private final HardwareNew robot = new HardwareNew(true);

    /**
     * {@inheritDoc}
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        final ElapsedTime runtime = new ElapsedTime();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean isBoostActive = false;
            double drive = -gamepad1.left_stick_y;
            double turn  = Range.clip(-gamepad1.right_stick_x, -0.4, 0.4);
            double leftPower = 0;
            double rightPower = 0;
            /*
            Boost Code
             */
            if (!isBoostActive)
            {
                leftPower = Range.clip(drive + turn, -0.8, 0.8);
                rightPower = Range.clip(drive - turn, -0.8, 0.8);
            }

            else if (isBoostActive)
            {
                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);
            }
            robot.getRightDrive().setPower(rightPower);
            robot.getBackRightDrive().setPower(rightPower);
            robot.getLeftDrive().setPower(leftPower);
            robot.getBackLeftDrive().setPower(leftPower);


            if (gamepad1.a)
            {

                if (!isBoostActive)
                {
                    isBoostActive = true;

                }
                else if (isBoostActive)
                {
                    isBoostActive = false;
                }
            }


            if (gamepad2.dpad_right) {
                telemetry.addLine("Pressed Right DPad");
                robot.getCarousel().setPower(.8);
                sleep(1000);
                robot.getCarousel().setPower(0);
                        }

            if (gamepad2.left_bumper) {
                robot.getRightClaw().setPosition(1.0);
                robot.getLeftClaw().setPosition(1.0);
            }
            else if (gamepad2.right_bumper)
            {
                robot.getRightClaw().setPosition(0.3);
                robot.getLeftClaw().setPosition(0.3);
            }

            if (gamepad2.left_trigger > 0)
            {
                robot.getArm().setPower(0.5);
                sleep(1);
                robot.getArm().setPower(0);
            }
            if (gamepad2.right_trigger > 0)
            {
                robot.getArm().setPower(-0.5);
                sleep(1);
                robot.getArm().setPower(0);
            }
            logData(this, "IMU", String.format("x: %f, y: %f, z: %f", robot.getImu().getAngularOrientation().firstAngle, robot.getImu().getAngularOrientation().secondAngle, robot.getImu().getAngularOrientation().thirdAngle) );
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public HardwareNew getRobot() {
        return robot;
    }
}