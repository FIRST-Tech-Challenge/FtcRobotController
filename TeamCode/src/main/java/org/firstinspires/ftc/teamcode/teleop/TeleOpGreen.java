package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

@TeleOp(name = "TeleOp Main", group = "Main")
public class TeleOpGreen extends GreenBotBaseOpMode {

    private final int INITIAL_LAUNCHER_TARGET_VELOCITY = 1250;
    private final int MIN_LAUNCHER_TARGET_VELOCITY = 200;
    private final int MAX_LAUNCHER_TARGET_VELOCITY = 2000;

    private final double INTAKE_POWER = 1.0;
    private final double FEEDER_POWER = 1.0;

    private boolean intakeOn = false;
    private double launcherTargetVelocity = INITIAL_LAUNCHER_TARGET_VELOCITY;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();


        helpMenu();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            /////////////////////////
            // handle driving
            /////////////////////////
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drive.drive(forward, strafe, turn);

            /////////////////////////
            // handle intake
            /////////////////////////
            if (gamepad1.leftBumperWasPressed()) {
                intakeOn = !intakeOn;
            }
            double intakePower = 0;
            if (intakeOn) {
                intakePower = INTAKE_POWER;
            } // else stays at 0
            intake.setPower(intakePower);
            leftIntakeServo.setPower(intakePower);
            rightIntakeServo.setPower(intakePower);

            /////////////////////////
            // handle feeder - only runs when buttons are held
            /////////////////////////
            if (gamepad1.y) {
                feeder.setPower(FEEDER_POWER);
            } else if (gamepad1.a) {
                feeder.setPower(-FEEDER_POWER);
            } else {
                feeder.setPower(0.0);
            }

            /////////////////////////
            // handle launcher - runs continuously
            /////////////////////////
            if (gamepad1.dpadUpWasPressed()) {
                launcherTargetVelocity += 100;
                if (launcherTargetVelocity > MAX_LAUNCHER_TARGET_VELOCITY) {
                    launcherTargetVelocity = MAX_LAUNCHER_TARGET_VELOCITY;
                }
            }
            if (gamepad1.dpadDownWasPressed()) {
                launcherTargetVelocity -= 100;
                if (launcherTargetVelocity < MIN_LAUNCHER_TARGET_VELOCITY) {
                    launcherTargetVelocity = MIN_LAUNCHER_TARGET_VELOCITY;
                }
            }
            launcher.setVelocity(launcherTargetVelocity);

            /////////////////////////
            // update telemetry
            /////////////////////////
            if (gamepad1.start) {
                helpMenu();
            } else {
                double launcherVelocity = launcher.getVelocity();
                telemetry.addData("Launcher - Target Velocity", launcherTargetVelocity);
                telemetry.addData("Launcher - Current Velocity", launcherVelocity);
            }
            telemetry.update();
        }
    }

    private void helpMenu() {
        telemetry.addData("Drive controls", "Standard Mecanum");
        telemetry.addData("Intake Toggle", "Left Bumper");
        telemetry.addData("Run Launcher", "FWD: Hold Y, REV: Hold A");
        telemetry.addData("Launcher +", "DPad Up");
        telemetry.addData("Launcher - ", "DPad Down");
    }
}