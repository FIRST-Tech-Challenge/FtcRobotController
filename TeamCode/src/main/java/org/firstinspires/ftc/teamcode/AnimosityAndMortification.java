package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public class AnimosityAndMortification extends Movable {
    //variables
    static double tgtPower = 0;
    static double tgtPower2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            tgtPower = this.gamepad1.left_stick_y/1.4;
            tgtPower2 = this.gamepad1.left_stick_x/1.4;

            // FRW is the only right moving wheel :(`
            if(gamepad1.left_stick_y > 0.3 || gamepad1.left_stick_y < -0.3) {
                //forward and backwards
                FLW.setPower(-tgtPower);
                BLW.setPower(-tgtPower);
                FRW.setPower(tgtPower);
                BRW.setPower(tgtPower);
            } else if (gamepad1.left_bumper) {
                // turning
                FLW.setPower(-0.6);
                BLW.setPower(-0.6);
                FRW.setPower(-0.6);
                BRW.setPower(-0.6);
            } else if (gamepad1.right_bumper) {
                FLW.setPower(0.6);
                BLW.setPower(0.6);
                FRW.setPower(0.6);
                BRW.setPower(0.6);
            } else if (gamepad1.left_trigger > .5) {
                // strafing, controls can be switched through inverse boolean
                if (!inverse) {
                    powerWheels(0, "left");
                } else {
                    powerWheels(0, "right");
                }
            } else if (gamepad1.right_trigger > .5) {
                if (!inverse) {
                    powerWheels(0, "right");
                } else {
                    powerWheels(0, "left");
                }
            } else {
                disablePower();
            }

            // scissor lift
            if(gamepad1.right_stick_y > 0.5){
                powerScissorLift(0, "ascend");
            }else if(gamepad1.right_stick_y < -0.5) {
                powerScissorLift(0, "descend");
            }else {
                disableScissorPower();
            }

            // slides/arms
            if (gamepad1.b) {
                moveSlides("thrust");
            } else if (gamepad1.a) {
                moveSlides("retract");
            }else if (gamepad1.x) {
                // inverses control
                inverseControls();
            }

            if (gamepad1.dpad_left) {
                if (!inverse) {
                    turn90("left");
                } else {
                    turn90("right");
                }
            } else if (gamepad1.dpad_right) {
                if (!inverse) {
                    turn90("right");
                } else {
                    turn90("left");
                }            }
            updatePhoneConsole();
        }
    }

    public void updatePhoneConsole() {
        telemetry.addData("Status","Running");
        telemetry.addData("Inverse", inverse);
        telemetry.addData("Target Power:", tgtPower);
        telemetry.addData("FLW Power:", FLW.getPower());
        telemetry.addData("BLW Power:", BLW.getPower());
        telemetry.addData("FRW Power:", FRW.getPower());
        telemetry.addData("BRW Power:", BRW.getPower());
        telemetry.addData("FRScissorLift Power:", FRScissorLift.getPower());
        telemetry.addData("BRScissorLift Power:", BRScissorLift.getPower());
        telemetry.addData("FLScissorLift Power:", FLScissorLift.getPower());
        telemetry.addData("BLScissorLift Power:", BLScissorLift.getPower());
        telemetry.addData("Left Stick X:", tgtPower2);
        telemetry.addData("Left Stick Y:", tgtPower);
        telemetry.update();
    }
}