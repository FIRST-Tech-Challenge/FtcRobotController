package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "GiGiEncoderTest")
public class GiGiEncoderTest extends LinearOpMode {

    private DcMotor Arm;
    private DcMotor MotoMoto;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        MotoMoto = hardwareMap.get(DcMotor.class, "MotoMoto");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    // Set the target position for the arm motor
                    Arm.setTargetPosition(10000);
                    // Set the arm motor to run to the target encoder position
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // Set the power for the arm motor
                    ((DcMotorEx) Arm).setVelocity(10000); // Adjust power level as needed

                    // Check if the arm motor is still busy running to the target position
                    while (opModeIsActive() && Arm.isBusy()) {
                        // Update telemetry or perform other actions if needed
                        telemetry.addData("Arm Encoder Position", Arm.getCurrentPosition());
                        telemetry.update();
                        idle();
                         if (gamepad1.right_stick_y != 0) {
                            MotoMoto.setPower(gamepad1.right_stick_y);
                        }

                        else {
                            MotoMoto.setPower(0);
                        }
                    }
                    telemetry.addData("", Arm.getCurrentPosition());
                    telemetry.update();

                    // Stop the arm motor after reaching the target position
                    ((DcMotorEx) Arm).setVelocity(0);
                    Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Switch back to using encoders
                }

                else if (gamepad1.right_stick_y != 0) {
                    MotoMoto.setPower(gamepad1.right_stick_y);
                }

                else {
                    MotoMoto.setPower(0);
                }
            }
        }
    }
}