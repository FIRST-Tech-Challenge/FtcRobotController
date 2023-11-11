package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GiGiMechDrive extends OpMode {

    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor Arm;
    private DcMotor Linear;
    private Servo Claw;


    public void init() {

        BLeft = hardwareMap.dcMotor.get("BLeft");
        BRight = hardwareMap.dcMotor.get("BRight");
        FLeft  = hardwareMap.dcMotor.get("FLeft");
        FRight = hardwareMap.dcMotor.get("FRight");
        Arm = hardwareMap.dcMotor.get("Arm");
        Linear = hardwareMap.dcMotor.get("Linear");
        Claw = hardwareMap.servo.get("Claw");

        FRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Linear.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void loop() {

        //Moves Forward and Backward

        if (gamepad1.right_stick_y != 0.0) {
            BLeft.setPower(0.5*gamepad1.right_stick_y);
            BRight.setPower(0.5*gamepad1.right_stick_y);
            FRight.setPower(0.5*gamepad1.right_stick_y);
            FLeft.setPower(0.5*gamepad1.right_stick_y);
        }
        // Strafing
        else if(gamepad1.right_stick_x != 0.0) {
            BLeft.setPower(0.5*gamepad1.right_stick_x);
            BRight.setPower(-0.5*gamepad1.right_stick_x);
            FRight.setPower(0.5*gamepad1.right_stick_x);
            FLeft.setPower(-0.5*gamepad1.right_stick_x);

        }
        // Rotation
        else if (gamepad1.left_stick_x != 0.0) {
            BLeft.setPower(-0.5 * gamepad1.left_stick_x);
            BRight.setPower(0.5 * gamepad1.left_stick_x);
            FRight.setPower(0.5 * gamepad1.left_stick_x);
            FLeft.setPower(-0.5 * gamepad1.left_stick_x);

        }

        else if (gamepad1.a) {
            // Set the target position for the arm motor
            Arm.setTargetPosition(1000);

            // Set the arm motor to run to the target encoder position
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the power for the arm motor
            Arm.setPower(0.5); // Adjust power level as needed

            // Check if the arm motor is still busy running to the target position
            while (Arm.isBusy()) {
                // Update telemetry or perform other actions if needed
                telemetry.addData("Arm Encoder Position", Arm.getCurrentPosition());
                telemetry.update();

            }

            // Stop the arm motor after reaching the target position
            Arm.setPower(0);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Switch back to using encoders

        }

        else if (gamepad2.right_stick_y != 0.0) {
            Linear.setPower(0.5 * gamepad2.right_stick_y);

        }

        else if (gamepad2.b) {
            Claw.setPosition(1.0);

        }

        else if (gamepad2.x) {
            Claw.setPosition(0.0);

        }

        else {
            BLeft.setPower(0);
            BRight.setPower(0);
            FRight.setPower(0);
            FLeft.setPower(0);
            Arm.setPower(0);
            Linear.setPower(0);
        }

    }

}
