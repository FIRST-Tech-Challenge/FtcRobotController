package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.kalipsorobotics.modules.OuttakePositions;

@TeleOp
public class TestOuttakeActions extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        DcMotor linearSlideTwo = hardwareMap.get(DcMotor.class, "linearSlideTwo");

        // Set encoder mode and reset
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoder mode
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final double CIRCUMFERENCE_OF_SPOOL = 2 * Math.PI * 18;


        double TICKS_PER_MM = CIRCUMFERENCE_OF_SPOOL / 145.1 ;

        // Sets a power to start
        double power = 0.5;

        linearSlideTwo.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            
            boolean prevGamepadA = false;
            boolean currentGamepadA = gamepad1.a;

            // Manual control with dpad for testing
            if (gamepad1.y) {
                linearSlide.setPower(power);
                linearSlideTwo.setPower(power);
            } else if (gamepad1.a) {
                linearSlide.setPower(-power);
                linearSlideTwo.setPower(-power);
            } else {
                linearSlide.setPower(0);
                linearSlideTwo.setPower(0);
            }

            // Adjust power with buttons
            if (gamepad1.dpad_up) {
                power = Math.max(0.1, power - 0.05);
            } else if (gamepad1.dpad_down) {
                power = Math.min(1, power + 0.05);
            }

            /*
            if (currentGamepadA && !prevGamepadA) {
                setTargetHeight(linearSlide, linearSlideTwo, power, TICKS_PER_INCH);
            } else if (currentGamepadA && !prevGamepadA) {
                setTargetHeight(OuttakeHeight.WALL_HEIGHT, linearSlide, linearSlideTwo, power, TICKS_PER_INCH);
            } else if (currentGamepadA && !prevGamepadA) {
                setTargetHeight(OuttakeHeight.LOW_BAR, linearSlide, linearSlideTwo, power, TICKS_PER_INCH);
            } else if (currentGamepadA && !prevGamepadA) {
                setTargetHeight(OuttakeHeight.HIGH_BAR, linearSlide, linearSlideTwo, power, TICKS_PER_INCH);
            } else if (currentGamepadA && !prevGamepadA) {
                setTargetHeight(OuttakeHeight.HIGH_BASKET, linearSlide, linearSlideTwo, power, TICKS_PER_INCH);
            }
            */

            // Calculate and display encoder position in inches
            double encoderPosition = linearSlideTwo.getCurrentPosition();
            double positionMM = encoderPosition / TICKS_PER_MM;

            // Display diagnostics for troubleshooting
            telemetry.addData("TICKS_PER_MM", TICKS_PER_MM);
            telemetry.addData("Encoder Position (ticks)", encoderPosition);
            telemetry.addData("Position (MM)", positionMM);
            telemetry.update();
        }
    }

    private void setTargetHeight(DcMotor motor1, DcMotor motor2, double power, double ticksPerInch) {
        double targetTicks = OuttakePositions.LOW_BAR.getTargetTicks(ticksPerInch);

        //move motor by ticks

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(power);
        motor2.setPower(power);

        telemetry.addData("Target Height (inches)", OuttakePositions.LOW_BAR.getHeight());
        telemetry.addData("Target Position (ticks)", targetTicks);
    }
}
