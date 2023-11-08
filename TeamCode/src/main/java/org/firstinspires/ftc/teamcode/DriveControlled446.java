package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveControlled446 extends LinearOpMode {

    // region variables
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor intakeMotor;
    private Servo frontIntake1;
    private Servo frontIntake2;
    private Servo outtake;
    private Servo flipper;

    // endregion
    @Override
    public void runOpMode() {

        //region Dc Motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        //endregion

        // region Servos
        frontIntake1 = hardwareMap.get(Servo.class, "frontIntake1");
        frontIntake2 = hardwareMap.get(Servo.class, "frontIntake2");
        outtake = hardwareMap.get(Servo.class, "outtake");
        flipper = hardwareMap.get(Servo.class, "flipper");
        // endregion

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            // region Mecanum Drive
            // Gamepad inputs
            double y = -gamepad1.left_stick_y; // Reverse the y-axis (if needed)
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            // Calculate motor powers
            double frontLeftPower = y + x + rotation;
            double frontRightPower = y - x - rotation;
            double backLeftPower = y - x + rotation;
            double backRightPower = y + x - rotation;

            // Clip motor powers to ensure they are within the valid range [-1, 1]
            frontLeftPower = Range.clip(frontLeftPower, -1, 1);
            frontRightPower = Range.clip(frontRightPower, -1, 1);
            backLeftPower = Range.clip(backLeftPower, -1, 1);
            backRightPower = Range.clip(backRightPower, -1, 1);

            // Set motor powers
            motorFL.setPower(frontLeftPower);
            motorFR.setPower(frontRightPower);
            motorBL.setPower(backLeftPower);
            motorBR.setPower(backRightPower);
            // endregion
            
            // region intake
            // This is just a test. Will change in the future
            if (gamepad1.a) {
                intakeMotor.setPower(1);
                outtake.setPosition(1);
            }
            if (gamepad2.b) {
                frontIntake1.setPosition(1);
                frontIntake2.setPosition(1);
            }
            if (gamepad2.x) {
                frontIntake1.setPosition(0);
                frontIntake2.setPosition(0);
                outtake.setPosition(0);
            }
            boolean isFlipperOpen = false;
            if (gamepad2.y) {
                if (!isFlipperOpen) {
                    flipper.setPosition(1);
                }
                else {
                    flipper.setPosition(0);
                }
            }
            // endregion

            // region Telemetry
            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());
            telemetry.addData("LF Position:", motorFL.getCurrentPosition());
            telemetry.addData("LB Position:", motorBL.getCurrentPosition());
            telemetry.addData("RF Position:", motorFR.getCurrentPosition());
            telemetry.addData("RB Position:", motorBR.getCurrentPosition());
            telemetry.addData("Intake Motor Power: ", intakeMotor.getPower());
            telemetry.addData("Intake Motor Position: ", intakeMotor.getCurrentPosition());
            telemetry.addData("FrontIntake1 Position: ", frontIntake1.getCurrentPosition());
            telemetry.addData("FrontIntake2 Position: ", frontIntake2.getCurrentPosition());
            telemetry.addData("Outtake Position: ", outtake.getCurrentPosition());
            telemetry.addData("Flipper position: ", flipper.getCurrentPosition());
            telemetry.update();
            //endregion
        }
    }
}

