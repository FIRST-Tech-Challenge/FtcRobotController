package org.firstinspires.ftc.teamcode.barebones;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 */
@TeleOp(name="BareBones: Tank", group="BareBones")
public class BareBones_Tank extends OpMode {

    // Instance Members.
    private boolean doMotors = true;
    private DcMotor leftDrive;
    private DcMotor rightDrive;


    // Called once, right after hitting the Init button.
    @Override
    public void init() {

        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power.
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Yo", "Initialized Tank, motors=%b", doMotors);
    }

    @Override
    public void loop() {

        if (doMotors) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("left", "%.2f", leftPower);
            telemetry.addData("right", "%.2f", rightPower);
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
    }
}
