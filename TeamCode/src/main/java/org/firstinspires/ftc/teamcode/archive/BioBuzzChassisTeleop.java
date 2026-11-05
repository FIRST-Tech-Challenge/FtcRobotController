 package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BioBuzz Chassie", group = "TeleOp")
public class BioBuzzChassisTeleop extends OpMode {

    // Declare motors globally so they can be accessed in both init() and loop()
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor intakeMotor;

    @Override
    public void init() {
        // 1. Hardware mapping
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRight");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");

        // 2. Reverse right-side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Runs repeatedly after pressing INIT until you press PLAY
    }

    @Override
    public void start() {
        // Runs once right when PLAY is pressed
    }

    @Override
    public void loop() {
        // 3. Read gamepad inputs
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // 4. Denominator math for power scaling
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        // 5. Apply power to motors
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.right_trigger > 0) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        // 6. Telemetry feedback
        telemetry.addData("Front Left/Right", "%.2f / %.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back Left/Right", "%.2f / %.2f", backLeftPower, backRightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Runs once when the OpMode is shut down
    }
}