package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Odometry Test", group="Test")
public class OdometryTest extends LinearOpMode {
    private DigitalChannel odometryWheel;
    private DcMotor frontLeft, frontRight, backLeft, backRight;



    @Override
    public void runOpMode() {
        // Initialize the odometry wheel
        odometryWheel = hardwareMap.get(DigitalChannel.class, "odometry"); // Use the name you configured

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        frontLeft = hardwareMap.get(DcMotor.class, "LF");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        backLeft = hardwareMap.get(DcMotor.class, "LB");
        backRight = hardwareMap.get(DcMotor.class, "RB");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Read and display the digital input value
            telemetry.addData("Odometry Reading", frontLeft.getCurrentPosition());
            telemetry.update();

            // Short sleep to prevent overwhelming the telemetry
            sleep(50);
        }
    }
}