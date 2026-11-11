package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {
    private DcMotor motorTest;
    private DcMotor kevinMotor;
    private DcMotor sriMotor;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        motorTest = hardwareMap.get(DcMotor.class, "motor1");
        kevinMotor = hardwareMap.get(DcMotor.class,"kevinMotor");

        // run until the end of the match (driver presses STOP)
        double tgtPower1Y = 0;
        double tgtPower1X = 0;

        while (opModeIsActive()) {
            tgtPower1Y = -this.gamepad1.left_stick_y;
            tgtPower1X = this.gamepad1.left_stick_x;
            motorTest.setPower(tgtPower1Y);
            kevinMotor.setPower(tgtPower1X);
            telemetry.addData("JoyStick 1 Value Y", tgtPower1Y);
            telemetry.addData("Joystick 1 Value X", tgtPower1X);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}