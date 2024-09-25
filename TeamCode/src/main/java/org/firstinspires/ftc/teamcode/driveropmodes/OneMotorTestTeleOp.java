package org.firstinspires.ftc.teamcode.driveropmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwaremaps.OneMotorHardwareMap;

/**
 * Class for testing of simple Mecanum movement
 */
@TeleOp(name="One Motor Test", group="Linear OpMode")
public class OneMotorTestTeleOp extends LinearOpMode {
    private OneMotorHardwareMap teamHardwareMap;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Testing Hardware Map Linear Op Mode");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // teamHardwareMap = new OneMotorHardwareMap(hardwareMap);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "FRW");
        waitForStart();
        // teamHardwareMap.Runtime.reset();

        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.x) {
                motor.setPower(1);
            } else {
                motor.setPower(0);
            }
        }

        telemetry.update();
    }
}
