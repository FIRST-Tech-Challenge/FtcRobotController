package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@TeleOp(name="Test: Elevator", group="Test")
public class Test_Elevator extends LinearOpMode {

    private Elevator elevator;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        elevator = new Elevator(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                elevator.extend();
            } else if (gamepad1.b) {
                elevator.retract();
            }
        }
    }
}
