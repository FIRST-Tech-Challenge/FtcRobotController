package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@TeleOp(name="Test: Elevator", group="Test")
public class Test_Elevator extends LinearOpMode {

    private Elevator elevator;
    @Override
    public void runOpMode() {
        //TODO: turn power on elevator to 100% for climbing - test if it can do it
        //TODO: add driving mechanism to controller for testing
        //TODO: duplicate elevator code for worm gear 
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        elevator = new Elevator(hardwareMap, telemetry);

        waitForStart();

        boolean lastB = false;

        while (opModeIsActive()) {
            elevator.periodic();
            if (gamepad1.a) {
                elevator.setPower((double) gamepad1.left_stick_y);
                telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            }
            if (gamepad1.b && !lastB) {
                elevator.retract(1.0);
            }
            lastB = gamepad1.b;
            telemetry.update();
        }
    }
}
