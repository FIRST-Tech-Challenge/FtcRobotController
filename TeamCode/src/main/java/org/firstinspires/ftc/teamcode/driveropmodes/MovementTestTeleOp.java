package org.firstinspires.ftc.teamcode.driveropmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TestingHardwareMap;
import org.firstinspires.ftc.teamcode.Mecanum;

/**
 * Class for testing of simple Mecanum movement
 */
@TeleOp(name="Basic: Movement Test Linear OpMode", group="Linear OpMode")
public class MovementTestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotor[] motors = TestingHardwareMap.GetDriveMotors(hardwareMap);
        Mecanum m = Mecanum.Init(motors[0], motors[1], motors[2], motors[3]);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            m.Move(gamepad1);
        }
    }
}
