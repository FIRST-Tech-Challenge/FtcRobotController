package org.firstinspires.ftc.teamcode.driveropmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardwaremaps.TestingHardwareMap;
import org.firstinspires.ftc.teamcode.Mecanum;

import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * Class for testing of simple Mecanum movement
 */
@TeleOp(name="Basic: Movement Test Linear OpMode", group="Linear OpMode")
public class MovementTestTeleOp extends LinearOpMode {

    private TestingHardwareMap teamHardwareMap;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotor[] motors = teamHardwareMap.GetDriveMotors();
        Mecanum m = Mecanum.Init(motors[0], motors[1], motors[2], motors[3]);

        int current_motor_selected;

        waitForStart();
        teamHardwareMap.Runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            m.Move(gamepad1);

            if(gamepad1.right_bumper) {
                Arrays.stream(motors)
                        .forEach(x -> x.setPower(1));
            } else {
                Arrays.stream(motors)
                        .forEach(x -> x.setPower(0));
            }

            if(gamepad1.a) {
                current_motor_selected = 0;
            } else if(gamepad1.b) {
                current_motor_selected = 1;
            } else if(gamepad1.x) {
                current_motor_selected = 2;
            } else if(gamepad1.y) {
                current_motor_selected = 3;
            } else {
                current_motor_selected = -1;
            }

            if(current_motor_selected != -1) {
                int sel = current_motor_selected;
                IntStream.range(0, motors.length)
                        .forEach(x -> motors[x].setPower(x == sel ? 1 : 0));
            }
        }

        telemetry.update();
    }
}
