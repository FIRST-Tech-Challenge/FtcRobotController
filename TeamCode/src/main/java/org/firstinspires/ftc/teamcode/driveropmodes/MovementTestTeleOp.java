package org.firstinspires.ftc.teamcode.driveropmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

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

    enum MoveDir {
        FORWARD,
        BACKWARD
    }

    private TestingHardwareMap teamHardwareMap;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Testing Hardware Map Linear Op Mode");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotor[] motors = teamHardwareMap.GetDriveMotors();
        Mecanum m = Mecanum.Init(motors[0], motors[1], motors[2], motors[3]);

        int current_motor_selected;
        int dir = 1;

        Gamepad current_gp = new Gamepad();
        Gamepad prev_gp = new Gamepad();

        waitForStart();
        teamHardwareMap.Runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            prev_gp.copy(current_gp);
            current_gp.copy(gamepad1);

            m.Move(current_gp);

            if(gamepad1.left_bumper && !prev_gp.left_bumper) {
                dir = -dir;
                telemetry.addData("Current motor direction", dir);
            }

            int final_dir = dir;

            if(gamepad1.right_bumper) {
                Arrays.stream(motors)
                        .forEach(x -> x.setPower(final_dir));
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
                        .forEach(x -> motors[x].setPower(x == sel ? final_dir : 0));
                telemetry.addData("Moving Motor: ", sel);
            }
        }

        telemetry.update();
    }
}
