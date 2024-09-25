package org.firstinspires.ftc.teamcode.driveropmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardwaremaps.DeepHardwareMap;

import java.util.Arrays;
import java.util.stream.IntStream;

/**
 * Class for testing of simple Mecanum movement
 */
@TeleOp(name="Fine Movement", group="Linear OpMode")
public class FineMovementTestTeleOp extends LinearOpMode {
    private DeepHardwareMap teamHardwareMap;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Fine Movement TeleOp");
        telemetry.addData("Status", "Initialized");

        teamHardwareMap = new DeepHardwareMap(hardwareMap);

        DcMotorSimple[] motors = new DcMotorSimple[] {teamHardwareMap.FrontRightMotor, teamHardwareMap.FrontLeftMotor, teamHardwareMap.BackRightMotor, teamHardwareMap.BackLeftMotor};

        int current_motor_selected;
        int dir = 1;

        // Gamepads to use for rising edge detector
        Gamepad current_gp = new Gamepad();
        Gamepad prev_gp = new Gamepad();

        // teamHardwareMap.Runtime.reset();

        telemetry.addData("FrontRight: ", teamHardwareMap.FrontRightMotor);
        telemetry.addData("FrontLeft: ", teamHardwareMap.FrontLeftMotor);
        telemetry.addData("BackRIght: ", teamHardwareMap.BackRightMotor);
        telemetry.addData("motor: ", teamHardwareMap.BackLeftMotor);

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            prev_gp.copy(current_gp);
            current_gp.copy(gamepad1);

            // If pressing left bump for first time then change dir
            if(current_gp.left_bumper && !prev_gp.left_bumper) {
                gamepad1.rumble(50);
                dir = -dir;
                if(dir == 1) {
                    gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                }
                telemetry.addData("Current motor direction", dir == 1 ? "Forward" : "Backward");
            }

            int final_dir = dir;

            if(gamepad1.right_bumper) {
                Arrays.stream(motors)
                        .forEach(x -> x.setPower(final_dir));
            }
            if(prev_gp.right_bumper && !current_gp.right_bumper) {
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

            if(prev_gp.a && !current_gp.a) {
                motors[0].setPower(0);
            }
            if(prev_gp.b && !current_gp.b) {
                motors[1].setPower(0);
            }
            if(prev_gp.x && !current_gp.x) {
                motors[2].setPower(0);
            }
            if(prev_gp.y && !current_gp.y) {
                motors[3].setPower(0);
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
