package org.firstinspires.ftc.teamcode.driveropmodes;

import static org.firstinspires.ftc.teamcode.Helper.CopyButtonsFromGamepad;
import static org.firstinspires.ftc.teamcode.Helper.GamepadColour;
import static org.firstinspires.ftc.teamcode.Helper.ReportAllMotorSpeed;
import static org.firstinspires.ftc.teamcode.Helper.ReportDriveMotorStatus;
import static org.firstinspires.ftc.teamcode.Helper.SetGamepadLight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardwaremaps.DeepHardwareMap;

import java.util.Arrays;
import java.util.stream.IntStream;

/**
 * Class for testing of manual movement
 */
@TeleOp(name="Manual Movement", group="Linear OpMode")
public class ManualMovementTeleOp extends LinearOpMode {

    // -----------------------------------------
    // | A / Cross    | Front Right Wheel      |
    // | B / Circle   | Front Left Wheel       |
    // | X / Square   | Back Right Wheel       |
    // | Y / Triangle | Back Left Wheel        |
    // | Right Bumper | All Wheels             |
    // | Left Bumper  | Change Wheel Direction |
    // -----------------------------------------


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Create hardware map
        DeepHardwareMap deepHardwareMap = new DeepHardwareMap(hardwareMap);

        // Gamepads to use for rising edge detector
        Gamepad current_gp = new Gamepad();
        Gamepad prev_gp = new Gamepad();

        DcMotorSimple[] motors = new DcMotorSimple[] {deepHardwareMap.FrontRightMotor, deepHardwareMap.FrontLeftMotor, deepHardwareMap.BackRightMotor, deepHardwareMap.BackLeftMotor};
        boolean[] prev_buttons, curr_buttons;

        // Some variables needed
        int motor_index;
        int dir = 1;
        boolean allmotor_active;

        // Make sure all motors are behaving properly
        ReportDriveMotorStatus(deepHardwareMap, telemetry);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Set up both gamepads
            // prev_gp has state of last loop
            prev_gp.copy(current_gp);
            prev_buttons = CopyButtonsFromGamepad(prev_gp);

            current_gp.copy(gamepad1);
            curr_buttons = CopyButtonsFromGamepad(current_gp);

            // If pressing left bump for first time then change dir
            if(current_gp.left_bumper && !prev_gp.left_bumper) {
                // Change Direction
                dir *= -1;
                SetGamepadLight(gamepad1, dir == 1 ? GamepadColour.GREEN : GamepadColour.BLUE);
                telemetry.addData("Current motor direction", dir == 1 ? "Forward" : "Backward");
            }

            // Needs to be final for .forEach  /  WHY ??
            int final_dir = dir;


            // If pressing bumper set motor power to right state
            allmotor_active = gamepad1.right_bumper;

            // If released bumper set motor power to 0
            if(prev_gp.right_bumper && !current_gp.right_bumper) allmotor_active = false;

            if(allmotor_active) {
                // Set all motors to right setting
                Arrays.stream(motors)
                        .forEach(x -> x.setPower(final_dir));
                continue;
            }


            // Assume no buttons are being pressed
            motor_index = -1;

            for(int i = 0; i < motors.length; i++) {
                // If right button is pressed then activate it
                if(curr_buttons[i]) motor_index = i;
                // If released button then deactivate it
                if(prev_buttons[i] && !curr_buttons[0]) {
                    motors[i].setPower(0);
                }
            }

            // If motor is selected then move motor in proper direction
            if(motor_index != -1) {
                // Selection has to be final for some reason
                int sel = motor_index;
                IntStream.range(0, motors.length)
                        .forEach(x -> motors[x].setPower(x == sel ? final_dir : 0));
            }

            ReportAllMotorSpeed(deepHardwareMap, telemetry);
        }
    }
}
