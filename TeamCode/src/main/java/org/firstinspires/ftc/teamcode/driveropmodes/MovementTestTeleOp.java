package org.firstinspires.ftc.teamcode.driveropmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardwaremaps.DeepHardwareMap;
import org.firstinspires.ftc.teamcode.Mecanum;

/**
 * Class for testing of simple Mecanum movement
 */
@TeleOp(name="Movement Test", group="Linear OpMode")
public class MovementTestTeleOp extends LinearOpMode {
    private DeepHardwareMap teamHardwareMap;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Testing Hardware Map Linear Op Mode");
        telemetry.addData("Status", "Initialized");

        teamHardwareMap = new DeepHardwareMap(hardwareMap);

        DcMotorSimple[] motors = new DcMotorSimple[] {teamHardwareMap.FrontRightMotor, teamHardwareMap.FrontLeftMotor, teamHardwareMap.BackRightMotor, teamHardwareMap.BackLeftMotor};

        // Gamepads to use for rising edge detector
        Gamepad current_gp = new Gamepad();
        Gamepad prev_gp = new Gamepad();

        telemetry.addData("Front Right Motor:", teamHardwareMap.FrontRightMotor == null ? "Dead" : "Alive");
        telemetry.addData("Front Left Motor:", teamHardwareMap.FrontLeftMotor == null ? "Dead" : "Alive");
        telemetry.addData("Back Right Motor:", teamHardwareMap.BackRightMotor == null ? "Dead" : "Alive");
        telemetry.addData("Back Left Motor:", teamHardwareMap.BackLeftMotor == null ? "Dead" : "Alive");

        Mecanum m = Mecanum.Init(motors[0], motors[1], motors[2], motors[3]);
        m.PowerMultiplier = 0.5;

        telemetry.addLine("Initiated Mecanum Drive Move");
        telemetry.update();

        waitForStart();

        telemetry.addLine("In init?");
        telemetry.update();

        while (opModeIsActive()) {
            // Copy both gamepads
            prev_gp.copy(current_gp);
            current_gp.copy(gamepad1);

            // Give gamepad to mecanum to move wheels
            double[] powers = m.Move(gamepad1);

            telemetry.addData("FRW Power", powers[0]);
            telemetry.addData("FLW Power", powers[1]);
            telemetry.addData("BRW Power", powers[2]);
            telemetry.addData("BLW Power", powers[3]);
            telemetry.update();
        }

    }
}