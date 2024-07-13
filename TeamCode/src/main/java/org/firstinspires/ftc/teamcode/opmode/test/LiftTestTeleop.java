package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Lift;

@Config
@TeleOp(name = "LiftTest", group = "Test")

public class LiftTestTeleop extends LinearOpMode {

    private Lift lift;
    public static double liftPowerFactor = 0.15;
    @Override
    public void runOpMode() {
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;
        liftPowerFactor = 0.5;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(hardwareMap, telemetry, true);
        waitForStart();

        while (opModeIsActive()) {

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (leftTrigger > 0.3) {
                lift.manualDown(leftTrigger*liftPowerFactor);
            } else if (rightTrigger > 0.3) {
                lift.manualUp(rightTrigger*liftPowerFactor);
            } else
            {
                lift.stop();
            }
        }
    }
}
