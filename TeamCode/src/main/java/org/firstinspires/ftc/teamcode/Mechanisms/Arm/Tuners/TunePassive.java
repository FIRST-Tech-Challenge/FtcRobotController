package org.firstinspires.ftc.teamcode.Mechanisms.Arm.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;


@Config
@Autonomous(name = "Test PassiveArm", group = "Autonomous")
public class TunePassive extends LinearOpMode {
    Drivetrain drivetrain = null;
    FtcDashboard dashboard;
    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Pos: " + arm.servoArmLeft.getPosition());
            telemetry.update();
        }
    }
}
