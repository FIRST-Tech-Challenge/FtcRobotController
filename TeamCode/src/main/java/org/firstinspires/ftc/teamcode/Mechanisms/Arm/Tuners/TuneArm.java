package org.firstinspires.ftc.teamcode.Mechanisms.Arm.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;


@Config
@Autonomous(name = "Test Arm", group = "Autonomous")
public class TuneArm extends LinearOpMode {
    Drivetrain drivetrain = null;

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                Actions.runBlocking(arm.servoArmSpec());
            }
        }
    }
}
