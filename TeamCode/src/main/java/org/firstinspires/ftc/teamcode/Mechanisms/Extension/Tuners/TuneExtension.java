package org.firstinspires.ftc.teamcode.Mechanisms.Extension.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;

@Config
@Autonomous(name = "Test Extension", group = "Autonomous")
public class TuneExtension extends LinearOpMode {
    Drivetrain drivetrain = null;

    @Override
    public void runOpMode() {
        Extension extension = new Extension(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x){
                Actions.runBlocking(extension.servoExtension());
            }
        }
    }
}
