package org.firstinspires.ftc.teamcode.Drivetrain.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Extension.Extension;

@Config
@Autonomous(name = "Test Extension", group = "Autonomous")
public class TuneExtension extends LinearOpMode {
    Drivetrain drivetrain = null;

    @Override
    public void runOpMode() {
        Extension extension = new Extension(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger>0.5){
                Actions.runBlocking(extension.servoExtension(Extension.extensionState.EXTEND));
            }
            if (gamepad1.right_trigger>0.5) {
                Actions.runBlocking(extension.servoExtension(Extension.extensionState.RETRACT));
            }
        }
    }
}
