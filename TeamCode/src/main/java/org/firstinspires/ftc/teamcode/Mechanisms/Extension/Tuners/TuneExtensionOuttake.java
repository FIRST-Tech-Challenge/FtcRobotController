package org.firstinspires.ftc.teamcode.Mechanisms.Extension.Tuners;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.ExtensionOuttake;

@Config
@Autonomous(name = "Tune Extension", group = "Autonomous")
public class TuneExtensionOuttake extends LinearOpMode {
    Drivetrain drivetrain = null;

    @Override
    public void runOpMode() {
        ExtensionOuttake extension = new ExtensionOuttake(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x){
                Actions.runBlocking(extension.servoExtension(ExtensionOuttake.extensionState.EXTEND));
            }
        }
    }
}
