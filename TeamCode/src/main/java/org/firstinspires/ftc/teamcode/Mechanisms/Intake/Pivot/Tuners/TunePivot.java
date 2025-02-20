package org.firstinspires.ftc.teamcode.Mechanisms.Intake.Pivot.Tuners;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Pivot.Pivot;

@Config
@Autonomous(name = "Tune Pivot Intake", group = "Autonomous")
public class TunePivot extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pivot pivot = new Pivot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.cross){
                Actions.runBlocking(pivot.setPosition(Intake.intakeState.OUTTAKE));
            }
        }
    }
}
