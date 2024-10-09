package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;

@Autonomous(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        boolean isSpecimen = true;
        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(intake.intake(true)
                    , new SleepAction(3)
                    , intake.isRightColorForSpecimen(isSpecimen)
            ));
        }
    }
}