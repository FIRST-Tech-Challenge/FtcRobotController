package org.firstinspires.ftc.teamcode.Mechanisms.Intake.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Pivot.Pivot;
@Config
@Autonomous(name = "Test V2 Action", group = "Autonomous")
public class TestV2 extends LinearOpMode {

    @Override
        public void runOpMode() throws InterruptedException {

            Intake intake = new Intake(hardwareMap);
            Pivot pivot = new Pivot(hardwareMap);

            waitForStart();

            while(opModeIsActive()) {
                if (gamepad1.cross) {
                    //Intake Off
                    Actions.runBlocking(intake.motorIntake(Intake.intakeState.STOP));
                } else if (gamepad1.triangle) {
                    //Intake On Pivot Down
                    Actions.runBlocking(intake.motorIntake(Intake.intakeState.INTAKE));
                    Actions.runBlocking(pivot.setPosition(Intake.intakeState.INTAKE));
                } else if (gamepad1.circle) {
                    //Intake Reversed Pivot Down
                    Actions.runBlocking(intake.motorIntake(Intake.intakeState.OUTTAKE));
                    Actions.runBlocking(pivot.setPosition(Intake.intakeState.INTAKE));
                } else if (gamepad1.square) {
                    //Intake On Pivot Up
                    Actions.runBlocking(intake.motorIntake(Intake.intakeState.INTAKE));
                    Actions.runBlocking(pivot.setPosition(Intake.intakeState.STOP));
                } else if (gamepad1.right_bumper) {
                    //Intake Reversed Pivot Up
                    Actions.runBlocking(intake.motorIntake(Intake.intakeState.OUTTAKE));
                    Actions.runBlocking(pivot.setPosition(Intake.intakeState.STOP));
                }
            }
        }
}
