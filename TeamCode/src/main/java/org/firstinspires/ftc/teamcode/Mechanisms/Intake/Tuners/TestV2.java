package org.firstinspires.ftc.teamcode.Mechanisms.Intake.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;

@Config
@Autonomous(name = "Test V2 Action", group = "Autonomous")
public class TestV2 extends LinearOpMode {
    Intake intake = new Intake(hardwareMap);
    Pivot pivot = new Pivot(hardwareMap);
    Robot robot = new Robot(hardwareMap);
    @Override
        public void runOpMode() throws InterruptedException {
            if(gamepad1.cross){
                //Intake Off
                intake.disableIntake();
            } else if(gamepad1.triangle){
                //Intake On Pivot Down
                intake.motorIntake(Intake.intakeState.INTAKE);
                pivot.setPosition(Intake.intakeState.INTAKE);
            } else if(gamepad1.circle){
                //Intake Reversed Pivot Down
                intake.motorIntake(Intake.intakeState.OUTTAKE);
                pivot.setPosition(Intake.intakeState.INTAKE);
            } else if(gamepad1.square){
                //Intake On Pivot Up
                intake.motorIntake(Intake.intakeState.INTAKE);
                pivot.setPosition(Intake.intakeState.STOP);
            } else if(gamepad1.right_bumper){
                //Intake Reversed Pivot Up
                intake.motorIntake(Intake.intakeState.OUTTAKE);
                pivot.setPosition(Intake.intakeState.STOP);
            }
        }
}
