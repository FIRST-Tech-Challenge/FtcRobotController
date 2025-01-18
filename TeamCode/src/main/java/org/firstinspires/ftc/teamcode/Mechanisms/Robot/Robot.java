package org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;

public class Robot {
    public Drivetrain drivetrain;
    public Battery battery;
    public Pivot pivot;
    public Intake intake;
    public Extension extension;
    public Arm arm;
    public Claw claw;
    public Lift lift;
    public Robot(HardwareMap hardwareMap){
        this.battery = new Battery(hardwareMap);
        this.drivetrain = new Drivetrain(hardwareMap, battery);
        this.pivot = new Pivot(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.arm = new Arm(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.lift = new Lift(hardwareMap, battery);
        this.extension = new Extension(hardwareMap);

    }
    public Action intakeMove(Intake.intakeState intakeMechState){
        return new SequentialAction(
                pivot.setPosition(intakeMechState),
                intake.motorIntake(intakeMechState)
        );
    }
    public Action intakeDown(){
        return new SequentialAction(
                intake.motorIntake(Intake.intakeState.INTAKE),
                pivot.setPosition(Intake.intakeState.INTAKE),
                extension.servoExtension(Extension.extensionState.EXTEND)
        );
    }
    public Action intakeUp(){
        return new SequentialAction(
                intake.motorIntake(Intake.intakeState.STOP),
                pivot.setPosition(Intake.intakeState.STOP),
                extension.servoExtension(Extension.extensionState.RETRACT)
        );
    }
    public Action sampleScore(){
        return new SequentialAction(
                claw.servoClaw(Claw.clawState.CLOSE),
                new SleepAction(0.25),
                new ParallelAction(
                    lift.moveToHeight(28),
                    new SleepAction(1),
                    arm.armExtend()
                )
        );
    }
    public Action resetScore(){
        return new SequentialAction(
                claw.servoClaw(Claw.clawState.OPEN),
                new ParallelAction(
                        lift.moveToHeight(0),
                        arm.armExtend()
                )
        );
    }
}
