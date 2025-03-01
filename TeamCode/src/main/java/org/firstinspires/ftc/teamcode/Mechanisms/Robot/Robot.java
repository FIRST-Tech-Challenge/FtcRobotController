package org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.Mechanisms.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Planners.MotionProfile;

public class Robot {
    public Drivetrain drivetrain;
    public Battery battery;
    public Pivot pivot;
    public Intake intake;
    public Extension extension;
    public Arm arm;
    public Claw claw;
    public Lift lift;
    public Sweeper sweeper;
    public Color colorSensor;
    public Robot(HardwareMap hardwareMap){
        this.battery = new Battery(hardwareMap);
        this.drivetrain = new Drivetrain(hardwareMap, battery);
        this.pivot = new Pivot(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.arm = new Arm(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.lift = new Lift(hardwareMap, battery);
        this.extension = new Extension(hardwareMap);
        this.sweeper = new Sweeper(hardwareMap);
        this.colorSensor = new Color(hardwareMap);

    }
    public Action intakeMove(Intake.intakeState intakeMechState){
        return new SequentialAction(
                pivot.setPosition(intakeMechState),
                intake.motorIntake(intakeMechState)
        );
    }
    public Action outtake(){
        return new SequentialAction(
                pivot.setPosition(Intake.intakeState.STOP),
                intake.motorIntake(Intake.intakeState.OUTTAKE)
        );
    }
    public Action intakeDown(){
        return new SequentialAction(
                pivot.setPosition(Intake.intakeState.INTAKE),
                extension.servoExtension(Extension.extensionState.EXTEND),
                new SleepAction(0.25),
                intake.motorIntake(Intake.intakeState.INTAKE)
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
    public Action stopIfBlue(){
        return new Action() {
        private ElapsedTime t;
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                t = new ElapsedTime();
                initialized = true;
            }
                if (colorSensor.isBlue()){
                    pivot.setPosition(Intake.intakeState.STOP);
                    intake.motorIntake(Intake.intakeState.OUTTAKE);
                }
                return t.seconds()>1;
            }
        };
    }
    public Action stopIfRed(){
        return new Action() {
            private ElapsedTime t;
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized){
                    t = new ElapsedTime();
                    initialized = true;
                }
                if (colorSensor.isRed()){
                    pivot.setPosition(Intake.intakeState.STOP);
                    intake.motorIntake(Intake.intakeState.OUTTAKE);
                }
                return t.seconds()>1;
            }
        };
    }
}
