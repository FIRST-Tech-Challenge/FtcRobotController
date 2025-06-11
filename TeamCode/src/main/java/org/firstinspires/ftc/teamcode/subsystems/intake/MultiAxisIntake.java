package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class MultiAxisIntake extends SonicSubsystemBase {
    //claw Servo
    private Servo clawServo;
    private boolean ClawOpen = true;
    //Y axis servo (The one that rotates up and down)
    private Servo pitchServo;
    //X Axis servo (The one that rotates in place, side to side)
    private Servo yawServo;
    //State
    private States state;
    private enum States {IntakeSample, IntakeSpeciman, OuttakeSample, OuttakeSpeicman}
    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;
    public MultiAxisIntake(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.yawServo = hardwareMap.get(Servo.class,"Yaw");
        this.pitchServo = hardwareMap.get(Servo.class,"Pitch");
        this.clawServo = hardwareMap.get(Servo.class, "Claw");

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        SetYawForward();
        SetPitchInStart();

        state = States.IntakeSample;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (state == States.IntakeSample) {
        }
        if (state == States.OuttakeSample) {
        }
    }

    public void SetYawForward() {
        Wait(300);
        yawServo.setPosition(0);
    }

    public void SetYawBackward() {
        yawServo.setPosition(1);
        Wait(300);
    }

    public void SetPitchIntakeSubmersible() {
        pitchServo.setPosition(0.5);
    }

    public void SetPitchOuttakeSample() {
        pitchServo.setPosition(.6);
    }

    public void SetPitchToSpecimenIntake() {
        pitchServo.setPosition(.8);
    }

    public void SetPitchToSpecimenOuttake() {
        pitchServo.setPosition(.5);
    }

    public void SequenceSpecimentOuttakeToIntake() {
        SetPitchIntakeSubmersible();
        Wait(200);
        SetYawForward();
        Wait(400);
        SetPitchToSpecimenIntake();
    }

    public void SequenceSpecimentIntakeToOuttake() {
        SetPitchIntakeSubmersible();
        Wait(200);
        SetYawBackward();
        Wait(200);
        SetPitchToSpecimenOuttake();
    }


    public void Wait() {
        Wait(500);
    }

    public void SetPitchInStart() {
        pitchServo.setPosition(0);
    }

    public void ToggleClaw() {
        if (ClawOpen) {
            OpenClaw();
        } else {
            CloseClaw();
        }
        ClawOpen = !ClawOpen;
    }

    public void OpenClaw() {
        this.clawServo.setPosition(0);
    }

    public void CloseClaw() {
        this.clawServo.setPosition(1);

    }

    public void CloseClawAndWait() {
        this.clawServo.setPosition(1);
        Wait(300);
    }


    public void SetIntake() {
        SetYawForward();
        SetPitchIntakeSubmersible();
    }

    public void SetSampleOuttake() {
        SetYawBackward();
        SetPitchOuttakeSample();
    }
}