package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;

public class MultiAxisIntake extends SonicSubsystemBase {
    //claw Servo
    private Servo clawServo;
    private boolean ClawOpen = false;
    //Y axis servo (The one that rotates up and down)
    private Servo yAxisServo;
    //X Axis servo (The one that rotates in place, side to side)
    private Servo xAxisServo;
    //State
    private States state;
    private enum States {IntakeSample, IntakeSpeciman, OuttakeSample, OuttakeSpeicman}
    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;
    public MultiAxisIntake(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.xAxisServo  = hardwareMap.get(Servo.class,"xAxisServo");
        this.yAxisServo  = hardwareMap.get(Servo.class,"yAxisServo");

        this.clawServo = hardwareMap.get(Servo.class, "clawServo");

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.xAxisServo.setPosition(1);

        state = States.IntakeSample;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (state == States.IntakeSample) {
            xAxisServo.setPosition(1);
            yAxisServo.setPosition(0.5);
        }
        if (state == States.OuttakeSample) {
            xAxisServo.setPosition(0);
            yAxisServo.setPosition(0.5);
        }
    }
    public void ToggleClaw() {
        if (ClawOpen) {
            this.clawServo.setPosition(0);
        } else {
            this.clawServo.setPosition(1);
        }
        ClawOpen = !ClawOpen;
    }

    public void setIntakeSample() {state = States.IntakeSample;}
    public void setOuttakeSample() {state = States.OuttakeSample;}
}