package org.firstinspires.ftc.teamcode.subsystems.specimen;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class SpecimenClaw extends SonicSubsystemBase {

    private Servo specimenArmServo;

    private Servo specimenClawServo;

    private Telemetry telemetry;

    public SpecimenClaw(HardwareMap hardwareMap, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.specimenArmServo  = hardwareMap.get(Servo.class,"SpecimenArm");
        this.specimenClawServo  = hardwareMap.get(Servo.class,"SpecimenClaw");

        this.telemetry = telemetry;

        dropArm();
        openClaw();
    }

    public void openClaw() {
        this.specimenClawServo.setPosition(1);
    }

    public void closeClaw() {
        this.specimenClawServo.setPosition(0);
    }

    boolean clawInOpenPosition = true;
    public void ToggleClaw() {
        if(clawInOpenPosition) {
            closeClaw();
        } else {
            openClaw();
        }

        clawInOpenPosition = !clawInOpenPosition;
    }

    public void raiseArm(){
        this.specimenArmServo.setPosition(1);
    }

    public void dropArm(){
        this.specimenArmServo.setPosition(0.37);

    }

    boolean armInIntakePosition = true;
    public void ToggleArm() {
        if(armInIntakePosition) {
            raiseArm();
        } else {
            dropArm();
        }

        armInIntakePosition = !armInIntakePosition;
    }
}