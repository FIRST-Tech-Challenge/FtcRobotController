package org.firstinspires.ftc.teamcode.subsystems.specimen;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class SpecimenSliderClaw extends SonicSubsystemBase {

    private Servo specimenClawServo;

    private Telemetry telemetry;

    boolean clawInOpenPosition = false;

    public SpecimenSliderClaw(HardwareMap hardwareMap, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.specimenClawServo  = hardwareMap.get(Servo.class,"SpecimenClaw");

        this.telemetry = telemetry;
        closeClaw();
    }

    public void openClaw() {
        this.specimenClawServo.setPosition(0.8);
    } 

    public void closeClaw() {
        this.specimenClawServo.setPosition(0.08);
    }


    public void ToggleClaw() {
        if(clawInOpenPosition) {
            closeClaw();
        } else {
            openClaw();
        }

        clawInOpenPosition = !clawInOpenPosition;


    }
}