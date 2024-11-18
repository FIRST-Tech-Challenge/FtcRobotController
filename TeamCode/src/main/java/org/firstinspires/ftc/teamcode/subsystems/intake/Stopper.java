package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;

public class Stopper extends SonicSubsystemBase {

    private Servo stopper;
    private Telemetry telemetry;


    public Stopper(HardwareMap hardwareMap, Telemetry telemetry, DeliveryPivot deliveryPivot) {
        this.stopper = hardwareMap.get(Servo.class,"Stopper");
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (DeliveryPivot.recordedPosition < 45) {
            this.stopper.setPosition(0.5);
        } else {
            this.stopper.setPosition(0.875);
        }
    }
    public void setPositionClosed() {
        this.stopper.setPosition(0.25);
        telemetry.addLine("Stopper closed");
        telemetry.update();
    }

    public void setPositionOpened() {


        this.stopper.setPosition(0.875);
        telemetry.addLine("Stopper opened");
        telemetry.update();
    }
    public void setPositionMid() {
        this.stopper.setPosition(0.5);
        telemetry.addLine("Stopper mid");
        telemetry.update();
    }
}