package org.nknsd.teamcode.components.handlers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class SpecimenRotationHandler implements NKNComponent {
    private final String rotatorName = "specimenRotate";
    private Servo servo;
    private SpecimenRotationPositions target = SpecimenRotationPositions.FORWARD;
    public enum SpecimenRotationPositions{
        BACK(0.15),
        MIDDLE(.6),
        FORWARD(0.95);
       final double position;
       SpecimenRotationPositions(double position) {this.position = position;}
    }
    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        servo = hardwareMap.servo.get(rotatorName);
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "specimenRotationHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
    telemetry.addData("specimenRotationPosition", servo.getPosition());
    }
    public boolean goToPosition(SpecimenRotationHandler.SpecimenRotationPositions specimenRotationPosition){
        servo.setPosition(specimenRotationPosition.position);
        target = specimenRotationPosition;
        return true;
    }
    public SpecimenRotationPositions targetPosition(){ return target;}
}
