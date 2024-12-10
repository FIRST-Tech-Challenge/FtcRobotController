package org.nknsd.teamcode.components.handlers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class SpecimenClawHandler implements NKNComponent {
    private final String clawName = "specimenClaw";
    private Servo servo;
    public ClawPositions clawPosition = ClawPositions.GRIP;
    private SpecimenRotationHandler specimenRotationHandler;
    public SpecimenRotationHandler.SpecimenRotationPositions firstClosedPosition;

    public void setClawPosition(ClawPositions clawPositions) {
        servo.setPosition(clawPositions.position);
        clawPosition = clawPositions;

        if (clawPositions == ClawPositions.GRIP) {
            firstClosedPosition = specimenRotationHandler.targetPosition();
        } else {
            firstClosedPosition = null;
        }
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        servo = hardwareMap.servo.get(clawName);
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
        return "SpecimenClawHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Claw Position", clawPosition.name());
        if (firstClosedPosition != null) {
            telemetry.addData("Last Pickup Loc", firstClosedPosition.name());
        }

    }

    public enum ClawPositions {
        GRIP(0),
        RELEASE(0.5);
        
        public final double position;
        ClawPositions(double position) { this.position = position;}
    }
    public void link(SpecimenRotationHandler specimenRotationHandler){this.specimenRotationHandler = specimenRotationHandler;}
}
