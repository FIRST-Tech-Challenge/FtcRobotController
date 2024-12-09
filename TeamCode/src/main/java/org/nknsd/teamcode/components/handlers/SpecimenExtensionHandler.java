package org.nknsd.teamcode.components.handlers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

import java.util.concurrent.TimeUnit;

public class SpecimenExtensionHandler implements NKNComponent {
    private final String extenderName = "motorSpecimenExtend";
    private final boolean doInvertMotor = true;
    private final double motorPower = 1;
    private DcMotor motor;          // extender motor
    int extenderPrevious = 0;
    private double lastResetAttempt = 200;

    private SpecimenExtensionPositions target = SpecimenExtensionPositions.RESTING;

    public boolean isExtensionDone() {
        return (Math.abs(motor.getCurrentPosition() - target.position) <= 15);
    }

    public enum SpecimenExtensionPositions {
        RESTING(0) {
            @Override
            boolean safeToExtend(SpecimenRotationHandler.SpecimenRotationPositions firstClosedPosition, SpecimenRotationHandler.SpecimenRotationPositions rotationPosition) {
                return true;
            }
        },
        SPECIMEN_READY(1950){
            @Override
            boolean safeToExtend(SpecimenRotationHandler.SpecimenRotationPositions firstClosedPosition, SpecimenRotationHandler.SpecimenRotationPositions rotationPosition) {
                return (firstClosedPosition != rotationPosition);
            }
        },
        SPECIMEN_CLIP(2300){
            @Override
            boolean safeToExtend(SpecimenRotationHandler.SpecimenRotationPositions firstClosedPosition, SpecimenRotationHandler.SpecimenRotationPositions rotationPosition) {
                return (firstClosedPosition != rotationPosition);
            }
        };

        final int position;

        SpecimenExtensionPositions(int position) {
            this.position = position;
        }
        abstract boolean safeToExtend(SpecimenRotationHandler.SpecimenRotationPositions firstClosedPosition, SpecimenRotationHandler.SpecimenRotationPositions rotationPosition);
    }
    
    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get(extenderName);
        if (doInvertMotor) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(motorPower);
        motor.setTargetPosition(SpecimenExtensionPositions.RESTING.position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        return "SpecimenExtensionHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        double RESET_DELAY = 400; //adjusts delay
        if ((runtime.now(TimeUnit.MILLISECONDS) - RESET_DELAY) > lastResetAttempt && target == SpecimenExtensionPositions.RESTING && motor.getCurrentPosition() <= 600) {
                if (motor.getCurrentPosition() == extenderPrevious) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                extenderPrevious = motor.getCurrentPosition();
            lastResetAttempt = runtime.now(TimeUnit.MILLISECONDS);
        }
    } //resets encoder when arm is resting and no longer moving

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Ext Current Position", motor.getCurrentPosition());
        telemetry.addData("Ext Target Position", motor.getTargetPosition());
        telemetry.addData("Ext State", target.name());
    }
    public boolean gotoPosition(SpecimenExtensionHandler.SpecimenExtensionPositions specimenExtensionPosition, SpecimenRotationHandler.SpecimenRotationPositions firstClosedPosition, SpecimenRotationHandler.SpecimenRotationPositions rotationPosition) {
        if(specimenExtensionPosition.safeToExtend(firstClosedPosition, rotationPosition)) {
            motor.setTargetPosition(specimenExtensionPosition.position);
            target = specimenExtensionPosition;

            return true;
        } else {
            return false;
        }
    }
    public void resetEncoder() {
        if (target == SpecimenExtensionPositions.RESTING) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public SpecimenExtensionPositions targetPosition() {
        return target;
    }
}
