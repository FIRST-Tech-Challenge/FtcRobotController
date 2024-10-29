package org.nknsd.robotics.team.components.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.RotationHandler;

import org.nknsd.robotics.team.components.GamePadHandler.GamepadButtons;

public class EACDriver implements NKNComponent {
    GamePadHandler gamePadHandler;
    RotationHandler rotationHandler;
    ExtensionHandler extensionHandler;
    IntakeSpinnerHandler servoHandler;

    GamepadButtons rotateUpButton = GamepadButtons.DPAD_UP;
    GamepadButtons rotateDownButton = GamepadButtons.DPAD_DOWN;
    GamepadButtons extendButton = GamepadButtons.DPAD_RIGHT;
    GamepadButtons retractButton = GamepadButtons.DPAD_LEFT;
    GamepadButtons takeButton = GamepadButtons.A;
    GamepadButtons releaseButton = GamepadButtons.B;

    Runnable rotateUp = new Runnable() {
        @Override
        public void run() {
            int nextIndex = rotationHandler.targetRotationPosition.ordinal() + 1;

            if (nextIndex >= RotationHandler.MAX_INDEX_OF_ROTATION_POSITIONS) {return;}

            rotationHandler.setTargetRotationPosition(RotationHandler.RotationPositions.values()[nextIndex]);
        }
    };
    Runnable rotateDown = new Runnable() {
        @Override
        public void run() {
            int prevIndex = rotationHandler.targetRotationPosition.ordinal() - 1;

            if (prevIndex < 0) {return;}

            rotationHandler.setTargetRotationPosition(RotationHandler.RotationPositions.values()[prevIndex]);
        }
    };

    Runnable extend = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = extensionHandler.targetPosition().ordinal();
            while (!done) {
                index ++;

                if (index >= ExtensionHandler.ExtensionPositions.values().length) {return;}

                done = extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.values()[index]);
            }
        }
    };
    Runnable retract = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = extensionHandler.targetPosition().ordinal();
            while (!done) {
                index --;

                if (index < 0) {return;}

                done = extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.values()[index]);
            }
        }
    };


    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        // Add event listeners
        gamePadHandler.addListener(rotateUpButton, 2, "armRotateUp", true, rotateUp);
        gamePadHandler.addListener(rotateDownButton, 2, "armRotateDown", true, rotateDown);
        gamePadHandler.addListener(extendButton, 2, "armExtend", true, extend);
        gamePadHandler.addListener(retractButton, 2, "armRetract", true, retract);
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "EACDriver";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        if (takeButton.detect(gamePadHandler.getGamePad2())) {
            servoHandler.setServoPower(IntakeSpinnerHandler.HandStates.GRIP);
        } else if (releaseButton.detect(gamePadHandler.getGamePad2())) {
            servoHandler.setServoPower(IntakeSpinnerHandler.HandStates.RELEASE);
        } else {
            servoHandler.setServoPower(IntakeSpinnerHandler.HandStates.REST);
        }
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Rot Target", rotationHandler.targetRotationPosition.name());
        telemetry.addData("Ext Target", extensionHandler.targetPosition().name());
        telemetry.addData("Servo State", servoHandler.getServoPower());
    }

    public void link(GamePadHandler gamePadHandler, RotationHandler rotationHandler, ExtensionHandler extensionHandler, IntakeSpinnerHandler servoHandler) {
        this.gamePadHandler = gamePadHandler;
        this.rotationHandler = rotationHandler;
        this.extensionHandler = extensionHandler;
        this.servoHandler = servoHandler;
    }
}
