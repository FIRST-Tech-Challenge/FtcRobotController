package org.nknsd.robotics.team.components.drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.SpecimenExtensionHandler;
import org.nknsd.robotics.team.controlSchemes.reals.KarstenSpecimenController;
import org.nknsd.robotics.team.components.GamePadHandler;

import java.util.concurrent.TimeUnit;
public class SpecimenDriver implements NKNComponent {
    GamePadHandler gamePadHandler;
    SpecimenExtensionHandler specimenExtensionHandler;

    GamePadHandler.GamepadButtons rotateUpButton = GamePadHandler.GamepadButtons.DPAD_UP;
    GamePadHandler.GamepadButtons rotateDownButton = GamePadHandler.GamepadButtons.DPAD_DOWN;
    GamePadHandler.GamepadButtons extendButton = GamePadHandler.GamepadButtons.DPAD_RIGHT;
    GamePadHandler.GamepadButtons retractButton = GamePadHandler.GamepadButtons.DPAD_LEFT;
    GamePadHandler.GamepadButtons takeButton = GamePadHandler.GamepadButtons.A;
    GamePadHandler.GamepadButtons releaseButton = GamePadHandler.GamepadButtons.B;

    Runnable specimenExtend = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = specimenExtensionHandler.targetPosition().ordinal();
            while (!done) {
                index++;

                if (index >= SpecimenExtensionHandler.SpecimenExtensionPositions.values().length) {
                    return;
                }

                done = specimenExtensionHandler.gotoPosition(SpecimenExtensionHandler.SpecimenExtensionPositions.values()[index]);
            }
        }
    };
    Runnable specimenRetract = new Runnable() {
        @Override
        public void run() {

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

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "SpecimenDriver";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        gamePadHandler.addListener(extendButton, 2, "specimenExtend", true, specimenExtend);
        gamePadHandler.addListener(retractButton, 2, "specimenRetract", true, specimenRetract);
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("SpecimenExtTarget", specimenExtensionHandler.targetPosition());
    }
    public void link (SpecimenExtensionHandler specimenExtensionHandler){
        this.specimenExtensionHandler = specimenExtensionHandler;
    }
}
