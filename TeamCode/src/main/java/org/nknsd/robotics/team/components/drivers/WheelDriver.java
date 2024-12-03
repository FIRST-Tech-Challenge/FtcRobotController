package org.nknsd.robotics.team.components.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.WheelHandler;
import org.nknsd.robotics.team.controlSchemes.abstracts.WheelControlScheme;

// Adds events to gamepad to control the wheels
public class WheelDriver implements NKNComponent {
    private final double speedMin;
    private final double speedMax;
    private final double speedStepAmount;
    private final GamePadHandler.GamepadSticks forwardStick;
    private final GamePadHandler.GamepadSticks strafeStick;
    private final GamePadHandler.GamepadSticks turnStick;
    private GamePadHandler gamePadHandler;
    private WheelHandler wheelHandler;
    private WheelControlScheme controlScheme;

    private double moveSpeedMultiplier;

    Runnable speedUp = new Runnable() {
        @Override
        public void run() {
            if (!(moveSpeedMultiplier + speedStepAmount > speedMax)) {
                moveSpeedMultiplier = moveSpeedMultiplier + speedStepAmount;
            }
        }
    };

    Runnable speedDown = new Runnable() {
        @Override
        public void run() {
            if (!(moveSpeedMultiplier - speedStepAmount < speedMin)) {
                moveSpeedMultiplier = moveSpeedMultiplier - speedStepAmount;
            }
        }
    };
    private Gamepad gamepad;

    public WheelDriver(double speedMin, double speedMax, int speedSteps, GamePadHandler.GamepadSticks forwardStick, GamePadHandler.GamepadSticks strafeStick, GamePadHandler.GamepadSticks turnStick) {
        this.speedMin = speedMin;
        this.speedMax = speedMax;
        this.forwardStick = forwardStick;
        this.strafeStick = strafeStick;
        this.turnStick = turnStick;
        speedStepAmount = (speedMax - speedMin) / speedSteps;
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        moveSpeedMultiplier = 0;
        this.gamepad = gamepad1;
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        gamePadHandler.addListener2(controlScheme.gearDown(), speedDown, "Speed Down");
        gamePadHandler.addListener2(controlScheme.gearUp(), speedUp, "Speed Up");
        gamePadHandler.addListener2(controlScheme.resetAngle(), () -> {}, "Reset Angle");
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "WheelDriver";
    }


    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        wheelHandler.relativeVectorToMotion(forwardStick.getValue(gamepad) * moveSpeedMultiplier, strafeStick.getValue(gamepad) * moveSpeedMultiplier, turnStick.getValue(gamepad) * moveSpeedMultiplier);
//        double y = 0; double x = 0;
//        if (GamePadHandler.GamepadButtons.DPAD_UP.detect(gamepad)) {
//            y = 0.4;
//        } else if (GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamepad)) {
//            y = -0.4;
//        }
//
//        if (GamePadHandler.GamepadButtons.DPAD_RIGHT.detect(gamepad)) {
//            x = 0.4;
//        } else if (GamePadHandler.GamepadButtons.DPAD_LEFT.detect(gamepad)) {
//            x = -0.4;
//        }
//
//        wheelHandler.relativeVectorToMotion(y * moveSpeedMultiplier, x * moveSpeedMultiplier, 0);
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Gear", (moveSpeedMultiplier - speedMin) / speedStepAmount);
        telemetry.addData("Raw Speed", moveSpeedMultiplier);
    }

    public void link(GamePadHandler gamePadHandler, WheelHandler wheelHandler, WheelControlScheme controlScheme) {
        this.gamePadHandler = gamePadHandler;
        this.wheelHandler = wheelHandler;
        this.controlScheme = controlScheme;
    }
}
