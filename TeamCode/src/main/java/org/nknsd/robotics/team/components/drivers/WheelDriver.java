package org.nknsd.robotics.team.components.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.WheelHandler;

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
        gamePadHandler.addListener(GamePadHandler.GamepadButtons.RIGHT_BUMPER, 1, "speedUp", true, speedUp);
        gamePadHandler.addListener(GamePadHandler.GamepadButtons.LEFT_BUMPER, 1, "speedDown", true, speedDown);
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
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Gear", (moveSpeedMultiplier - speedMin) / speedStepAmount);
        telemetry.addData("Raw Speed", moveSpeedMultiplier);
    }

    public void link(GamePadHandler gamePadHandler, WheelHandler wheelHandler) {
        this.gamePadHandler = gamePadHandler;
        this.wheelHandler = wheelHandler;
    }
}
