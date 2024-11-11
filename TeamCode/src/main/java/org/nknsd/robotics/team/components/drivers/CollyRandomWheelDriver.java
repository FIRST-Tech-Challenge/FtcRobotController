package org.nknsd.robotics.team.components.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.WheelHandler;

// Adds events to gamepad to control the wheels
public class CollyRandomWheelDriver implements NKNComponent {
    private final double speedMin;
    private final double speedMax;
    private final double speedStepAmount;
    private final GamePadHandler.GamepadSticks forwardStick;
    private final GamePadHandler.GamepadSticks strafeStick;
    private final GamePadHandler.GamepadSticks turnStick;
    private GamePadHandler gamePadHandler;
    private WheelHandler wheelHandler;

    private double moveSpeedMultiplier;
    private boolean imuCorrection = true;

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

    Runnable disableAutonomousIMUCorrection = new Runnable() {
        @Override
        public void run() {
            imuCorrection = false;
        }
    };
    private Gamepad gamepad;
    private IMUComponent imuComponent;

    public CollyRandomWheelDriver(double speedMin, double speedMax, int speedSteps, GamePadHandler.GamepadSticks forwardStick, GamePadHandler.GamepadSticks strafeStick, GamePadHandler.GamepadSticks turnStick) {
        this.speedMin = speedMin;
        this.speedMax = speedMax;
        this.forwardStick = forwardStick;
        this.strafeStick = strafeStick;
        this.turnStick = turnStick;
        speedStepAmount = (speedMax - speedMin) / 5;
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        moveSpeedMultiplier = 0;
        this.gamepad = gamepad1;

        gamePadHandler.addListener(GamePadHandler.GamepadButtons.Y, 1, "disableAutonomousIMUYawCorrection", true, disableAutonomousIMUCorrection);
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        gamePadHandler.addListener(GamePadHandler.GamepadButtons.RIGHT_BUMPER, 1, "speedUp", true, speedUp);
        gamePadHandler.addListener(GamePadHandler.GamepadButtons.LEFT_BUMPER, 1, "speedDown", true, speedDown);

        gamePadHandler.removeListener(GamePadHandler.GamepadButtons.Y, 1, "disableAutonomousIMUYawCorrection", true);
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "AdvancedWheelDriver";
    }


    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        double yaw = imuComponent.getYaw();
        if (imuCorrection) {
            yaw += 90;
        }

        if (GamePadHandler.GamepadButtons.Y.detect(gamepad)) {
            wheelHandler.relativeVectorToMotion(strafeStick.getValue(gamepad) * moveSpeedMultiplier, forwardStick.getValue(gamepad) * moveSpeedMultiplier, turnStick.getValue(gamepad) * moveSpeedMultiplier);
        } else {
            wheelHandler.absoluteVectorToMotion(strafeStick.getValue(gamepad) * moveSpeedMultiplier, forwardStick.getValue(gamepad) * moveSpeedMultiplier, turnStick.getValue(gamepad) * moveSpeedMultiplier, yaw, telemetry);
        }

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Gear", (moveSpeedMultiplier - speedMin) / speedStepAmount);
        telemetry.addData("Raw Speed", moveSpeedMultiplier);
    }

    public void link(GamePadHandler gamePadHandler, WheelHandler wheelHandler, IMUComponent imuComponent) {
        this.gamePadHandler = gamePadHandler;
        this.wheelHandler = wheelHandler;
        this.imuComponent = imuComponent;
    }
}
