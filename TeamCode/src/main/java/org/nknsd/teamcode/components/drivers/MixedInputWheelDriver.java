package org.nknsd.teamcode.components.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.handlers.gamepad.GamePadHandler;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.controlSchemes.defaults.WheelControlScheme;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class MixedInputWheelDriver implements NKNComponent {
    private final double speedMin;
    private final double speedMax;
    private final double speedStepAmount;
    private final GamePadHandler.GamepadSticks forwardStick;
    private final GamePadHandler.GamepadSticks strafeStick;
    private final GamePadHandler.GamepadSticks turnStick;
    private GamePadHandler gamePadHandler;
    private PowerInputMixer powerInputMixer;
    private WheelControlScheme controlScheme;

    private double moveSpeedMultiplier;
    private double headingSpeed;

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

    public MixedInputWheelDriver(double speedMin, double speedMax, int speedSteps, GamePadHandler.GamepadSticks forwardStick, GamePadHandler.GamepadSticks strafeStick, GamePadHandler.GamepadSticks turnStick) {
        this.speedMin = speedMin;
        this.speedMax = speedMax;
        this.forwardStick = forwardStick;
        this.strafeStick = strafeStick;
        this.turnStick = turnStick;
        speedStepAmount = (speedMax - speedMin) / speedSteps;
        headingSpeed = (speedMin + speedMax) * 63/100;
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
        gamePadHandler.addListener(controlScheme.gearDown(), speedDown, "Speed Down");
        gamePadHandler.addListener(controlScheme.gearUp(), speedUp, "Speed Up");
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "MixedInputWheelDriver";
    }


    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        // NOTE:
        // THE STICKS ARE INVERTED UPON REQUEST BY COLLY
        // WE HAVE NO WAY TO DO THAT VIA CONTROL SCHEME
        powerInputMixer.setManualPowers(new double[]{strafeStick.getValue(gamepad) * -moveSpeedMultiplier, forwardStick.getValue(gamepad) * -moveSpeedMultiplier, turnStick.getValue(gamepad) * headingSpeed});
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
//        wheelHandlerTODODELETEMEUSEPOWERINPUTMIXER.relativeVectorToMotion(y * moveSpeedMultiplier, x * moveSpeedMultiplier, 0);
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Gear", (moveSpeedMultiplier - speedMin) / speedStepAmount);
        telemetry.addData("Raw Speed", moveSpeedMultiplier);
        telemetry.addData("Wheel Controls", controlScheme.getName());
    }

    public void link(GamePadHandler gamePadHandler, PowerInputMixer powerInputMixer, WheelControlScheme controlScheme) {
        this.gamePadHandler = gamePadHandler;
        this.powerInputMixer = powerInputMixer;
        this.controlScheme = controlScheme;
    }
}
