package org.nknsd.robotics.team.components.testfiles;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.GamePadHandler;

public class EventHandlerTester implements NKNComponent {
    private GamePadHandler gamePadHandler;
    private int pressCount = 0;

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {


        Runnable event = new Runnable() {
            @Override
            public void run() {
                pressCount++;
            }
        };

        gamePadHandler.addListener(GamePadHandler.GamepadButtons.A, 1, "sayA", true, event);
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
        return null;
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("press count",pressCount);
    }

    public void link(GamePadHandler gamePadHandler) {
        this.gamePadHandler = gamePadHandler;
    }
}
