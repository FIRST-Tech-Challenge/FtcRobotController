package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public abstract class OpModeTemplate extends CommandOpMode {

    protected GamepadEx driverGamepad;
    protected GamepadEx operatorGamepad;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

         // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
    }

    public void end() {
        super.stop();
        // Release gamepad resources
        if (driverGamepad != null) {
            driverGamepad = null;
        }
        if (operatorGamepad != null) {
            operatorGamepad = null;
        }
    }

    public void Wait(long timeout) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < timeout)) {
            // Allow other processing
            Thread.yield();
        }
//        try {
//            synchronized (this) {
//                wait(timeout);
//            }
//        } catch (java.lang.InterruptedException e) {
//        }
    }
}
