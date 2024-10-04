package org.nknsd.robotics.framework;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * NKNComponent can be extended with your functionality and then loaded into the iterative op mode
 */
public interface NKNComponent {
    /**
     * init() is called once during startup
     *  @return true if successfully started, otherwise false (which stops the init process)
     */
    boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2);

    void init_loop(ElapsedTime runtime, Telemetry telemetry);

    void start(ElapsedTime runtime, Telemetry telemetry);

    void stop(ElapsedTime runtime, Telemetry telemetry);

    String getName();

    void loop(ElapsedTime runtime, Telemetry telemetry);

    void doTelemetry(Telemetry telemetry);
}
