package org.nknsd.teamcode.frameworks;

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

    // Code to run ONCE when the driver hits INIT
    boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2);

    // Code to run REPEATEDLY after hitting INIT but before hitting PLAY
    void init_loop(ElapsedTime runtime, Telemetry telemetry);

    // Code to run ONCE when the driver hits PLAY
    void start(ElapsedTime runtime, Telemetry telemetry);

    // Code to run ONCE after the driver hits STOP
    void stop(ElapsedTime runtime, Telemetry telemetry);

    // Returns the name of the component
    String getName();

    // Code to run REPEATEDLY after the driver hits PLAY
    // Does NOT handle telemetry
    void loop(ElapsedTime runtime, Telemetry telemetry);

    // Runs once every ~500 milliseconds
    // ONLY calls for components to do telemetry
    void doTelemetry(Telemetry telemetry);
}
