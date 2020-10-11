package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.io.FileNotFoundException;
import java.io.IOException;

/**
 * Core robot interface to be implemented by all robot classes
 */
public interface Robot2 {
    public enum ProgramType {
        TELE_OP, AUTO_RED, AUTO_BLUE, DIAGNOSIS
    }

    /**
     * Returns unique robot name, usually a result of calling <code>getClass().getSimpleName()</code>
     */
    String getName();

    /**
     * Configure robot. Invoked once during LinearOpMode initialization after robot instance has been created.
     * Robot implementation should configure all its nested components / devices from within this method
     *  and save a reference to telemetry if robot intends to use it.
     * @param configuration robot configuration. Provides access to <code>HardwareMap</code> and
     *                      configuration management for <code>Configurable</code> components / devices
     * @param telemetry telemetry instance to provide updates to driver station
     * @see Configuration
     * @see Telemetry
     */
    void configure(Configuration configuration, Telemetry telemetry, ProgramType autoColor) throws FileNotFoundException;

    /**
     * Reset robot. All applicable hardware devices should be reset to their initial state.
     */
    void reset(boolean auto);
    void end() throws InterruptedException, IOException;
}
