package org.firstinspires.ftc.team6220;

import com.wilyworks.common.Wily;
import com.wilyworks.common.WilyWorks;

/**
 * This customizes the configuration of the Wily Works simulator to match your robot.
 */
@Wily
public class WilyConfig extends WilyWorks.Config {
    WilyConfig() {
        // Impersonate the DevBot when running the simulator:
        deviceName = "DevBot";

        // Use these dimensions for the robot:
        robotWidth = 18.0;
        robotLength = 18.0;
    }
}
