/* FTC Team 7572 - Version 1.0 (10/22/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp Full Control.
 */
@TeleOp(name="Teleop", group="7592")
//@Disabled
public class TeleopRed extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        // PowerPlay season doesn't have any red/blue alliance-specific differences
    }
} // TeleopRed
