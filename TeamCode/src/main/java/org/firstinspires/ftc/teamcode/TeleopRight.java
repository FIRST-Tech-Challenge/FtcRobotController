/* FTC Team 7572 - Version 1.0 (10/22/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp RIGHT side of field
 */
@TeleOp(name="Teleop-Right", group="7592")
//@Disabled
public class TeleopRight extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        // PowerPlay is symmetric for Red and Blue, but differs for LEFT and RIGHT sides
        leftAlliance = false;
    }
} // TeleopRight
