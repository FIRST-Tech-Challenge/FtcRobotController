/* FTC Team 7572 - Version 1.0 (11/11/2023)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp LEFT side of field
 */
@TeleOp(name="Teleop-Left", group="7592")
//@Disabled
public class TeleopLeft extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        // PowerPlay is symmetric for Red vs. Blue, and Left vs. Right
        // during Tele-Op.  We define this, but so far never use it.
        leftAlliance = true;
    }
} // TeleopLeft
