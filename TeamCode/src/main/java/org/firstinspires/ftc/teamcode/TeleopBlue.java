/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp for BLUE alliance
 */
@TeleOp(name="Teleop-Blue", group="7592")
//@Disabled
public class TeleopBlue extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        // INTO THE DEEP has different AprilTags for Red vs. Blue.
        // INTO THE DEEP has no differnces for Left vs. Right during Teleop
      //leftAlliance = true;
        blueAlliance = true;

        // INTO THE DEEP AprilTag assignments:
        aprilTagHuman  = 11;  // Blue Alliance Audience / Human Player
        aprilTagStart  = 12;  // Blue Alliance Starting Wall
        aprilTagBasket = 13;  // Blue Alliance Basket
    }
} // TeleopBlue
