/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp for RED alliance
 */
@TeleOp(name="Teleop-Red", group="7592")
//@Disabled
public class TeleopRed extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        // INTO THE DEEP has different AprilTags for Red vs. Blue.
        // INTO THE DEEP has no differnces for Left vs. Right during Teleop
      //leftAlliance = true;
        blueAlliance = false;

        // INTO THE DEEP AprilTag assignments:
        aprilTagHuman  = 14;  // Red Alliance Audience / Human Player
        aprilTagStart  = 15;  // Red Alliance Starting Wall
        aprilTagBasket = 16;  // Red Alliance Basket
    }
} // TeleopRed
