/* FTC Team 7572 - Version 1.0 (11/11/2023)
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
        // CENTERSTAGE has different AprilTags for Red vs. Blue.
        // CENTERSTAGE has no differnces for Left vs. Right during Teleop
        //leftAlliance = true;
        blueAlliance = false;

        // CENTERSTAGE AprilTag assignments:
        aprilTagLeft   = 4;  // Red Alliance LEFT   Backdrop
        aprilTagCenter = 5;  // Red Alliance CENTER Backdrop
        aprilTagRight  = 6;  // Red Alliance RIGHT  Backdrop
        aprilTagSmall  = 8;  // Red Alliance SMALL Audience  5-stack 2"/50mm
        aprilTagLarge  = 7;  // Red Alliance LARGE Audience  5-stack 5"/127mm
    }
} // TeleopRed
