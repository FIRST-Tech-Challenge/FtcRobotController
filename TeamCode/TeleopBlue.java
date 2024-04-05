/* FTC Team 7572 - Version 1.0 (11/11/2023)
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
        // CENTERSTAGE has different AprilTags for Red vs. Blue.
        // CENTERSTAGE has no differnces for Left vs. Right during Teleop
        //leftAlliance = true;
        blueAlliance = true;

        // CENTERSTAGE AprilTag assignments:
        aprilTagLeft   = 1;  // Blue Alliance LEFT   Backdrop
        aprilTagCenter = 2;  // Blue Alliance CENTER Backdrop
        aprilTagRight  = 3;  // Blue Alliance RIGHT  Backdrop
        aprilTagSmall  = 9;  // Blue Alliance SMALL Audience  5-stack 2"/50mm
        aprilTagLarge  = 10; // Blue Alliance LARGE Audience  5-stack 5"/127mm
    }
} // TeleopBlue
