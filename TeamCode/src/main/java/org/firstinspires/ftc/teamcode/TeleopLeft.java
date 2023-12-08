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

        // CENTERSTAGE AprilTag assignments:
        int  aprilTagLeft   = 1;  // Blue Alliance LEFT   Backdrop
        int  aprilTagCenter = 2;  // Blue Alliance CENTER Backdrop
        int  aprilTagRight  = 3;  // Blue Alliance RIGHT  Backdrop
        // Blue Alliance 5-stack 2"/50mm  = 9
        // Blue Alliance 5-stack 5"/127mm = 10

    }
} // TeleopLeft
