/* FTC Team 7572 - Version 2.0 (12/10/2021)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Full Control.
 */
@TeleOp(name="Teleop-Blue", group="7592")
//@Disabled
public class TeleopBlue extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        duckPower = -0.670;      //blue (negative!)
        duckVelocity = -1600;    //blue target counts per second (negative!)
    }
} // TeleopBlue
