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
        // 435 rpm motor  = 384.5 encoder pulse/rev at output shaft 
        // 1150 rpm motor = 145.1 encoder pulse/rev at output shaft 
        duckVelocityNow  = -18; //blue target counts per second (negative!)
        duckVelocityStep = -18; //blue ramp-up step size (negative!)
    }
} // TeleopBlue
