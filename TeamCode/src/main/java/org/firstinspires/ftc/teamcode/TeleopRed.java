/* FTC Team 7572 - Version 1.0 (10/22/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp Full Control.
 */
@TeleOp(name="Teleop-Red", group="7592")
//@Disabled
public class TeleopRed extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        // 435 rpm motor  = 384.5 encoder pulse/rev at output shaft 
        // 1150 rpm motor = 145.1 encoder pulse/rev at output shaft 
        duckVelocityNow  =  100; //red target counts per second (positive!)
        duckVelocityStep =   90; //red ramp-up step size (positive!)
    }
} // TeleopRed
