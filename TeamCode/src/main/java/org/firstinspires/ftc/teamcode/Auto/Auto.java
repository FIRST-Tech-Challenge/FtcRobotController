package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


/**
 * Auto creates a robots and runs it in auto mode.
 *
 * <p>Auto currently just initializes the Robot as Auto.runOpMode() is empty.</p>
 *
 * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
 */

//Tasks:
// Deliver duck from carousel (10)
//Deliver freight to hub (6)
// - deliver freight to corresponding level of custom element (20)
//Park in warehouse (10)
@Autonomous(name = "Auto", group = "Concept")
public class Auto extends LinearOpMode {
    /** Override of runOpMode()
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.</p>
     *
     * @throws InterruptedException
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        Robot robot = new Robot(this, timer, true);
         //**NOTE** The directions here are based on an orientation where the audience is at the bottom, blue alliance on the left, and red alliance on the right

        //Assume start on position closest to carousels

        //Detect frieght position (vision)

        //Move down 2ft

        //Deliver duck from carousel

        //Park in warehouse (completely)

        //Move towards center horizontally 7ft

        //Move up 5ft

        //Deliver frieght to hub 
            //Vision to detect custom element position

            //Determine corresponding level on shipping hub & drop in frieght

        //Move up 3ft

        //Move towards (our) alliance 3.5ft

        //Move up 3ft to park
        
    }
}
