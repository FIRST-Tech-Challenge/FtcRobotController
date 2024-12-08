// Program created by: Danny and William
// Purpose: FTC Robot Software

// The file path of the class
package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.Auto;

// Import all of the necessary FTC libraries and code

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

// Create an Autonomous program (Auto) that preselects a TeleOp (controller operated)
@Autonomous(name = "AutonomousCodeCloserToBasket", preselectTeleOp = "TeleOpCode_RobotCentric")
public class AutonomousCodeHighBasket extends LinearOpMode {

    // Execute the function from the RobotInitialize class
    RobotInitialize robot;

    // The code that runs in Auto
    @Override
    public void runOpMode() throws InterruptedException {
        // Way for the RobotInitialize class to be used inside of this class
        robot = new RobotInitialize(this, true);
        // Waits for a person to press start on the control hub
        // then it runs the rest of the program
        waitForStart();
//        robot.newTurnFunction(90);
//        robot.newTurnFunction(-90);





//        robot.stopMechanisms();
//        sleep(10000);


        // Phase 1 auto program (go forward, raise arm device, place pre-loaded sample, then strafe
        // into the ascent zone and use the arm device to touch the first bar
        // Fairly accurate but might need improvement



        robot.strafeR(120, 500);
        robot.goStraight(470, 500); // 500 is good velocity for now
        robot.liftExtender(2702, 800);
        robot.liftPitch(172, 300);
        robot.extake(1000);
        //robot.clawRoll.setPosition(0.3372);
        robot.liftPitch(0, 600);

        robot.liftExtender(0, 800);



        //to park in ascent zone
        robot.turnFunction(-90);
        //robot.clawRoll.setPosition(0);
        robot.strafeR(340, 500); //Adjust this
        robot.goStraight(1780, 500);
        robot.turnFunction(-180);
        robot.goStraight(770, 500);
        //goal = 0.946
        robot.parkingServo.setPosition(0.946);
        sleep(1000);








        //to par in observation
//        robot.newTurnFunction(179);
//        robot.clawRoll.setPosition(0);
//        robot.goStraight(3300, 500);
//




        //robot.goStraight(3300, 500);






        //robot.liftPitch(1042, 500);




        //robot.strafeR(200, 500);









        //robot.newTurnFunction()
        //robot.strafeL(11, 500);




        //Log.d("Testing Value ", String.valueOf((robot.inchesToEncoderTicks)));


        //Old auto code (keep for now)
        {
//            robot.strafeR(1100, 500);
//            robot.goStraight(500, 500);
//            robot.strafeL(50, 100);
//            robot.newTurnFunction(90);
//            robot.goStraight(200, 100);
            //}

            // Shutdown all mechanisms when the code ends
            robot.stopMechanisms();
        }
    }
}