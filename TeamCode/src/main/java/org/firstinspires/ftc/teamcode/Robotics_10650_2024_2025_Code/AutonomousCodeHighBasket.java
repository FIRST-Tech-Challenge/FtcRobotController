// Program created by: Danny and William
// Purpose: FTC Robot Software

// The file path of the class
package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

// Import all of the necessary FTC libraries and code
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Create an Autonomous program (Auto) that preselects a TeleOp (controller operated)
@Autonomous(name = "AutonomousCodeHighBasket", preselectTeleOp = "TeleOpCode_RobotCentric")
public class AutonomousCodeHighBasket extends LinearOpMode {

    // Execute the function from the RobotInitialize class
    RobotInitialize robot;

    // The code that runs in Auto
    @Override
    public void runOpMode() throws InterruptedException {
        // Way for the RobotInitialize class to be used inside of this class
        robot = new RobotInitialize(this);
        // Waits for a person to press start on the control hub
        // then it runs the rest of the program
        waitForStart();

        // Phase 1 auto program (go forward, raise arm device, place pre-loaded sample, then strafe
        // into the ascent zone and use the arm device to touch the first bar
        // Fairly accurate but might need improvement

        //robot.claw.setPower(.5);
        //robot.liftExtender.setTargetPosition(0);

        //Positive values make the lift go backwards
        //Negative values make the lift go forwards
//        robot.liftPitch(200, 0.05);
//        robot.intakeToggle(1);

        // Main movement of drivetrain during Auto
        robot.goStraight(850, 500);


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