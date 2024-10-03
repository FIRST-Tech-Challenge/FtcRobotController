package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

// Import all of the necessary FTC libraries and code
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "AutonomousWithSensors", preselectTeleOp = "TeleOpCode")
public class AutonomousCode extends LinearOpMode {

    RobotInitialize robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotInitialize(this);
//        telemetry.addData("wait for start", "ye");
//        telemetry.update();

        waitForStart();
//        telemetry.addData("after start", "ye");
//        telemetry.update();
        // Waits for a person to press start on the control hub
        // then it runs the rest of the program


        //forward
       robot.goStraight(850, 500);
       robot.strafeR(1100, 500); // Continue editing lines 29-31
       robot.goStraight(500, 500);
       robot.strafeL(50, 100);

//        robot.newTurnFuncton(90);
//        robot.strafeL(20, 500);
//        robot.strafeR(20, 500);
//        robot.goStraight(60, 500);
//        robot.goStraight(-60, 500);
//        robot.newTurnFunction(90);

        // Shutdown motors when the code ends
        robot.stopMotors();

    }
}