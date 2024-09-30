package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

// Import all of the necessary FTC libraries and code
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "AutonomousWithSensors")
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

//<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robotics_10650_2024_2025_Code/AutonomousWithSensors.java
        //forward
       robot.goStraight(1200, 500);
       sleep(2000);
       robot.goStraight(-300, 500);
       robot.strafeL(2000, 700);
//        robot.newTurnFunction(90);
//        robot.strafeL(20, 500);
//        robot.strafeR(20, 500);
//        robot.goStraight(60, 500);
//        robot.goStraight(-60, 500);
//        robot.newTurnFunction(90);
//=======
        robot.strafeL(20, 500);
        robot.strafeR(20, 500);
        robot.goStraight(20, 500);
        robot.goStraight(-20, 500);
        robot.newTurnFunction(90);
//>>>>>>> 147f788f63f5be8c43e9cafafadeb7468aa70921:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robotics_10650_2024_2025_Code/AutonomousCode.java

        // Shutdown motors when the code ends
        robot.stopMotors();

    }
}