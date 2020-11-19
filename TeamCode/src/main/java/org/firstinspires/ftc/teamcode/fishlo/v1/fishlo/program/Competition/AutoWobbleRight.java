package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Utility.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous 
public class AutoWobbleRight extends FishloAutonomousProgram {
    //Create the variable targetZone to store the targetZone value from vision
    protected char targetZone;

    //Create the variables for PID constants
    protected final double Kp = 0.1;
    protected final double HEADING_THRESHOLD = 1;
    protected final double Kd = 0; //425;
    PID pidController = new PID(Kp, Kd, 0);



    //Build the robot
    @Override
    public Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    //This method is for code that needs to run during the init phase
    @Override
    public void preMain() {
        //Initialize the imu
        gyro.initGyro();

        //Reset claw and arm to starting position
        claw.open();
        claw.armUp();
        claw.close();
        //Timer for vision

        //Make sure that the telemetry clears when printing on the screen
        telemetry.setAutoClear(true);
        //Find the targetZone based on the starter stack
        while (!isStopRequested() && !gyro.isCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Gyro", gyro.getCallibrationStatus());
        telemetry.update();
        while (!isStarted()) {

            targetZone = vision.getTargetZone();

        }



    }

    //This method is for code that needs to run after start is pressed.
    @Override
    public void main() {

        //Move wobble goal to target zone A
        if (targetZone == 'A') {
            telemetry.addData("Main", "Driving to Target Zone A");
            telemetry.update();
            //Status update: The robot is driving to the target zone
            telemetry.addData("Main", "Driving - P: 55 in, S: 0.5");
            telemetry.update();
            //Drives to position (68 inches forward at 0.5 power)
            drive.moveToPosition(68, 0.5);
            //Drops the wobble goal
            claw.open();
            sleep(100);
           //Status update: The robot turning to the desired angle
            telemetry.addData("Main", "Strafing - P:-12 in, S: 0.4");
            telemetry.update();
            drive.strafeToPosition(-12, 0.4);

        }
        //Move wobble goal to target zone B
        else if (targetZone == 'B') {
            telemetry.addData("Main", "Driving to Target Zone B");
            telemetry.update();
            drive.moveToPosition(95,0.5);
            drive.strafeToPosition(-35,0.4);
            sleep(1500);
            //Drop the wobble goal
            claw.open();
            drive.moveToPosition(-15, 0.3);
        }
        //Move wobble goal to target zone C
        else if (targetZone == 'C') {
            telemetry.addData("Main", "Driving to Target Zone C");
            telemetry.update();
            //Status update: The robot is driving to the target zone
            telemetry.addData("Main", "Driving - P: 100 in, S: 0.5");
            telemetry.update();
            //Drives to position (100 inches forward at 0.5 power)
            drive.moveToPosition(125,0.5);
            sleep(50);
            //Status update: The robot is driving to the line
            telemetry.addData("Main", "Driving - P: -35 in, S: 0.5");
            telemetry.update();
            //Drives to position (35 inches backward at 0.5 power)
            drive.moveToPosition(-50,0.5);
            claw.open();
            sleep(50);
            //Status update: The robot is strafing away from the target zones and towards the line

            //Strafes to position (18 inches left at 0.4 power)
        }
        //Clears the screen of all telemetry updates
        telemetry.clear();
        //Status update: The program is complete
        telemetry.addData("Main", "Program Complete");
        telemetry.update();
    }

//    public void gyroTurn(double speed, double angle) {
//        while (opModeIsActive() && !onHeading(speed, angle, Kp)) {
//            telemetry.update();
//        }
//    }
//
//    boolean onHeading(double speed, double angle, double Kp) {
//        double error;
//        double steer;
//        boolean onTarget = false;
//
//        double power;
//
//        error = getError(angle);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0;
//            power = 0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, angle);
//            power = steer;
//
//        }
//
//        telemetry.addData("Power: ", power);
//        telemetry.update();
//
//        drive.turn(power);
//        sleep(10);
//        return onTarget;
//
//    }
//
//    public double getError(double targetAngle) {
//        double robotError;
//
//        robotError = targetAngle - gyro.getHeading();
//
//        while (robotError > 180) robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
//
//    public double getSteer(double error, double angle) {
//        pidController.setError(error);
//        return pidController.getSetValue();
//    }
}

