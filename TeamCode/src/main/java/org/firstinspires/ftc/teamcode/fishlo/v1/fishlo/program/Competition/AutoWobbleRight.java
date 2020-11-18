package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Utility.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous 
public class AutoWobbleRight extends FishloAutonomousProgram {
    //Create the variable targetZone to store the targetZone value from vision
    protected char targetZone;

    //Create the variables for PID constants
    protected final double Kp = 1;
    protected final double Ki = 0;
    protected final double Kd = 0;

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
        //Timer for vision
        claw.reset();
        claw.armUp();
        claw.grab();
        ElapsedTime timer = new ElapsedTime();
        //Find the target zone based on the starter stack
        timer.reset();
        telemetry.setAutoClear(true);
        while (timer.milliseconds() < 2000) {
            targetZone = 'B'; //vision.getTargetZone();
        }
        while (!isStopRequested() && !gyro.isCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Gyro", gyro.getCallibrationStatus());
        telemetry.update();

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
            //Drives to position (55 inches forward at 0.5 power)
            drive.moveToPosition(68, 0.5);
            claw.armDown();
            claw.reset();
           //Status update: The robot turning to the desired angle
            telemetry.addData("Main", "Turning - P:90R, S: 0.2");
            telemetry.update();
            drive.strafeToPosition(-12, 0.4);

        }
        //Move wobble goal to target zone B
        else if (targetZone == 'B') {
            telemetry.addData("Main", "Driving to Target Zone B");
            telemetry.update();
            //Turning left 5 degrees at 0.2 power using PID
            //Reset the heading of the gyroscope
            gyro.getHeading();
            gyro.resetHeading();
            //Initializes PID with constants
            PID pid = new PID(Kp, Ki, Kd);
            //Sets loop time
            pid.setLoopTime(10);
            //Loops until destination is reached
            while (true) {
                //Sets the error of the PID to -5- the current gyro heading
                pid.setError(-5-gyro.getHeading());
                //Uses the error to find the appropriate motor power
                double power = pid.getSetValue();
                //Status update: Displays the power given by PID
                telemetry.addData("Main", "PID power" + power);
                telemetry.update();
                //Creates a buffer to make sure the robot does not move at a power below 0.1
                if (power <= 0.1) {
                    break;
                }
                //Turns using the created power
                drive.turn(power);
                sleep(10);
            }
            telemetry.addData("Main", "Driving - P: 80 in, S: 0.5");
            telemetry.update();
            drive.moveToPosition(80,0.5);
            sleep(50);
            telemetry.addData("Main", "Strafing - P: -10 in, S: 0.4");
            telemetry.update();
            drive.strafeToPosition(-10,0.4);
            sleep(50);
            telemetry.addData("Main", "Driving - P: -15 in, S: 0.4");
            drive.moveToPosition(-15,0.4);
        }
        //Move wobble goal to target zone C
        else if (targetZone == 'C') {
            telemetry.addData("Main", "Driving to Target Zone C");
            telemetry.update();
            //Turning right 5 degrees at 0.2 power
            //Resets the heading of the gyroscope
            gyro.getHeading();
            gyro.resetHeading();
            //Initializes PID with constants
            PID pid = new PID(Kp, Ki, Kd);
            //Sets loop time
            pid.setLoopTime(10);
            //Loops until the destination is reached
            while (true) {
                //Sets the error of the PID to 5- the current gyro heading
                pid.setError(5-gyro.getHeading());
                //Uses the error to find the appropriate motor power
                double power = pid.getSetValue();
                //Status update: Displays power given by PID
                telemetry.addData("Main", "PID power" + power);
                telemetry.update();
                //Creates a buffer to make sure the robot does not move at a power below 0.1
                if (power <= 0.1) {
                    break;
                }
                //Turns with created power
                drive.turn(power);
                sleep(10);
            }
            //Status update: The robot is driving to the target zone
            telemetry.addData("Main", "Driving - P: 100 in, S: 0.5");
            telemetry.update();
            //Drives to position (100 inches forward at 0.5 power)
            drive.moveToPosition(100,0.5);
            sleep(50);
            //Status update: The robot is driving to the line
            telemetry.addData("Main", "Driving - P: -35 in, S: 0.5");
            telemetry.update();
            //Drives to position (35 inches backward at 0.5 power)
            drive.moveToPosition(-35,0.5);
            sleep(50);
            //Status update: The robot is strafing away from the target zones and towards the line
            telemetry.addData("Main", "Strafing - P: -18 in, S: 0.5");
            telemetry.update();
            //Strafes to position (18 inches left at 0.4 power)
            drive.strafeToPosition(-18,0.4);
        }
        //Clears the screen of all telemetry updates
        telemetry.clear();
        //Status update: The program is complete
        telemetry.addData("Main", "Program Complete");
        telemetry.update();
    }
}
