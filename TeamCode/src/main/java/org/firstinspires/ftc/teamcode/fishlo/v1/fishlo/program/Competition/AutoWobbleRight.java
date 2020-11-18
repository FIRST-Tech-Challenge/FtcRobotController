package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous 
public class AutoWobbleRight extends FishloAutonomousProgram {
    //Create the variable targetZone to store the targetZone value from vision
    protected char targetZone;

    //Create the variables for PID constants
    protected final double Kp = 0.1;
    protected final double HEADING_THRESHOLD = 1;

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
        //Waits until gyro is fully initialized and calibrated before continuing
        while (!isStopRequested() && !gyro.gyroCalibrated()) {
            sleep(50);
            idle();
        }
        //Reset claw and arm to starting position
        claw.open();
        claw.armUp();
        claw.close();
        //Timer for vision
        ElapsedTime timer = new ElapsedTime();
        //Reset the timer
        timer.reset();
        //Make sure that the telemetry clears when printing on the screen
        telemetry.setAutoClear(true);
        //Find the targetZone based on the starter stack
        while (timer.milliseconds() < 2000) {
            targetZone = 'B';
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
            drive.moveToPosition(90,0.5);
            drive.strafeToPosition(5,0.4);
            gyroTurn(0.3, -90);
            //Drop the wobble goal
            claw.open();
        }
        //Move wobble goal to target zone C
        else if (targetZone == 'C') {
            telemetry.addData("Main", "Driving to Target Zone C");
            telemetry.update();
            gyroTurn(0.3, 5);
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

    public void gyroTurn(double speed, double angle) {
        while (opModeIsActive() && !onHeading(speed, angle, Kp)) {
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double Kp) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;
        boolean test = false;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0;
            leftSpeed = 0;
            rightSpeed = 0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, Kp);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        drive.drive(leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {
        double robotError;

        robotError = targetAngle - gyro.getHeading();

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double Kp) {
        return Range.clip(error * Kp, -1, 1);
    }
}


