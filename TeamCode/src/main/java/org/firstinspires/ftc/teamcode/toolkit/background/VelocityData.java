package org.firstinspires.ftc.teamcode.toolkit.background;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Background;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

public class VelocityData extends Background {

    UpliftRobot robot;
    LinearOpMode opMode;

    private static final int TICKS_PER_SHOOTER_WHEEL_ROTATION = 28;

    public VelocityData(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.opMode = robot.opMode;
    }

    @Override
    public void loop() {
        getShooter1RawVelocity();
        getShooter2RawVelocity();
        getShooter1SmoothVelocity();
        getShooter2SmoothVelocity();
//            Log.i("Shooter1Velocity", robot.shooter1SmoothVel + "");
//            Log.i("Shooter2Velocity", robot.shooter2SmoothVel + "");
    }

    public boolean isHighGoalShooterReady() {
        double lowerLimit = robot.highGoalVelocity - 50;
        double upperLimit = robot.highGoalVelocity + 50;
        return (robot.shooter1SmoothVel > lowerLimit && robot.shooter2SmoothVel > lowerLimit && robot.shooter1SmoothVel < upperLimit && robot.shooter2SmoothVel < upperLimit);
    }

    public boolean isPowerShotShooterReady() {
        double lowerLimit = robot.powerShotVelocity - 50;
        double upperLimit = robot.powerShotVelocity + 50;
        return (robot.shooter1SmoothVel > lowerLimit && robot.shooter2SmoothVel > lowerLimit && robot.shooter1SmoothVel < upperLimit && robot.shooter2SmoothVel < upperLimit);
    }

    // get the raw velocity of the shooter1 motor, in ticks/second
    public void getShooter1RawVelocity() {
        double initialTime = System.currentTimeMillis();
        int initialTicks = robot.shooter1.getCurrentPosition();
        Utils.sleep(10);
        double deltaTime = System.currentTimeMillis() - initialTime;
        int deltaTicks = robot.shooter1.getCurrentPosition() - initialTicks;
        robot.shooter1RawVel = (deltaTicks * 1000) / (deltaTime);
    }

    // get the raw velocity of the shooter2 motor, in ticks/second
    public void getShooter2RawVelocity() {
        double initialTime = System.currentTimeMillis();
        int initialTicks = robot.shooter2.getCurrentPosition();
        Utils.sleep(10);
        double deltaTime = System.currentTimeMillis() - initialTime;
        int deltaTicks = robot.shooter2.getCurrentPosition() - initialTicks;
        robot.shooter2RawVel = (deltaTicks * 1000) / (deltaTime);
    }

    // get the velocity of the shooter1 motor, smoothed over 5 data values
    public void getShooter1SmoothVelocity() {
        double tpsTotal = 0;
        for(int i = 0; i < 5; i++) {
            double initialTime = System.currentTimeMillis();
            int initialTicks = robot.shooter1.getCurrentPosition();
            Utils.sleep(10);
            double deltaTime = System.currentTimeMillis() - initialTime;
            int deltaTicks = robot.shooter1.getCurrentPosition() - initialTicks;
            tpsTotal += (deltaTicks * 1000) / (deltaTime);
        }
        robot.shooter1SmoothVel = tpsTotal / 5;
    }

    // get the velocity of the shooter2 motor, smoothed over 5 data values
    public void getShooter2SmoothVelocity() {
        double tpsTotal = 0;
        for(int i = 0; i < 5; i++) {
            double initialTime = System.currentTimeMillis();
            int initialTicks = robot.shooter2.getCurrentPosition();
            Utils.sleep(10);
            double deltaTime = System.currentTimeMillis() - initialTime;
            int deltaTicks = robot.shooter2.getCurrentPosition() - initialTicks;
            tpsTotal += (deltaTicks * 1000) / (deltaTime);
        }
        robot.shooter2SmoothVel = tpsTotal / 5;
    }
}
