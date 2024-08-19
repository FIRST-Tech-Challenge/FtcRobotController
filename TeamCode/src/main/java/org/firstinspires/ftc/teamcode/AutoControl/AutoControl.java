package org.firstinspires.ftc.teamcode.AutoControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotClass;

public class AutoControl extends OpMode {

    RobotClass robot;
    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
    }

    @Override
    public void loop() {


    }
    public void AutoDrive(double power, double targetDistance, double angle){

        angle -= robot.getHeading();
        double distanceUpper = targetDistance + 10;
        double distanceLower = targetDistance - 10;

        double distanceTraveled = 0;


        double currentPositionEncoder = robot.drivetrain.driveMotors[0].getCurrentPosition();
        boolean isAtPos = false;

        while(!isAtPos){
            distanceTraveled =  robot.drivetrain.driveMotors[0].getCurrentPosition() - currentPositionEncoder;
            double error = targetDistance - robot.drivetrain.getAvgPos();

            double x = targetDistance * Math.cos(angle);
            double y = targetDistance * Math.sin(angle);

            double rotY = x * Math.cos(-angle) - y * Math.sin(-angle);
            double rotX = x * Math.sin(angle) - y * Math.cos(angle);

            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY), 1);

            robot.drivetrain.driveMotors[0].setPower((((rotY + rotX) / denominator)) / power);
            robot.drivetrain.driveMotors[1].setPower((((rotY - rotX) / denominator)) / power);
            robot.drivetrain.driveMotors[2].setPower((((rotY - rotX) / denominator)) / power);
            robot.drivetrain.driveMotors[3].setPower((((rotY + rotX) / denominator)) / power);

            if(error < 10 && error > -10) {
                isAtPos = true;
            }

        }
        robot.drivetrain.setPowerBasic(0);
    }


    @Override
    public void stop(){

        if(robot == null) return; // ensures that stop() is not called before initialization
        robot.drivetrain.driveMotors[0].setPower(0);
        robot.drivetrain.driveMotors[1].setPower(0);
        robot.drivetrain.driveMotors[2].setPower(0);
        robot.drivetrain.driveMotors[3].setPower(0);
    }
}
