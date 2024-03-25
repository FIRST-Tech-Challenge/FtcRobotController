package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Control extends OpMode {
    RobotClass robot;
    /*
        define global enums here
         v

         ex.
         public enum Pos{
            OPEN,
            CLOSED
         }

         ^
     */
    public enum DriveMode {
        GLOBAL,
        LOCAL
    }

    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
    }

    @Override
    public void loop() {

    }
    public void driveBasic(double forward, double turn){
        robot.drivetrain.driveBasic(forward, turn);
    }

    public void mecanumDrive(float forward, float strafe, double turn){
        robot.drivetrain.mecanumDrive(forward, strafe, turn);
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
