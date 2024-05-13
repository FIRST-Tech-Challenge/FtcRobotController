package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    public enum FieldSide{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
        //robot.drivetrain.setDriveDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {

    }
    public void driveBasic(double forward, double turn){
        robot.drivetrain.driveBasic(forward, turn);
    }

    public void mecanumDrive(double leftY, double leftX, double turn){

        robot.drivetrain.mecanumDrive(leftY, leftX, turn, robot.getHeading(), telemetry);
    }
    public void resetIMU(boolean button){
        if (!button){
            return;
        }
        robot.resetIMU();
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
