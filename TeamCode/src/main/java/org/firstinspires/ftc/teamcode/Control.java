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

    public void mecanumDrive(double leftX, double leftY, double turn){
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
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0);
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0);
    }
}
