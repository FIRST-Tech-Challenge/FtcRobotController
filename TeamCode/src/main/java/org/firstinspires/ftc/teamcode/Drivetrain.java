package org.firstinspires.ftc.teamcode;

public class Drivetrain {
    HardwareMapV2 robot;

    public void forward(double power){
        robot.setPowerAll(power);
    }
    public void stop(){
        forward(0);
    }
    public void spin(boolean right, double power){

        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
    }
}
