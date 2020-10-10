package org.firstinspires.ftc.teamcode;

public class Drivetrain {
    HardwareMapV2 robot;

    public void forward(double power){
        robot.setPowerAll(power);
    }
    public void stop(){
        forward(0);
    }
    public void setMotorPowers(double lb, double rb, double lf, double rf){
        robot.frontRight.setPower(rf);
        robot.backRight.setPower(rb);
        robot.frontLeft.setPower(lf);
        robot.backLeft.setPower(lb);
    }
    public void spin(boolean right, double power){
        power = (right) ? power : -power;
        setMotorPowers(power, -power, power, -power);
    }
    public void outtakeAll(double power){
        outtakeAll(power, power);
    }
    public void outtakeAll(double conveyor, double outtake){
        robot.conveyor.setPower(conveyor);
        robot.outtake.setPower(outtake);
    }
}
