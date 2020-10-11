package org.firstinspires.ftc.teamcode;

public class Drivetrain {
    HardwareMapV2 robot;
    enum tiltDirect{
        UP, DOWN
    }
    static int pos = 0;

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
    public void tilt(double pos){
        robot.leftTilt.setPosition(pos);
        robot.rightTilt.setPosition(pos);
    }
    public void tiltpos(tiltDirect direct){
        if (direct == tiltDirect.UP && pos != 2){
           pos++;
        }else if(direct == tiltDirect.DOWN && pos != 0){
            pos--;
        }
        tilt((pos==0) ? 0.0 : (pos==1) ? 0.5 : 1.0);
    }

}
