package org.firstinspires.ftc.teamcode;

public class Drivetrain {
    HardwareMapV2 robot;
    Drivetrain (HardwareMapV2 robot) { this.robot = robot; }
    enum tiltDirect{
        UP, DOWN
    }
    enum moveDirection {
        FORWARD, BACKWARD, LEFT, RIGHT
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
    public void moveDirect(moveDirection direction, double power){
        switch (direction) {
            case FORWARD:
                setMotorPowers(power, power, power, power);
                break;
            case BACKWARD:
                setMotorPowers(-power, -power, -power, -power);
                break;
            case LEFT:
                setMotorPowers(power, -power, -power, power);
                break;
            case RIGHT:
                setMotorPowers(-power, power, power, -power);
                break;
        }
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
        if (0.0 <= pos && pos <= 1.0) {
            robot.leftTilt.setPosition(pos);
            robot.rightTilt.setPosition(pos);
        }
    }
    public void tiltpos(tiltDirect direct){
        if (direct == tiltDirect.UP && pos != 2){
           pos++;
        }else if(direct == tiltDirect.DOWN && pos != 0){
            pos--;
        }
        tilt((pos==0) ? 0.0 : (pos==1) ? 0.5 : 1.0);
    }
    public void incrementtilt(double amount){
        incrementtilt(amount, amount);
    }

    public void incrementtilt(double amountRT, double amountLT){
        amountLT = (robot.leftTilt.getPosition()-amountLT<0 || robot.leftTilt.getPosition()-amountLT>1.0) ? robot.leftTilt.getPosition() : amountLT;
        amountRT = (robot.rightTilt.getPosition()+amountRT<0 || robot.rightTilt.getPosition()+amountRT>1.0) ? robot.rightTilt.getPosition() : amountRT;
        robot.leftTilt.setPosition(robot.leftTilt.getPosition()-amountLT);
        robot.rightTilt.setPosition(robot.rightTilt.getPosition()+amountRT);
    }

    public void incrementaltiltRight(double amount){
        robot.rightTilt.setPosition(robot.rightTilt.getPosition()+amount);
    }
    public void incrementaltiltLeft(double amount){
        robot.leftTilt.setPosition(robot.leftTilt.getPosition()+amount);
    }

}
