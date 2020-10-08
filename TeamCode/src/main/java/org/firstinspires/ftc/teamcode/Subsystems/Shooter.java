package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    // Define hardware objects
    public DcMotor shooterleft=null;
    public DcMotor shooterright=null;
    public Servo moveforward=null;
    public Servo moveback=null;

    //
    //Constants
    private static final double ShooterSpeedfastleft=.6;
    private static final double ShooterSpeedfastright=.8;
    private static final double shooterSpeedslowleft=.55;
    private static final double shooterSpeedslowright=.75;
    private static final double jamClear=-.35;
    private static final double servoforwardreturn=0;
    private static final double servoforwardlaunch=1;
    private static final double servobackreturn=0;
    private static final double servobacklaunch=0;


    public void init(HardwareMap hwMap)  {
        shooterleft=hwMap.get(DcMotor.class,"LeftShooter");
        shooterright=hwMap.get(DcMotor.class,"RightShooter");
        moveforward=hwMap.get(Servo.class,"ForwardMove");
        moveback=hwMap.get(Servo.class,"BackMove");

        shooterleft.setDirection(DcMotor.Direction.FORWARD);
        shooterright.setDirection(DcMotor.Direction.REVERSE);
        shooterleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void shootMiddleGoal() {
        shooterleft.setPower(shooterSpeedslowleft);
        shooterright.setPower(shooterSpeedslowright);
    }
    public void shootHighGoal() {
        shooterleft.setPower( ShooterSpeedfastleft);
        shooterright.setPower( ShooterSpeedfastright);
    }
    public void shooterOff() {
            shooterleft.setPower(0);
            shooterright.setPower(0);
        }
    public void jamClear() {
        shooterleft.setPower(jamClear);
        shooterright.setPower(jamClear);


    }
}












