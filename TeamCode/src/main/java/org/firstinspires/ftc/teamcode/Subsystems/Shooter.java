package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    // Define hardware objects
    public DcMotor shooterleft=null;
    public DcMotor shooterright=null;
    public Servo leftFlipper=null;
    public Servo rightFlipper=null;
    public Servo stacker=null;

    //
    //Constants for shooter motors
    private static final double ShooterSpeedfastleft=.6;
    private static final double ShooterSpeedfastright=.8;
    private static final double shooterSpeedslowleft=.55;
    private static final double shooterSpeedslowright=.75;
    private static final double jamClear=-.35;
    //Constants for stacker servos
    private static final double leftUp = 0.75; // .75 a little shy but ok due to hitting bolt
    private static final double leftBack = .4; //good at 0.4;
    private static final double rightUp = (1-leftUp);
    private static final double rightBack = (1-leftBack);
    private static final double flippercenter = 0.5;
    private static final double stackerReload = 0.5;
    private static final double stackerShoot = 0.55;




    public void init(HardwareMap hwMap)  {
        shooterleft=hwMap.get(DcMotor.class,"LeftShooter");
        shooterright=hwMap.get(DcMotor.class,"RightShooter");
        leftFlipper = hwMap.get(Servo.class, "Left_Flipper");
        rightFlipper = hwMap.get(Servo.class, "Right_Flipper");
        stacker =  hwMap.get(Servo.class, "Stacker");

        shooterleft.setDirection(DcMotor.Direction.REVERSE);
        shooterright.setDirection(DcMotor.Direction.FORWARD);
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
    public void stackerMoveToShoot() {
        leftFlipper.setPosition(stackerShoot);

    }
    public void stackerMoveToReload() {
        stacker.setPosition(stackerReload);

    }
    public void flipperCalibrateinCenter() {
        leftFlipper.setPosition(flippercenter);
        rightFlipper.setPosition(flippercenter);
    }
    public void flipperForward() {
        leftFlipper.setPosition(leftUp);
        rightFlipper.setPosition(rightUp);
    }
    public void flipperBackward() {
        leftFlipper.setPosition(leftBack);
        rightFlipper.setPosition(rightBack);
    }
    public void shootoneRing() {
        shootHighGoal();
        stackerMoveToShoot();
        flipperForward();
        flipperBackward();
    }
    public void shooterReload() {
        stackerMoveToReload();
        flipperBackward();
        shooterOff();

        }

    public void jamClear() {
        shooterleft.setPower(jamClear);
        shooterright.setPower(jamClear);


    }
}












