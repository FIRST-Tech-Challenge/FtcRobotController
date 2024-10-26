package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;


@Config
public class Slides {
    public enum SlidesState {
        RETRACTED,
        INTAKE,
        DEPOSIT
    }
    private static final double S_LOWER_BOUND = 0; //number in ticks
    private static final double S_UPPER_BOUND = -1100; //80.25 inches
    private static final double INCHES_TO_TICKS = -34.97131; //number in ticks

    private static final double A_LOWER_BOUND = 0;
    private static final double A_UPPER_BOUND = 800; //180 degrees
    public static final double TICKS_DEGREE_CONSTANT = 4.44444444444; //.225

    public static double kp=5;
    private double error;
    private DcMotorBetter s;
    private DcMotorBetter a;
    private DcMotorBetter bl;
    private DcMotorBetter br;


    private PID linSlideController;
    private PID armController;

    public static double targetLinSlidePosition = 0;
    public static double targetArmPosition = 0;

    public static double sKp = 7, sKi = 3, sKd = .5;
    public static double aKp = 5, aKi = 0, aKd = 0;
    private boolean lowering = false;
    private double targetAngle=0;


    public SlidesArm(HardwareMap hardwareMap){
        s = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "s"));
        a = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "a"));
        a.setDirection(DcMotorSimple.Direction.REVERSE);
        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "bl"));
        bl.setLowerBound(S_LOWER_BOUND);
        bl.setUpperBound(S_UPPER_BOUND);

        br = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "br"));
        br.setLowerBound(A_LOWER_BOUND);
        br.setUpperBound(A_UPPER_BOUND);

        this.linSlideController = new PID(new PID.Coefficients(sKp, sKi, sKd),
                () -> (this.bl.getCurrentPosition()) - this.targetLinSlidePosition,
                factor -> {
                    this.s.setPower(M.clamp(factor, .8, -1)); //b is extension
                });

    }
    public SlidesArm stopAndResetEncoder() {
        this.br.stopAndResetEncoder();
        this.bl.stopAndResetEncoder();
        return this;
    }
    public double getError(){
        return error;
    }
    public double getCurrentSlidesPosition() {
        return this.bl.getCurrentPosition();
    }
    public double getCurrentArmPosition() {
        return this.br.getCurrentPosition();
    }

    public void setDegrees(double degrees){
        targetAngle=degrees;
        targetArmPosition = (degrees * TICKS_DEGREE_CONSTANT) / A_UPPER_BOUND;
    }
    public double getCurrentDegrees(){
        return (getCurrentArmPosition()/TICKS_DEGREE_CONSTANT) * A_UPPER_BOUND;
    }



    public void setInches(double inches) {
        targetLinSlidePosition = (inches * INCHES_TO_TICKS)  / S_UPPER_BOUND; //untested
    }
    public double getCurrentInches() {
        return  (getCurrentSlidesPosition()/INCHES_TO_TICKS) * S_UPPER_BOUND;
    }
    public double getTargetAngle(){
        return targetAngle;
    }
    public void update(){
        linSlideController.update();
        if(getTargetAngle()==0)
        {
            lowering=true;
        }else{
            lowering=false;
        }
        error = -1 * getCurrentArmPosition()-targetArmPosition ;

        double armPower = kp*error;
        if(getCurrentDegrees() < 85 && armPower > .2&&lowering){
            armPower = 0;
        }


        a.setPower(armPower);

        a.update();
        s.update();
    }

}