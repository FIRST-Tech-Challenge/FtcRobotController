package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.routines.Routine;

public class ViperSystem extends Subsystem{
    private DcMotor viper_motor_r;
    private DcMotor viper_motor_l;

    private double lowerBound = 0.0;
    private double upperBound = 0.0;
    public DcMotor getViper_motor_l() {
        return viper_motor_l;
    }

    public DcMotor getViper_motor_r() {
        return viper_motor_r;
    }

    public void moveOnTick(boolean isUp, boolean isDown){
        double pow = 10;
        if(isUp && isDown){
            getViper_motor_l().setPower(0);
            getViper_motor_r().setPower(0);
        }
        else{
            if(isUp){
                getViper_motor_l().setPower(pow);
                getViper_motor_r().setPower(pow);
            }
            else{
                getViper_motor_l().setPower(0);
                getViper_motor_r().setPower(0);
            }
            if(isDown){
                getViper_motor_l().setPower(-pow);
                getViper_motor_r().setPower(-pow);
            }
            else{
                getViper_motor_l().setPower(0);
                getViper_motor_r().setPower(0);
            }
        }


    }

    public ViperSystem(Routine routine) {
        super(routine);


        viper_motor_l = routine.hardwareMap.get(DcMotor.class, "viper_motor_l");

        viper_motor_r = routine.hardwareMap.get(DcMotor.class, "viper_motor_r");



        viper_motor_r.setDirection(DcMotor.Direction.REVERSE);

        viper_motor_l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viper_motor_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
