package org.firstinspires.ftc.Team.BasicRobots;

//This file is the basic framework for a robot that drives on mecanum wheels

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import java.util.function.Consumer;

public class BasicMecanum {

    public DcMotor[] motors = {null, null, null, null};
    public int RFMotor = 0;
    public int RBMotor = 1;
    public int LFMotor = 2;
    public int LBMotor = 3;

    String[] names = {"RFMotor", "RBMotor", "LFMotor", "LBMotor"};

    //public double mult = 0.29;

    public void init(HardwareMap Map) {
        for (int i = 0; i < 5; i++) {
            motors[i] = Map.dcMotor.get(names[i]);
        }
        
        mapAll(m -> {
            m.setPower(0.0);
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });
    }

    public void mapAll (Consumer<DcMotor> f) {
        for (DcMotor motor : motors) {
            f.accept(motor);
        }
    }

    public void setPower (double a, double b, double c, double d) {
        motors[0].setPower(a);
        motors[1].setPower(b);
        motors[2].setPower(c);
        motors[3].setPower(d);
    }
    
    public void mapRight (Consumer<DcMotor> f) {
        f.accept(motors[RFMotor]);
        f.accept(motors[RBMotor]);
    }
    
    public void mapLeft (Consumer<DcMotor> f) {
        f.accept(motors[LFMotor]);
        f.accept(motors[LBMotor]);
    }
    
    public void mapFront (Consumer<DcMotor> f) {
        f.accept(motors[RFMotor]);
        f.accept(motors[LFMotor]);
    }
    
    public void mapBack (Consumer<DcMotor> f) {
        f.accept(motors[RBMotor]);
        f.accept(motors[LBMotor]);
    }

    //Move straight function

    public void moveStraight(double power) {
        mapAll(m -> m.setPower(power));
    }

    //Move left function

    public void moveLeft(double power) {
        motors[LFMotor].setPower(-power);
        motors[RBMotor].setPower(-power);
        motors[RFMotor].setPower(power);
        motors[LBMotor].setPower(power);
    }

    //Move right function

    public void moveRight(double power) {
        motors[LFMotor].setPower(power);
        motors[RBMotor].setPower(power);
        motors[RFMotor].setPower(-power);
        motors[LBMotor].setPower(-power);
    }

    //Move back function

    public void moveBackwards(double power) {
        mapAll(m -> m.setPower(-power));
    }

    //Stop moving function

    public void stop() {
        mapAll(m -> m.setPower(0));
    }

    //Pivot in place based on input from two buttons (left/right (booleans)) and a power setting (0-1 (double-precision float))

    public void pivotTurn(double input_power, boolean rightBumper, boolean leftBumper) {
        final double power = input_power * 0.5;
        if(rightBumper && leftBumper) {
            this.stop();
        } else if(leftBumper) {
            mapRight(m -> m.setPower(power));
            mapLeft(m -> m.setPower(-power));
        } else if(rightBumper) {
            mapRight(m -> m.setPower(-power));
            mapLeft(m -> m.setPower(power));
        }
    }

    //Strafe in all 8 directions based on button inputs (up/down/left/right (boolean)) and a power setting (0-1 (double-precision float))

    public void octoStrafe(double power, boolean up, boolean down, boolean left, boolean right) {
        if (up) {
            if (right) {
                motors[0].setPower(power);
                motors[2].setPower(0);
                motors[1].setPower(0);
                motors[3].setPower(power);
            } else if (left) {
                motors[0].setPower(0);
                motors[2].setPower(power);
                motors[1].setPower(power);
                motors[3].setPower(0);
            } else {
                mapAll(m -> m.setPower(power));
            }
        } else if (down) {
            if (right) {
                motors[0].setPower(0);
                motors[2].setPower(-power);
                motors[1].setPower(-power);
                motors[3].setPower(0);
            } else if (left) {
                motors[0].setPower(-power);
                motors[2].setPower(0);
                motors[1].setPower(0);
                motors[3].setPower(-power);
            } else {
                motors[0].setPower(-1);
                motors[2].setPower(-power);
                motors[1].setPower(-1);
                motors[3].setPower(-power);
            }
        }
        else {
            if (right) {
                motors[0].setPower(-power);
                motors[2].setPower(power);
                motors[1].setPower(power);
                motors[3].setPower(-power);
            } else if (left) {
                motors[0].setPower(power);
                motors[2].setPower(-power);
                motors[1].setPower(-power);
                motors[3].setPower(power);
            }
        }
    }
}