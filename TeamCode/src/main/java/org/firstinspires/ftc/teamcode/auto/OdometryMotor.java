package org.firstinspires.ftc.teamcode.auto;

// a class for handling odometry motor and it calculations
// it extends the MotorEx class because a odometry motor is not powered and so it use the motor functions in that calss
public class OdometryMotor extends MotorEx {
    public enum WHEELTYPE{
        MM,
        INCHES
    }
    public enum TYPE{
        PPR,
        TICKPERREV
    }

    public static double mmToInches = 0.0393701;
    public double inchesPerCount = 0;

    //all the math for converting ticks to inches for any motor
    public OdometryMotor(String name, WHEELTYPE wheeltype, int wheelDiameter, TYPE type, int tick){
        super(name);
        if(wheeltype==WHEELTYPE.MM && type==TYPE.TICKPERREV){
            inchesPerCount = Math.PI * wheelDiameter*mmToInches / tick;
        }
        if(wheeltype==WHEELTYPE.MM && type==TYPE.PPR){
            inchesPerCount = Math.PI * wheelDiameter*mmToInches / (tick * 4);
        }
        if(wheeltype==WHEELTYPE.INCHES && type==TYPE.TICKPERREV){
            inchesPerCount = Math.PI * wheelDiameter / tick;
        }
        if(wheeltype==WHEELTYPE.INCHES && type==TYPE.PPR){
            inchesPerCount = Math.PI * wheelDiameter / (tick * 4);
        }
    }


    //pass in the ticks and it returns the amount of inches those ticks are
    private double getInches(int ticks){
        return inchesPerCount * ticks;
    }
    public void move(double inches) {
        double ticks = inches / inchesPerCount;
        int position = motor.getCurrentPosition() + (int)ticks;
        motor.setTargetPosition(position);
    }
}
