package org.firstinspires.ftc.teamcode.auto;

public class OdometryMotor extends MotorEx {

    public final int inRangeTicks = 5;
    public static int j = 0;

    public enum WHEELTYPE{
        MM,
        INCHES
    }
    public enum TYPE{
        PPR,
        TICKPERREV
    }

    public static double mmToInches = 0.03937008;
    private double tolerance = 0.5;


    public double inchesPerCount = 0;

    public OdometryMotor(String name, WHEELTYPE wheeltype, int wheelDiameter, TYPE type, int tick){
        super(name);
        if(wheeltype==WHEELTYPE.MM || type==TYPE.TICKPERREV){
            inchesPerCount = Math.PI * wheelDiameter*mmToInches / tick;
        }
        if(wheeltype==WHEELTYPE.MM || type==TYPE.PPR){
            inchesPerCount = Math.PI * wheelDiameter*mmToInches / (tick * 4);
        }
        if(wheeltype==WHEELTYPE.INCHES || type==TYPE.TICKPERREV){
            inchesPerCount = Math.PI * wheelDiameter / tick;
        }
        if(wheeltype==WHEELTYPE.INCHES || type==TYPE.PPR){
            inchesPerCount = Math.PI * wheelDiameter / (tick * 4);
        }
    }


    private double getDesiredInches(int ticks){
        return inchesPerCount * ticks;
    }

    public boolean BUSY(){
        double target2 = Math.abs(motor.getTargetPosition()*inchesPerCount - motor.getCurrentPosition()*inchesPerCount);
//        return motorEx.isBusy();
        return target2 < tolerance;
    }

    public void changeTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    public void move(double inches) {
        double ticks = inches / inchesPerCount;
        int position = motor.getCurrentPosition() + (int)ticks;
        motor.setTargetPosition(position);
    }
}
