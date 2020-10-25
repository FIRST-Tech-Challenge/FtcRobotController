package autofunctions;

import global.TerraBot;
import util.Vector;

public class Odometry {
//    TerraBot bot; // Robot Object
//    public double tx = 0; //total x
//    public double ty = 0; //total y
//    double rp = 0; //right pos
//    double lp = 0; //left pos
//    double cp = 0; //center pos
//    double deltaRP = 0; //change in right pos
//    double deltaLP = 0; //change in left pos
//    double deltaCP = 0; //change in center pos
//
//    public double forward = 0;
//    public double turn = 0;
//    public double strafe = 0;
//    public double theta = 0; //total x
//
//    public double sr = 0;
//    public double sl = 0;
//    public double sc = 0;
//    public double cr = 0;
//    public double cl = 0;
//    public double cc = 0;
//
//    public double vx = 0;
//
//
//    public final double TICKS_FOR_ODOMETRY =  8192;
//    public final double ENCODER_WHEEL_RADIUS = 2.5; // in cm
//    public final double RIGHT_AND_LEFT_ENCODER_RADIUS = 13.3;
//    public final double ENCODER_RATIO = 1;
//
//    public void init(TerraBot b){
//        bot = b;
//
//        updateEncoderPositions();
//
//        sr = bot.getRightEncoder();
//        sl = bot.getLeftEncoder();
//        sc = bot.getCenterEncoder();
//
//    }
//
//    public void reset(){
//        tx = 0;
//        ty = 0;
//        theta = 0;
//        updateEncoderPositions();
//    }
//
//    public void updateGlobalPosition(){
//
//        deltaRP = bot.getRightEncoder()-rp;
//        deltaLP = bot.getLeftEncoder()-lp;
//        deltaCP = bot.getCenterEncoder()-cp;
//
//        updateEncoderPositions();
//
//
//        forward = (deltaRP + deltaLP)/4;
//        turn = (deltaRP - deltaLP)/2;
//        strafe = ((deltaCP*ENCODER_RATIO)-turn)/2;
//
//        //theta += inchesToDegrees(ticksToInches(turn)); // Convert to degrees
//        theta =  bot.getHeading();
//
//        Vector movementVect = new Vector(ticksToInches(strafe), ticksToInches(forward));
//        movementVect = movementVect.getRotatedVector(theta);
//
//        vx = movementVect.x;
//
//        ty += movementVect.y;
//        tx += movementVect.x;
//
//        cr = bot.getRightEncoder()-sr;
//        cl = bot.getLeftEncoder()-sl;
//        cc = theta;
//
//
//    }
//    public double ticksToInches(double in){
//        return (in/TICKS_FOR_ODOMETRY)* (2*Math.PI*ENCODER_WHEEL_RADIUS);
//    }
//    public double inchesToDegrees(double in){
//        return (in/(2*Math.PI*RIGHT_AND_LEFT_ENCODER_RADIUS))*360;
//    }
//
//    public void updateEncoderPositions(){
//        rp = bot.getRightEncoder();
//        lp = bot.getLeftEncoder();
//        cp = bot.getCenterEncoder();
//    }
//
//    public double[] getGlobalPose(){
//        double[] out = new double[3];
//        out[0] = tx;
//        out[1] = ty;
//        out[2] = theta;
//        return out;
//    }

}
