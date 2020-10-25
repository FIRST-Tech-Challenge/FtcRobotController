//package org.firstinspires.ftc.teamcode.odometry;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//
//import org.firstinspires.ftc.teamcode.utility.pose;
//
//public class SimulatedOdometryWheel extends OdometryWheel {
//
//    DcMotor physicalWheel;
//
//    double speed = 0;
//    public SimulatedOdometryWheel(pose offset) {
//        super(offset);
//    }public SimulatedOdometryWheel(pose offset, double speed, DcMotor wheel) {
//        super(offset);
//        this.speed = speed;
//        physicalWheel = wheel;
//    }
//
//    public void setSpeed(double speed){
//        this.speed = speed;
//    }
//
//    double getDeltaPosition(){
//        // theta * radians = arc length
//        //return -1;
//        return  speed;
//
//    }
//
//    @Override
//    long getCurrentPosition() {
//        return 0;
//    }
//
//    @Override
//    void updateDelta(){
//
//   }
//
//
//    public double robotAngleToOdoDelta(double angle, double xCenter, double yCenter){
//        return super.robotAngleToOdoDelta(angle, xCenter, yCenter);
//    }
//
//    public double odoDeltaToBotAngle(double deltaPosition, double xCenter, double yCenter){
//        return super.odoDeltaToBotAngle(deltaPosition, xCenter, yCenter);
//    }
//
//}
