//package org.firstinspires.ftc.teamcode.Autos.Odometry;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
//import org.firstinspires.ftc.teamcode.PID.PIDV2;
//
//public class EncoderPID {
//    private HMap robot;
//    private PIDV2 pidL, pidR;
//    public double response = 0.0;
//
//    public boolean opModeActive = false;
//
//    public int success_counter = 0;
//    private double wheel_radius, x02, encoder_count_per_rev, max_voltage;
//
//    public EncoderPID(double kp, double ki, double kd, HMap robot,
//                      double wheel_radius, double xo2,
//                      double encoder_count_per_rev, final double max_voltage){
//        this.robot = robot; // hardware map
//        this.wheel_radius = wheel_radius; // wheel radius
//        this.x02 = xo2; // xo2 is the distance from wheel to wheel divided by 2
//        this.encoder_count_per_rev = encoder_count_per_rev; // enc count per rev
//        this.max_voltage = max_voltage;
//        this.pidL = new PIDV2(kp, ki, kd) {
//            @Override
//            public void perform(double response) {
//                moveLeft(response * max_voltage);
//                setResponse(response);
//            }
//
//            @Override
//            public double getInputData() {
//                return avgLeftEncoder();
//            }
//        };
//        this.pidR = new PIDV2(kp, ki, kd) {
//            @Override
//            public void perform(double response) {
//                moveRight(response*max_voltage);
//            }
//
//            @Override
//            public double getInputData() {
//                // average the right side encoder values
//                return avgRightEncoder();
//            }
//        };
//    }
//
//    public void setOpModeActive(boolean opModeActive) {
//        this.opModeActive = opModeActive;
//    }
//
//    public boolean isOpModeActive() {
//        return opModeActive;
//    }
//
//    public void moveLeft(double power){
//        robot.TL.setPower(power);
//        robot.BL.setPower(power);
//    }
//
//    public void moveRight(double power){
//        robot.TR.setPower(power);
//        robot.BR.setPower(power);
//    }
//
//    private double avgRightEncoder(){
//        double avg = avgMotorEncoderValues(robot.TR, robot.BR);
//        if (avg == 0){
//            return 0.0001;
//        } else {
//            return avg;
//        }
//    }
//
//    private double avgLeftEncoder(){
//        double avg = avgMotorEncoderValues(robot.TL, robot.BL);
//        if (avg == 0){
//            return 0.0001;
//        } else {
//            return avg;
//        }
//    }
//
//    private int avgMotorEncoderValues(DcMotor m1, DcMotor m2){
//        return (int)Math.floor((m1.getCurrentPosition() + m2.getCurrentPosition())/2.0);
//    }
//
//    private double distance_to_encoder(double distance){
//        return ((distance/(2*3.14*wheel_radius)) * encoder_count_per_rev);
//    }
//
//    public double turn_distance(double angle_of_turn){
//        return (0.0174533)*(x02 * angle_of_turn);
//    }
//
//
//
//    public void setResponse(double response) {
//        this.response = response;
//    }
//
//    private boolean isWithin(double tolerance, double value){
//        return ((0-tolerance) < value) && (value < (0+tolerance));
//    }
//
//    public void move(double distance){
//        double target = distance_to_encoder(distance);
//
//        pidL.executePID(target);
//        pidR.executePID(target);
//        if (isWithin(0.05, response)){
//            success_counter += 1;
//        }
//    }
//
//    public void move_left_to_distance(double distance){
//        double target = distance_to_encoder(distance);
//        pidL.executePID(target);
//    }
//
//    public void move_right_to_distance(double distance){
//        double target = distance_to_encoder(distance);
//        pidR.executePID(target);
//    }
//
//    public void resetParam(){
//        robot.resetEncoders();
//        success_counter = 0;
//        pidL.restartPID();
//        pidR.restartPID();
//    }
//}
