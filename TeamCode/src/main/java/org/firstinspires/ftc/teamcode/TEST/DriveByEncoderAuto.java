//package org.firstinspires.ftc.teamcode.Autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Autos.Odometry.DistancePID;
//import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
//import org.firstinspires.ftc.teamcode.FSM.State;
//import org.firstinspires.ftc.teamcode.middleend.HardwareMappings.HMap;
//import org.firstinspires.ftc.teamcode.backend.control.low_level.PID;
//
//@Autonomous(group = "AutoPIDTest", name = "EncoderPID")
//public class DriveByEncoderAuto extends LinearOpMode {
//    // External Class Imports
//    HMap robot = new HMap();
//    private double wheel_rad = 0.0508; // in m
//
////    double kp = 2.7, ki = 0.0000006, kd = 0.0;
//    double kp = 2.7, ki = 0.0, kd = 0.0;
//
//
//    // Parameters
//    double error_tolerance = .02;
//
//    PID L_PID = new PID(kp, ki, kd, error_tolerance) {
//        @Override
//        public void doThingWithResponse(double response) {
//            if(response < 0 && L_PID.current_target<0){
//                robot.TL.setPower(response);
//                robot.BL.setPower(response);
//            } else if (response < 0 && L_PID.current_target > 0){
//                robot.TL.setPower(-response);
//                robot.BL.setPower(-response);
//            } else if (response > 0 && L_PID.current_target > 0){
//                robot.TL.setPower(response);
//                robot.BL.setPower(response);
//            } else {
//                robot.TL.setPower(-response);
//                robot.BL.setPower(-response);
//            }
//        }
//    };
//
//    PID R_PID = new PID(kp, ki, kd, error_tolerance) {
//        @Override
//        public void doThingWithResponse(double response) {
//            if(response < 0 && R_PID.current_target<0){
//                robot.TR.setPower(response);
//                robot.BR.setPower(response);
//            } else if (response < 0 && R_PID.current_target > 0){
//                robot.TR.setPower(-response);
//                robot.BR.setPower(-response);
//            } else if (response > 0 && R_PID.current_target > 0){
//                robot.TR.setPower(response);
//                robot.BR.setPower(response);
//            } else {
//                robot.TR.setPower(-response);
//                robot.BR.setPower(-response);
//            }
//        }
//    };
//
//    // Sensor Values
//    private double L_ENCODER, R_ENCODER;
//
//    private State BULK_READ = new State() {
//        @Override
//        public void state_action() {
//            L_ENCODER = (robot.TL.getCurrentPosition() + robot.BL.getCurrentPosition())/2.0;
//            R_ENCODER = (robot.TR.getCurrentPosition() + robot.BR.getCurrentPosition())/2.0;
//        }
//    };
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        FiniteStateMachine.change_state(BULK_READ);
//
//        waitForStart();
//
//        driveDistance(0.40);
//        driveDistance(-0.40);
//        turnLeft(90);
//
//        robot.resetEncoders();
//        robot.TL.setPower(0.0);
//        robot.TR.setPower(0.0);
//        robot.BL.setPower(0.0);
//        robot.BR.setPower(0.0);
//    }
//
//    public void driveDistance(double distance){
//        int target_encoder_ticks = setPIDTarget(distance);
//        robot.resetEncoders();
//        FiniteStateMachine.change_state(BULK_READ);
//        L_PID.giveInput(L_ENCODER, (L_ENCODER+target_encoder_ticks));
//        R_PID.giveInput(R_ENCODER, (R_ENCODER+target_encoder_ticks));
//
//        // This while loop, although lengthy, is actually pretty simple.
//        // It states that the loop should continue until the PID TARGET has been reached for all
//        // wheels, and no stop has been requested.
//        while ((!L_PID.PID_TARGET_REACHED || !R_PID.PID_TARGET_REACHED) && ((opModeIsActive()
//                && !isStopRequested()))){
//            FiniteStateMachine.change_state(BULK_READ);
//
//            L_PID.giveInput(L_ENCODER);
//            R_PID.giveInput(R_ENCODER);
//        }
//    }
//
//    public void turnLeft(double degree){
//        double distance = (2*3.14*.18*degree)/360;
//        int target_encoder_ticks = setPIDTarget(distance);
//        robot.resetEncoders();
//        FiniteStateMachine.change_state(BULK_READ);
//        L_PID.giveInput(L_ENCODER, (L_ENCODER+target_encoder_ticks));
//        R_PID.giveInput(R_ENCODER, (R_ENCODER-target_encoder_ticks));
//
//        // This while loop, although lengthy, is actually pretty simple.
//        // It states that the loop should continue until the PID TARGET has been reached for all
//        // wheels, and no stop has been requested.
//        while ((!L_PID.PID_TARGET_REACHED || !R_PID.PID_TARGET_REACHED) && ((opModeIsActive()
//                && !isStopRequested()))){
//            FiniteStateMachine.change_state(BULK_READ);
//
//            L_PID.giveInput(L_ENCODER);
//            R_PID.giveInput(R_ENCODER);
//        }
//    }
//
//    public void turnRight(double degree){
//        int target_encoder_ticks = (int)((28/9)*degree);
//        robot.resetEncoders();
//        FiniteStateMachine.change_state(BULK_READ);
//        L_PID.giveInput(L_ENCODER, (-target_encoder_ticks));
//        R_PID.giveInput(R_ENCODER, (target_encoder_ticks));
//
//        // This while loop, although lengthy, is actually pretty simple.
//        // It states that the loop should continue until the PID TARGET has been reached for all
//        // wheels, and no stop has been requested.
//        while ((!L_PID.PID_TARGET_REACHED || !R_PID.PID_TARGET_REACHED) && ((opModeIsActive()
//                && !isStopRequested()))){
//            FiniteStateMachine.change_state(BULK_READ);
//
//            L_PID.giveInput(L_ENCODER);
//            R_PID.giveInput(R_ENCODER);
//        }
//    }
//    public int setPIDTarget(double x){
//        // This takes a distance in meters, but it should be converted to encoders
//        // distance/circumference of wheel = # of rev; 1 rev is 280 encoder ticks
//        return (int)((double)(x/(2*3.14*wheel_rad)) * 1120);
//    }
//}