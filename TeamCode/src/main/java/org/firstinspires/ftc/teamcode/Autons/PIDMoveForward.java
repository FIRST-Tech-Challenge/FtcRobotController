//package org.firstinspires.ftc.teamcode.Autons;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
////
//@Autonomous
//@Config
//public class PIDMoveForward extends LinearOpMode {
//    private PIDController controller;
//
//    public static double P = 0;
//    public static double I = 0;
//    public static double D = 0;
//    public static double x_pos = 0;
//    public static double y_pos = 0;
//    public static double prevHeading = 0;
//    public static double prevEncoder = 0;
//    public static double change = 0;
//
//
//    public static double targetTicks = 4000;
//
//    private Drivetrain drivetrain;
//    //change ticks to inches later
//    //i am 13 inches
//    public void odom() throws InterruptedException {
//        // ds= {2rh+sin(theta/2), 2rvsin(theta/2)} {cos(-thetac)  -sin(-thetac),  sin(-thetac) cos(-thetac)}
//        double curHeading = drivetrain.getHeading();
//        double deltaTheta = curHeading - prevHeading;
//        double distance = drivetrain.get1Position() - prevEncoder;
//        prevEncoder = drivetrain.get1Position();
//
//        double thetaChange = prevHeading + deltaTheta / 2;
//        double dv;
//        if (deltaTheta == 0) {
//            dv = distance;
//        } else {
//            double vertical_radius = distance / deltaTheta;
//            dv = 2 * vertical_radius * Math.sin(deltaTheta / 2);
//        }
//        double x_change = dv * Math.cos(thetaChange);
//        double y_change = dv * Math.sin(thetaChange);
//        x_pos += x_change;
//        y_pos += y_change;
//        prevHeading = curHeading;
//        telemetry.addData("x_pos", x_pos);
//        telemetry.addData("y_pos", y_pos);
//    }
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        PIDController controller = new PIDController(P, I, D);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        drivetrain = new Drivetrain();
//        drivetrain.init(hardwareMap);
//
//        waitForStart();
//
//        //double targetTicks = targetDistance * (Drivetrain.TICKS_PER_REV / (2 * Math.PI * Drivetrain.WHEEL_RADIUS_INCHES * Drivetrain.GEAR_RATIO));
//        controller.setSetPoint(targetTicks);
//
//
//        while (opModeIsActive() && !isStopRequested()) {
//            odom();
////            controller.setPID(P, I, D);
//            double currentTicks = drivetrain.get1Position();
//            double current2Ticks = drivetrain.get2Position();
//            double currentHeading = drivetrain.getHeading();
////            double pid = controller.calculate(currentTicks, targetTicks);
////            drivetrain.move(pid, 0, 0);
//
//            telemetry.addData("Current Position (ticks)", currentTicks);
//            //telemetry.addData("Target (ticks)", targetTicks);
//            //telemetry.addData("Power", pid);
//            telemetry.addData("encoder 2 ticks", current2Ticks);
//            telemetry.addData("heading in radians", currentHeading);
//            telemetry.update();
//        }
//
//        drivetrain.setPowers(0);
//    }
//}
//
//
//
///* Kp = someValue;
//Ki = someValue;
//Kd = someValue;
//
//reference = someValue;
//lastReference = reference;
//integralSum = 0;
//
//lastError = 0;
//
//maxIntegralSum = someValue;
//
//a = 0.8; // a can be anything from 0 < a < 1
//previousFilterEstimate = 0;
//currentFilterEstimate = 0;
//
//// Elapsed timer class from SDK, please use it, it's epic
//ElapsedTime timer = new ElapsedTime();
//
//while (setPointIsNotReached) {
//
//
//// obtain the encoder position
//encoderPosition = armMotor.get1Position();
//// calculate the error
//error = reference - encoderPosition;
//
//errorChange = (error - lastError)
//
//// filter out hight frequency noise to increase derivative performance
//currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
//previousFilterEstimate = currentFilterEstimate;
//
//// rate of change of the error
//derivative = currentFilterEstimate / timer.seconds();
//
//// sum of all error over time
//integralSum = integralSum + (error * timer.seconds());
//
//
//        // max out integral sum
//        if (integralSum > maxIntegralSum) {
//integralSum = maxIntegralSum;
//    }
//
//            if (integralSum < -maxIntegralSum) {
//integralSum = -maxIntegralSum;
//    }
//
//            // reset integral sum upon setpoint changes
//            if (reference != lastReference) {
//integralSum = 0;
//        }
//
//out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//        armMotor.setPower(out);
//
//lastError = error;
//
//lastReference = reference;
//
//// reset the timer for next time
//    timer.reset();
//
//}
//*/