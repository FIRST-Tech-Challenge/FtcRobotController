//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//
///**
// * Created by tom on 9/26/17.
// * This assumes that you are using a REV Robotics Expansion Hub
// * as your DC motor controller. This op mode uses the extended/enhanced
// * PID-related functions of the DcMotorControllerEx class.
// * The REV Robotics Expansion Hub supports the extended motor controller
// * functions, but other controllers (such as the Modern Robotics and
// * Hitechnic DC Motor Controllers) do not.
// */
//
//@Autonomous(name="FeedForward", group = "Examples")
//public class FeedForward extends LinearOpMode {
//
//    // our DC motor.
//    private DcMotor motor;
//
//    public static final double NEW_P = 2.5;
//    public static final double NEW_I = 0.1;
//    public static final double NEW_D = 0.2;
//
//    public void runOpMode() {
//        // get reference to DC motor.
//
//        // wait for start command.
//        waitForStart();
//
//        // get a reference to the motor controller and cast it as an extended functionality controller.
//        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
//        motor = hardwareMap.get(DcMotor.class, "leftMotor");;
//
//        // get the port number of our configured motor.
//        motor.
//
//        // get the PID coefficients for the RUN_USING_ENCODER  modes.
//
//        // change coefficients.
//        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
//        motor.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//
//        // re-read coefficients and verify change.
//        PIDCoefficients pidModified = motor.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // display info to user.
//        while(opModeIsActive()) {
//            telemetry.addData("Runtime", "%.03f", getRuntime());
//            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
//                    pidOrig.p, pidOrig.i, pidOrig.d);
//            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
//                    pidModified.p, pidModified.i, pidModified.d);
//            telemetry.update();
//        }
//    }
//}