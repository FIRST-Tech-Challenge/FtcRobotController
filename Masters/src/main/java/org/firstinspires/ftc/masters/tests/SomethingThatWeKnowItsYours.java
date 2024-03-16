//package org.firstinspires.ftc.masters.tests;
//
//
//import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//@TeleOp(name="SingleMotorTest")
//
//public class SomethingThatWeKnowItsYours extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // Make a new variable with type DcMotor with the name "active"
//        DcMotor active = hardwareMap.dcMotor.get("active");
//        CRServo wheel = hardwareMap.crservo.get("wheel");
//
//        // Call waitForStart();
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        // This is a while loop that does stuff and it loops
//        while (opModeIsActive()) {
//
//
//            // This sets the motor power to 0.5, making it spin at half speed
//            active.setPower(-0.8);
//            wheel.setPower(-1.0);
//
//            }
//
//        }
//
//    }
//
//
//// Hello human it seems you have scrolled to the end of the code i hope you have a great rest of your day :)
//
//// Your supreme lord and master here, what the fuck is the naming convention for this file.
//// Fix it, you knaves