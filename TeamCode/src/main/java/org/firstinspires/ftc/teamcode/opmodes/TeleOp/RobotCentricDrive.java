package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.opmodes.Subsystems.ClawCode;
import org.firstinspires.ftc.teamcode.opmodes.Subsystems.HangCode;
import org.firstinspires.ftc.teamcode.opmodes.Subsystems.PresetArmCode;
//import org.firstinspires.ftc.teamcode.opmodes.Subsystems.PresetSlideCode;
//import org.firstinspires.ftc.teamcode.opmodes.Subsystems.SlidePreset;

@TeleOp(name = "TeleOpRobotCentric", group = "TeleOpFINAL")
//@Disabled
public class RobotCentricDrive extends LinearOpMode {

    // Subsystems
    private PresetArmCode armControl;
//    private PresetSlideCode slideControl;
    private ClawCode clawControl;
    private HangCode hangControl;
//    private SlidePreset slideControl;

    DcMotorEx linearSlideMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare the motors and servos
//        DcMotorEx linearSlideMotor = hardwareMap.get(DCMotorEx,"armMotor");
         linearSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "armMotor");

        Servo rollServo = hardwareMap.get(Servo.class, "roll");
        Servo yawServo = hardwareMap.get(Servo.class, "yaw");
        Servo pitchServo = hardwareMap.get(Servo.class, "pitch");
        Servo clawServo = hardwareMap.get(Servo.class, "claw");
        DcMotor HangMotor1 = hardwareMap.dcMotor.get("HM1");
        DcMotor HangMotor2 = hardwareMap.dcMotor.get("HM2");
        DcMotor armMotor = hardwareMap.dcMotor.get("liftMotor");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        if (isStopRequested()) return;

        // Initialize and configure IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);

        // Set zero power behavior
        linearSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setPositionPIDFCoefficients(10.0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize subsystems
        armControl = new PresetArmCode(linearSlideMotor, armMotor);

//        slideControl = new SlidePreset(hardwareMap);
        clawControl = new ClawCode(clawServo, rollServo, yawServo, pitchServo);
        hangControl = new HangCode(HangMotor1, HangMotor2);
        waitForStart();

        while (opModeIsActive()) {


            double y = Math.pow(-gamepad1.left_stick_y,3); // Remember, Y stick value is reversed
            double x = Math.pow(gamepad1.left_stick_x * 1.1,3); // Counteract imperfect strafing
            double rx = Math.pow(gamepad1.right_stick_x,3);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double speed = 2.5;
            if(gamepad1.left_trigger >0.1){
                speed = 1.2;
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), speed);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            // ARM CONTROL
            armControl.controlArmAndSlide(gamepad2);
            armControl.pickSpecimenFromWall(gamepad1);
         //   slideControl.controlSlide(gamepad2);
            /*
            LINEAR SLIDE
            Gamepad2 Trigger:
            a) Using Left Trigger - move linear slide backward
            b) Using Right Trigger - move linear slide forward

            ARM
            Gamepad2 Dpad:
            c) Using Dpad UP - Lift to highest position, for hanging
            d) Using Dpad DOWN - Lift to Lowest position, pick up blocks from submersible
            e) Using Dpad LEFT - Lift to low position, i.e Low Basket
            f) Using Dpad RIGHT - Lift to high position, i.e Upper Basket

            Gamepad1 Right Bumper:
             */

            // INTAKE CONTROL
            clawControl.controlClawPS4(gamepad2);
            clawControl.controlClawLogitech(gamepad1);
            /*
//Re-write controls accordingly
             */

            // WRIST CONTROL
            /*
//  Adjust accordingly - Note that you're normally supposed to make a variable for this,
// not just directly put in a double
    */


            // HANG CONTROL
            hangControl.controlHang(gamepad1);
            /*
            Gamepad1 Buttons (Logitech Controller):
            a) Using A button - Extends the linearSlides
            b) Using B button - Retracts the linearSlides
            Note: HangMotor1 Encoder Value should ALWAYS be equal to HangMotor2 Encoder Value
             */

            // TELEMETRY
            telemetry.addData("Front Left Power", frontLeftMotor.getPower());
            telemetry.addData("Back Left Power", backLeftMotor.getPower());
            telemetry.addData("Front Right Power", frontRightMotor.getPower());
            telemetry.addData("Back Right Power", backRightMotor.getPower());
            telemetry.addData("LinearSlide Motor Power", linearSlideMotor.getPower());
            telemetry.addData("Arm Motor Power", armMotor.getPower());
            telemetry.addData("Claw Power", clawServo.getPosition());
            telemetry.addData("Pitch Position", pitchServo.getPosition());
            telemetry.addData("Yaw Position", yawServo.getPosition());
            telemetry.addData("Roll Position", rollServo.getPosition());


            telemetry.addData("Linear Slide Encoder", linearSlideMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Encoder", armMotor.getCurrentPosition());
            telemetry.addData("Hang Motor1 Encoder", HangMotor1.getCurrentPosition());
            telemetry.addData("Hang Motor2 Encoder", HangMotor2.getCurrentPosition());
            if (gamepad1.a) {
                telemetry.addData("Button", "A1 Pressed");
            }
            if (gamepad1.b) {
                telemetry.addData("Button", "B1 Pressed");
            }
            if (gamepad1.x) {
                telemetry.addData("Button", "X1 Pressed");
            }
            if (gamepad1.y) {
                telemetry.addData("Button", "y1 Pressed");
            }
            if (gamepad2.right_bumper) {
                telemetry.addData("Bumper2", "Right Pressed");
            }
            if (gamepad2.circle) {
                telemetry.addData("Button", "Circle2 Pressed");
            }
            if (gamepad2.cross) {
                telemetry.addData("Button", "Cross2 Pressed");
            }
            telemetry.update();
        }
    }
}
