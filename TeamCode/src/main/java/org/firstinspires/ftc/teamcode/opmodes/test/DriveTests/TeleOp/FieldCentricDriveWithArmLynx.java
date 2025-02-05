/*
package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems.DriveCode;
import org.firstinspires.ftc.teamcode.opmodes.Subsystems.HangCode;
import org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems.IntakeCode;
import org.firstinspires.ftc.teamcode.opmodes.Subsystems.PresetArmCode;
import org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems.WristCode;

//@TeleOp(name = "TeleOpLynx", group = "TeleOpFINAL")
public class FieldCentricDriveWithArmLynx extends LinearOpMode {

    // Subsystems
    private PresetArmCode armControl;
    private IntakeCode intakeControl;
    private WristCode wristControl;
    private DriveCode driveControl;
    private HangCode hangControl;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare the motors and servos

        private DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        private DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        private DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"backRightMotor");
        private DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeftMotor");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

/*
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

 */
/*
        DcMotor linearSlideMotor = hardwareMap.dcMotor.get("armMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("liftMotor");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intake");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist");
        DcMotor HangMotor1 = hardwareMap.dcMotor.get("HM1");
        DcMotor HangMotor2 = hardwareMap.dcMotor.get("HM2");

        // Initialize and configure IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);

        // Reverse the necessary motors - Right = Forward, Left = Reverse
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); //Forward
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
   //     backRightMotor.setDirection(DcMotor.Direction.FORWARD);

*/
        // Set zero power behavior
/*        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/
        // Initialize subsystems
 /*       armControl = new PresetArmCode(linearSlideMotor, armMotor);
        intakeControl = new IntakeCode(intakeServo);
        wristControl = new WristCode(wristServo);
        driveControl = new DriveCode(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);
        hangControl = new HangCode(HangMotor1, HangMotor2);
        waitForStart();
*/
/*
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()) {
            // Will run two bulk read per cycles,
            // as frontLeftMotor.getCurrentPosition() is called twice
        }
*/
/*
        while (opModeIsActive()) {

            /*
            Overall, Gamepad 1 Controls Drive, Wrist, Intake.
                     Gamepad 2 Controls ArmPrests and LinearSlide
             */
/*
            // DRIVE CODE
            int frontLeftEncoderPos = frontLeftMotor.getCurrentPosition();
            int frontRightEncoderPos = frontRightMotor.getCurrentPosition();
            int backLeftEncoderPos = backLeftMotor.getCurrentPosition();
            int backRightEncoderPos = backRightMotor.getCurrentPosition();
*/
  //          driveControl.drive(gamepad1);
            /*
            Gamepad1 Joysticks:
            a) LeftStick, when moved in Y direction controls up and down movement
            b) LeftStick, when moved in X direction controls left and right (strafing) movement
            c) RightStick, when moved in the +X direction right, and left in the -X direction
             */

            // ARM CONTROL
//            armControl.controlArmPreset(gamepad2);
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
            f) Using Dpad BLUERIGHT - Lift to high position, i.e Upper Basket
             */

            // INTAKE CONTROL
//            intakeControl.controlIntake(gamepad1.right_bumper, gamepad1.left_bumper);
            /*
            Gamepad1 Trigger:
            a) Using Left Trigger - pushes block OUTWARD
            b) Using Right Trigger - take block IN
             */

            // WRIST CONTROL
  //          wristControl.controlWrist(gamepad2.square, gamepad2.circle);
            /*
            Gamepad2 Buttons (PS4 Controller):
            a) Using Square Button - Bends Wrist to the Left
            b) Using Circle Trigger - Makes Wrist straight
             */

            //HANG CONTROL
//            hangControl.controlHang(gamepad1);
            /*
            Gamepad1 Buttons (Logitech Controller):
            a) Using A button - Extends the linearSlides
            b) Using B button - Retracts the linearSlides
            Note: HangMotor1 Encoder Value should ALWAYS be equal to HangMotor2 Encoder Value
             */
/*
            // TELEMETRY
            telemetry.addData("Front Left Power", frontLeftMotor.getPower());
            telemetry.addData("Back Left Power", backLeftMotor.getPower());
            telemetry.addData("Front Right Power", frontRightMotor.getPower());
            telemetry.addData("Back Right Power", backRightMotor.getPower());
            telemetry.addData("LinearSlide Motor Power", linearSlideMotor.getPower());
            telemetry.addData("Arm Motor Power", armMotor.getPower());
            telemetry.addData("Intake Power", intakeServo.getPower());
            telemetry.addData("Wrist Position", wristServo.getPosition());
            telemetry.addData("Linear Slide Encoder", linearSlideMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Encoder", armMotor.getCurrentPosition());
            telemetry.addData("Hang Motor1 Encoder", HangMotor1.getCurrentPosition());
            telemetry.addData("Hang Motor2 Encoder", HangMotor2.getCurrentPosition());
            telemetry.update();
        }
    }
}
*/
