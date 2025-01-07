package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.MainDrive;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;
import org.firstinspires.ftc.teamcode.Debug.Debug;

@TeleOp(name = "TeleOpMain", group = "Main")
public class TeleOpMain extends LinearOpMode {

    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

    private DcMotorEx leftViper;
    private DcMotorEx rightViper;

    private Servo leftWrist;
    private Servo rightWrist;

    private Servo leftGrabber;
    private Servo rightGrabber;

    private double targetPower;
    private double increment;
    private double incrementDividend;
    private boolean isAccelerating;
    private long lastUpdateTime;
    private double currentPower;
    private int updateDelay = 10;

    float frontMultiplier = 1;
    float backMultiplier = 1;

    boolean debugMode = false;
    boolean emergencyStop = false;

    @Override
    public void runOpMode() {
        // define part objects
        HorizontalSlide hSlide = new HorizontalSlide(this, 3);
        ViperSlide viperSlide = new ViperSlide(this);
        Intake intake = new Intake(this, hSlide);
        MainDrive mainDrive = new MainDrive(this);

        Debug debug = new Debug(this);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();




        while (opModeIsActive() && !emergencyStop) {
            // Emergency stop
//            if (gamepad1.dpad_left && gamepad1.b) {
//                emergencyStop = true;
//                telemetry.addData("Emergency Stop", "Activated");
//                telemetry.update();
//                break;
//            }

            // Gamepad 1 controls



            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;



            // TODO toggleable deadzones
//            if (gamepad1.left_stick_x < 40 && gamepad1.left_stick_x > 60) {
//                x = 50;
//            }
//            if (gamepad1.left_stick_y < 45 && gamepad1.left_stick_y > 55) {
//                y = 50;
//            }


            mainDrive.checkInputs(
                    -gamepad1.left_stick_y,       // y
                    gamepad1.left_stick_x,        // x
                    gamepad1.right_stick_x,       // rx
                    gamepad1.y,                   // forward
                    gamepad1.b,                   // reverse
                    gamepad1.left_trigger,       // 1/2 speed
                    gamepad1.right_trigger         // 1/4 speed
            );

            viperSlide.checkInputs(
                    gamepad2.left_trigger,        // retract
                    gamepad2.right_trigger,       // extend
                    gamepad2.guide,               // reset encoders
                    gamepad2.back,                // hold viper position
                    gamepad2.x,                   // bucket rest
                    gamepad2.y,                   // bucket score
                    gamepad2.b,                   // open bucket
                    gamepad2.a,                   // close bucket
//                    gamepad2.b,                   // open/close bucket
                    gamepad2.dpad_down,           // grab specimen
                    gamepad2.dpad_up,             // release specimen
                    gamepad2.dpad_left,           // bucket specimen
                    gamepad2.dpad_right
            );

            hSlide.checkInputs(
                    gamepad1.right_bumper,        // extend
                    gamepad1.left_bumper,         // retract
                    gamepad1.guide                // reset encoders
            );

            intake.checkInputs(
                    gamepad1.x,                   // wrist down position
                    gamepad1.a,                   // wrist up position
                    gamepad1.back,                // wrist half
//                    (gamepad1.dpad_left         // wrist modifier
                    gamepad2.right_bumper,        // intake on
                    gamepad2.left_bumper          // intake reverse

            );



            debug.checkDebugButtons(gamepad1);

            telemetry.update();
        }
    }

}