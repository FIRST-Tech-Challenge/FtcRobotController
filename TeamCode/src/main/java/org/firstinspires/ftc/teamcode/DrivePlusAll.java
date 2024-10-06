package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// servoPlacer added
// Need lift code, launcher code and ARM code
@TeleOp
// @Disabled
public class DrivePlusAll extends LinearOpMode {
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    Servo servoPlacer;
    Servo servoLauncher;

    private ElapsedTime runtime = new ElapsedTime();


    //
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors\
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor IntakeWheel = hardwareMap.dcMotor.get("IntakeWheel");
        //Intake Wheel setting
        IntakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //servo position

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //servo setting
        servoPlacer = hardwareMap.get(Servo.class, "servoPlacer");
        servoLauncher = hardwareMap.get(Servo.class, "servoLauncher");
        //for lift
        DcMotor liftRight = hardwareMap.dcMotor.get("liftRight");
        DcMotor liftLeft = hardwareMap.dcMotor.get("liftLeft");
        //set direction of the lift motors be different directions
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //Arm code
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        //DcMotor liftLeft = hardwareMap.dcMotor.get("liftLeft");
        //set direction of the lift motors be different directions
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        servoPlacer.setPosition(MAX_POS * 0.1);//start with MAX position

        //if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            float t = gamepad1.left_trigger;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower;
            double rearLeftPower;
            double frontRightPower;
            double rearRightPower;


            if (t == 00) {
                frontLeftPower = (y + x + rx) / denominator;
                rearLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                rearRightPower = (y + x - rx) / denominator;
            } else {
                frontLeftPower = 0.5 * (y + x + rx) / denominator;
                rearLeftPower = 0.5 * (y - x + rx) / denominator;
                frontRightPower = 0.5 * (y - x - rx) / denominator;
                rearRightPower = 0.5 * (y + x - rx) / denominator;
            }
            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            boolean wf = gamepad2.a; // a is forward
            boolean wb = gamepad2.b; // b is backward


//below are intake wheel code
            double wheelspeed = 0;

            //double wheelpower = 0;
            if (wf == true) {
                wheelspeed = 1;
                IntakeWheel.setPower(wheelspeed);
            }
            if (wb == true) {
                wheelspeed = -1;
                IntakeWheel.setPower(wheelspeed);
            } else {
                wheelspeed = 0;
                IntakeWheel.setPower(wheelspeed);
            }
//below are servoPlacer code
            if (gamepad2.x == true) {
                //placer_position = MIN_POS;
                servoPlacer.setPosition(MIN_POS);
            } else {
                //placer_position = MIN_POS;
                servoPlacer.setPosition(MAX_POS * 0.65);
            }

            //servo launcher code
            double Launcherposition = (MIN_POS); //start at min
            if (gamepad2.right_bumper == true) {
                Launcherposition = MAX_POS;
                servoLauncher.setPosition(Launcherposition);

            } else {
                Launcherposition = MIN_POS;
                servoLauncher.setPosition(Launcherposition);

            }


            //lift code
            boolean liftup = gamepad2.dpad_up; //lift go up
            boolean liftdown = gamepad2.dpad_down; //lift go down
            double liftspeed = 1;
            if (liftup == true) {
                liftRight.setPower(liftspeed * 2);
                liftLeft.setPower(liftspeed * 2);
            }
            if (liftdown == true) {
                liftRight.setPower(-liftspeed);
                liftLeft.setPower(-liftspeed);
            } else {
                liftspeed = 0;
                liftRight.setPower(liftspeed);
                liftLeft.setPower(liftspeed);
                //arm code
                double armup = gamepad2.right_stick_y; //arm move up or down
                //boolean liftdown = gamepad2.dpad_down; //lift go down
                //double armspeed = 0.25;
                if (armup > 0) {
                    arm.setPower(armup);

                }
                if (armup < 0) {
                    arm.setPower(armup * 1);
                } else {
                    arm.setPower(armup * 0);
                }
            }
            //added the following for the launch angle January 19, 2024
            if (gamepad2.left_bumper==true){

                double airplane_angle =1;//this is the power to raise it
                arm.setPower(airplane_angle);
                runtime.reset();//will change time to find the right angle for launch airplane//1.5 sec is 31 degree
                while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();//1.25 sec is 22 degree, 1 sec is 20-21 degree
                }
            }
        }

    }

}

