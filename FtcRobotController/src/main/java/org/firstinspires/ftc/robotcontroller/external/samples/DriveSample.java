package org.firstinspires.ftc.robotcontroller.external.samples;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MainDrive")
//Our Class for Drive Controlled period of the game
public class DriveSample extends LinearOpMode {

    //Declares the Variables for all of our motors and servos
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    //This below runs the OpMode/Program for our robot
    @Override
    public void runOpMode() throws InterruptedException {

        //Initializing Hardware in method down below called initHardware();
        initHardware();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //Gamepad controls
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                //Drive variables used in the calculations to run our motors
                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);
                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                /**
                 In basics this is taking the x and y of the left stick making them into an angle
                 with the power being the hypot which is the square root of the sum of squares of the inputs
                 more info here https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/hypot
                 then takes the sin and cos of the angle making sure to convert to radians. It then creates a max
                 using the absolute value of the sin and cos.

                 The idea is that where you are going is angle theta with each wheel being a vector and when combined make the target direction when rotated 45 degrees

                 Found on YT www.youtube.com/watch?v=gnSW2QpkGXQ which is a video about coding for mecanum drive wheels
                 */


                //Calculations for our drive motors

                double fl = (power * cos / max + turn);
                double fr = (power * sin / max - turn);
                double bl = (power * sin / max + turn);
                double br = (power * cos / max - turn);

                /**
                 In continuation the power is then calculated with the angles multiplied by the sin or cos divided the difference or sum of the max and turn
                 */

                //If statement below is to make sure one motor does not exceed the power limit making it scale down

                if ((power + Math.abs(turn)) > 1) {
                    fl /= power + Math.abs(turn);
                    fr /= power + Math.abs(turn);
                    bl /= power + Math.abs(turn);
                    br /= power + Math.abs(turn);
                }

                initTelemetry();
                telemetry.update();

            }
        }
    }

    private void initHardware(){
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        //Servos
        //Reversal of Motors
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Zero Power Behaviors
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    private void initTelemetry() {
        //Put telemetry here
    }
}