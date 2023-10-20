package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

@TeleOp(name = "MecanumDrive", group = "Opmode" )
public class MecanumDrive extends OpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftWheel = null;
    private DcMotor BackLeftWheel = null;
    private DcMotor FrontRightWheel = null;
    private DcMotor BackRightWheel = null;
    @Override
    public void init() {
        //Assigning motors
        FrontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        BackLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        FrontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        BackRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");

        //Reversing motors
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        BackRightWheel.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }


    @Override
    public void loop() {
        //Get joystick measures
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;


        double turn = gamepad1.right_stick_x;

        //Calculate vector-related vars
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        //Initial motor power
            double FrontLeftPower  = power * cos/max + turn;
            double BackLeftPower   = power * sin/max + turn;
            double FrontRightPower = power * sin/max - turn;
            double BackRightPower  = power * cos/max - turn;

            //Checks and adjusts if power + turn is impossible (achieves optimal speed)
            if ((power + Math.abs(turn)) > 1 ){
                FrontLeftPower  /= power + Math.abs(turn);
                BackLeftPower   /= power + Math.abs(turn);
                FrontRightPower /= power + Math.abs(turn);
                BackRightPower  /= power + Math.abs(turn);
            }

            //set motor power
            FrontLeftWheel.setPower(FrontLeftPower * .50);
            BackLeftWheel.setPower(BackLeftPower* .50);
            FrontRightWheel.setPower(FrontRightPower* .50);
            BackRightWheel.setPower(BackRightPower* .50);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", FrontLeftPower, FrontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BackLeftPower, BackRightPower);
            telemetry.update();


    }
}
