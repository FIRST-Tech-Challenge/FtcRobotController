package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestOPLinear", group="Linear Opmode")

public class TestOPLinear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private boolean slowMode = false;
    private double acc = 1.0;
    private Servo servo1 = null;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        leftDCFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightDCFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftDCBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDCBack = hardwareMap.get(DcMotor.class, "rightBack");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        //Set direction to be forward in case the robot's motors are oriented otherwise; can change FORWARD to REVERSE if necessary

        leftDCFront.setDirection(DcMotor.Direction.FORWARD);
        rightDCFront.setDirection(DcMotor.Direction.FORWARD);
        leftDCBack.setDirection(DcMotor.Direction.FORWARD);
        rightDCBack.setDirection(DcMotor.Direction.FORWARD);

        //Set DC motors to run with encoder

        leftDCFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDCFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDCBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDCBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDCFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDCFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDCBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDCBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); //Wait for the driver to press Init
        runtime.reset(); //Reset the runtime

        //Run until the driver presses Stop
        while (opModeIsActive()) {

            double leftFrontSpeed = 0.0;
            double rightFrontSpeed = 0.0;
            double leftBackSpeed = 0.0;
            double rightBackSpeed = 0.0;

            telemetry.addData("Status", "Running"); //Add telemetry to show that the program is currently in the loop function
            telemetry.addData("Runtime", runtime.toString() + " Milliseconds"); //Display the runtime
            telemetry.addData("DCMotors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack(%.2f)",
                    leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed); //In (%.2f), the % means that special modifier is to follow, that modifier being .2f. In .2f, the .2 means to round to to digits after the decimal point, and the f means that the value to be rounded is a float.
            //(%.2f) is used here so that the displayed motor speeds aren't excessively long and don't clutter the screen.
            telemetry.addData("Servos", "servo1 (%.2f)", servo1.getPosition()); //Same with (%.2f) here; we just use it to display the servo value
            telemetry.update(); //Updates the telemetry
        }
    }
}