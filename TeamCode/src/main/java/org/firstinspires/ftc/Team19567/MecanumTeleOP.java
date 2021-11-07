package org.firstinspires.ftc.Team19567; //The "namespace" for the project is declared here. To distinguish it from other teams' packages, Team19567 is used.

//Import necessary packages/libraries

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mecanum TeleOP", group="Iterative Opmode") //Gives the TeleOp its name in the driver station menu and categorizes it as a TeleOp (Iterative OpMode)
public class MecanumTeleOP extends OpMode {           //Declares the class TestOPIterative, which is a child of OpMode
    //Declare OpMode members
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private DcMotor intakeDC = null;
    private DcMotor carouselDC = null;
    private DcMotor linearSlideDC = null;
    private boolean slowMode = false;
    private double acc = 1.0;
    private Servo servo1 = null;
    private boolean isServoMaxed = false;
    private double servo1Pos = 0.0;
    private double carouselDCPos=0.0;
    private double intakeDCPos = 0.0;

    @Override
    public void init() {
        //Get the motors from the robot's configuration

        leftDCFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightDCFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftDCBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDCBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeDC = hardwareMap.get(DcMotor.class, "intake");
        carouselDC = hardwareMap.get(DcMotor.class, "carousel");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        //Set direction to be forward in case the robot's motors are oriented otherwise; can change FORWARD to REVERSE if necessary

        leftDCFront.setDirection(DcMotor.Direction.FORWARD);
        rightDCFront.setDirection(DcMotor.Direction.FORWARD);
        leftDCBack.setDirection(DcMotor.Direction.FORWARD);
        rightDCBack.setDirection(DcMotor.Direction.FORWARD);
        intakeDC.setDirection(DcMotor.Direction.FORWARD);
        carouselDC.setDirection(DcMotor.Direction.FORWARD);
        linearSlideDC.setDirection(DcMotor.Direction.FORWARD);
        linearSlideDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Awaiting Start");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Started");
        telemetry.update();
        runtime.reset(); //Reset runtime
    }

    @Override
    public void loop() {
        /* Mecanum drive applies forces at a 45 degree angle to its wheels, instead of directly (like most non-holonomic chassis);
        this means that its axis of movement is not the tradition al x-y plane, but rather said plane rotated 45 degrees. This also means that the wheels
        of a mecanum chassis should be as close to a square as possible. That way, the rotated plane's axes push straight through the wheels for
        maximum efficiency. */

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //Gets the amount that we want to translate
        double angleDC = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; /* This gets the angle theta we desire to turn at. It is subtracted
        by PI/4 radians, which equals 45 degrees. One can also add 45 degrees. It doesn't matter so long as the axes of the robot are rotated 45 degrees. */

        /* The axes we want to move on are the hypotenuse of the right triangle with non-right angle angleDC. The size of this hypotenuse is determined by the
        value of r, which we computed earlier. Thus, the amount that we want to move in the standard x-y plane are the legs of the right triangle. To determine
        which wheels drive the x leg and which drive the y leg (base), we must take a look at the configuration of the wheels. In most robots, which have the wheels
        pointed outwards in an X shape (maximum efficiency), we see that the front left and rear right wheels point in the y direction, while the rear left
        and front right wheels point in the x direction. Thus, we use cosine for the first group and sine for the second group. */

        final double leftFrontSpeed = r*acc*Math.sin(angleDC) + gamepad1.right_stick_x; //Using the math explained above, we can obtain the values we want to multiply each wheel by. acc is the variable which controls the overall multiplier of how fast we want to go.
        final double rightFrontSpeed = r*acc*Math.cos(angleDC) - gamepad1.right_stick_x;
        final double leftBackSpeed = r*acc*Math.cos(angleDC) + gamepad1.right_stick_x;
        final double rightBackSpeed = r*acc*Math.sin(angleDC) - gamepad1.right_stick_x;

        if(gamepad1.x) slowMode = !slowMode;
        if(gamepad1.right_bumper) {
            servo1Pos += 0.03;
        }
        else if(gamepad1.left_bumper) {
            servo1Pos -= 0.03;
        }
        if(gamepad1.a) {
            linearSlideDC.setPower(1.0);
        }


        if(slowMode) acc = 0.3; //If slowmode is on, then change movement multiplier speed to 0.3 (30%)
        else acc = 1.0; //Otherwise, keep it at 1.0 (100%)

        leftDCFront.setPower(leftFrontSpeed); //Set all the motors to their corresponding powers/speeds
        rightDCFront.setPower(rightFrontSpeed);
        leftDCBack.setPower(leftBackSpeed);
        rightDCBack.setPower(rightBackSpeed);
        intakeDC.setPower(gamepad1.left_trigger);
        carouselDC.setPower(gamepad1.right_trigger);

        telemetry.addData("Status", "Looping"); //Add telemetry to show that the program is currently in the loop function
        telemetry.addData("Runtime", runtime.toString() + " Milliseconds"); //Display the runtime
        telemetry.addData("DCMotors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack(%.2f)",
                leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed); //In (%.2f), the % means that special modifier is to follow, that modifier being .2f. In .2f, the .2 means to round to to digits after the decimal point, and the f means that the value to be rounded is a float.
        //(%.2f) is used here so that the displayed motor speeds aren't excessively long and don't clutter the screen.
        telemetry.addData("Servos", "servo1 (%.2f)", servo1.getPosition()); //Same with (%.2f) here; we just use it to display the servo value
        telemetry.update(); //Updates the telemetry
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}