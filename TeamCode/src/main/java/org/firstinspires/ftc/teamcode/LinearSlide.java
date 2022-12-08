package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

//poop


@TeleOp(name="LinearSlide", group="Linear Opmode")

public class LinearSlide extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor liftMotor = null;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;

    int direction = 0;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    public void waitTime(double time){
        runtime.reset();
        while(runtime.seconds()<time){
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        liftMotor  = hardwareMap.get(DcMotor.class, "lift_motor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // If making a new Auto or TeleOp file, make sure to include waitForStart so it can pass
        // inspection and you don't get dq-ed from tourneys (mainly for Auto)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Look through old files to try to find POV Mode. Currently using tank drive code

            boolean liftUp = gamepad1.dpad_up;
            boolean liftDown = gamepad1.dpad_down;

            if(liftUp){
                liftMotor.setPower(.1);
                waitTime(.5);
                liftMotor.setPower(0);
            }
            if(liftDown){
                liftMotor.setPower(-.1);
                waitTime(.5);
                liftMotor.setPower(0);
            }
            // Show the elapsed game time and wheel power.
        }
    }
}