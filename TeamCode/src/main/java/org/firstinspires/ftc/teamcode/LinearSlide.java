package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LinearSlide", group="Linear Opmode")

public class LinearSlide extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor liftMotor = null;
    private Servo servoGrabber1 = null;
    private Servo servoGrabber2 = null;

    static final double MAX_POS     =  270;
    static final double MAX_POS2    =    0;
    static final double MIN_POS     =  150;
    static final double MIN_POS2    =  120;

    int direction = 0;
    double position = 270;
    double position2 = 0;


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
        servoGrabber1 = hardwareMap.get(Servo.class, "servo_grabber_one");
        servoGrabber2 = hardwareMap.get(Servo.class, "servo_grabber_two");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lift = -gamepad2.right_stick_y;
            boolean openGrabber = gamepad2.dpad_up;
            boolean closeGrabber = gamepad2.dpad_down;

            if(lift > .05){
                liftMotor.setPower(1);
            }else{
                liftMotor.setPower(0);
            }
            if(lift < -.05){
                liftMotor.setPower(-.3);
            }else{
                liftMotor.setPower(0);
            }

            if(openGrabber){
                direction = -1;
                if(position<MAX_POS && position2>MAX_POS2) {
                    position = position + 1;
                    position2 = position2 - 1;
                }
                if(position > 1.0){
                    position=1.0;
                }
                telemetry.addData("Forward", "servo: " + position);
                servoGrabber1.setPosition(position);
                servoGrabber2.setPosition(position2);
            }

            if(closeGrabber){
                direction = 1;
                position = position - 1;
                position2 = position2 + 1;
                if(position < MIN_POS || position2 > MIN_POS2){
                    position = 0.0;
                    position2 = 120.0;
                }
                telemetry.addData("back", "servo: " + position);
                servoGrabber1.setPosition(position);
                servoGrabber2.setPosition(position2);
            }
        }
    }
}