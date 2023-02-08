package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LinearSlide", group="Linear Opmode")

public class LinearSlide extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx liftMotor = null;
    private Servo servoGrabber1 = null;
    private Servo servoGrabber2 = null;

    static final double MIN_LIFT_POS =  0;
    static final double MAX_LIFT_POS =  5000;

    static final double MAX_POS     =    .5;
    static final double MAX_POS2    =    .50;
    static final double MIN_POS     =     1;
    static final double MIN_POS2    =     0;

    double direction = 0;
    double position = 1;
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
        liftMotor  = hardwareMap.get(DcMotorEx.class, "lift_motor");
        servoGrabber1 = hardwareMap.get(Servo.class, "servo_grabber_one");
        servoGrabber2 = hardwareMap.get(Servo.class, "servo_grabber_two");

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoGrabber1.setPosition(position);
        servoGrabber2.setPosition(position2);

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lift = -gamepad2.left_stick_y;
            double grabber = -gamepad2.right_stick_y;

            if(lift > .05 && liftMotor.getCurrentPosition() < MAX_LIFT_POS){
                liftMotor.setPower(1);
            }else{
                liftMotor.setPower(0);
            }
            if(lift < -.05 && liftMotor.getCurrentPosition() > MIN_LIFT_POS){
                liftMotor.setPower(-.4);
            }else{
                liftMotor.setPower(0);
            }

            if(grabber>.1 || grabber<-.1){
                if(grabber<.05){
                    direction = .1;
                }else{
                    direction = -.1;
                }
            }else{
                direction = 0;
            }

            position+=direction;
            position2+= -direction;

            if(position < MAX_POS || position2 > MAX_POS2){
                position=MAX_POS;
                position2=MAX_POS2;
            }

            if(position > MIN_POS || position2 < MIN_POS2){
                position=MIN_POS;
                position2=MIN_POS2;
            }

            servoGrabber1.setPosition(position);
            servoGrabber2.setPosition(position2);
            telemetry.addData("Status", "Direction: " + direction);
            telemetry.update();
        }
    }
}