package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Test", group="Linear Opmode")
public class MotorTestOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo clawServo = null;
    private Servo rotationServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        front_right  = hardwareMap.get(DcMotor.class, "front_right");
//        front_left  = hardwareMap.get(DcMotor.class, "front_left");
//        rear_right  = hardwareMap.get(DcMotor.class, "rear_right");
//        rear_left  = hardwareMap.get(DcMotor.class, "rear_left");
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        runtime.reset();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
////            if (runtime.milliseconds() % 2000 < 1000)
////                motor.setPower(0.5);
////            else
////                motor.setPower(0.0);
//            rear_right.setPower(1.0);
//            rear_left.setPower(1.0);
//            front_right.setPower(1.0);
//            front_left.setPower(1.0);
//
//            // Show the elapsed game time.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.update();
//        }


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        clawServo  = hardwareMap.get(Servo.class, "claw");
        rotationServo = hardwareMap.get(Servo.class, "rotator");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        double position = 0;
        while (opModeIsActive()) {
            if (runtime.milliseconds() % 2000 == 0) {
                position = 1 - position;
            }

            clawServo.setPosition(position);
            rotationServo.setPosition(position);


            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
