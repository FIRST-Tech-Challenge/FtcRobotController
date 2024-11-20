
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="test servos", group="Linear OpMode")
//@Disabled
public class testServos extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo servo1;
    private Servo servo2;
    public static final double MID_SERVO   =  0.5 ;

    @Override
    public void runOpMode() {


        servo1  = hardwareMap.get(Servo.class, "2");
        servo1  = hardwareMap.get(Servo.class, "3");
        servo1.setPosition(MID_SERVO);
        servo2.setPosition(MID_SERVO);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double power1 = gamepad1.left_stick_y;
            servo1.setPosition(power1);
        /*
            if (gamepad1.a){
                servo1.setPosition(0);
            }
            else if (gamepad1.b){
                servo1.setPosition(MID_SERVO);
            }
            else{
                servo1.setPosition(gamepad1.left_stick_y);
            }

            if (gamepad1.x){
                servo2.setPosition(0);
            }
            else if (gamepad1.y){
                servo2.setPosition(MID_SERVO);
            }
            else{
                servo2.setPosition(gamepad1.right_stick_y);
            }*/

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("servo1 position", "double", servo1.getPosition());
            telemetry.addData("servo2 position", "double", servo2.getPosition());
            telemetry.update();
        }
    }}
