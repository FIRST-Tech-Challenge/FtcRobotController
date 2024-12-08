
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="test servos", group="Linear OpMode")
//@Disabled
public class testServos extends LinearOpMode {
    /* Declare OpMode members. */
    public Servo    servo1    = null;
    public Servo    servo2   = null;

    double clawOffset1 = 0;
    double clawOffset2= 0;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double MID_SERVO   =  0.5 ;


    @Override
    public void runOpMode() {
        double left;
        double right;

        clawOffset1 = MID_SERVO;
        clawOffset2 = MID_SERVO;
        // Define and initialize ALL installed servos.
        servo1  = hardwareMap.get(Servo.class, "5");
        servo2 = hardwareMap.get(Servo.class, "4");
        servo1.setPosition(MID_SERVO);
        servo2.setPosition(MID_SERVO);
        //servo1.setPosition(MID_SERVO);
        //        servo2.setPosition(MID_SERVO);
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            clawOffset1 = clawOffset1 +(-gamepad1.left_stick_y)*0.3;
            clawOffset2 = clawOffset2 +(-gamepad1.right_stick_y)*0.3;
            left = clawOffset1;
            right = clawOffset2;
            servo1.setPosition(left);
            servo2.setPosition(right);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw", "%.01f,", left);
            telemetry.addData("left",  "%.01f", servo1.getPosition());
            telemetry.addData("right", "%.01f", servo2.getPosition());
            telemetry.addData("right, gamepad", "%.01f", gamepad1.right_stick_y);
            telemetry.addData("right, gamepad", "%.01f", gamepad1.left_stick_y);

            // Pace this loop so jaw action is reasonable speed.
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

