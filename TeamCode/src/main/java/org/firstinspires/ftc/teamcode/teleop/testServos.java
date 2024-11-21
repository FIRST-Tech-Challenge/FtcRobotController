
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

    double clawOffset = 0;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double MID_SERVO   =  0.5 ;


    @Override
    public void runOpMode() {
        double left;
        double right;
        double turn;

        // Define and initialize ALL installed servos.
        servo1  = hardwareMap.get(Servo.class, "2");
        servo2 = hardwareMap.get(Servo.class, "3");
        servo1.setPosition(MID_SERVO);
        servo2.setPosition(MID_SERVO);
        //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            left = -gamepad1.left_stick_y + 0.5;
            right = gamepad1.right_stick_y + 0.5;
            servo1.setPosition(left);
            servo2.setPosition(right);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", servo1.getPosition());
            telemetry.addData("right", "%.2f", servo2.getPosition());


            // Pace this loop so jaw action is reasonable speed.
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }
    }
}

