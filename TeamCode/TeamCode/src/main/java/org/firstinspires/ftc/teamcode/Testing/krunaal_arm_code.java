package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Arm  servo  d")
//@Disabled
public class krunaal_arm_code extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo;
    private Servo servo1;


    @Override
    public void runOpMode() {
        telemetry.addLine();
        telemetry.update();

        servo  = hardwareMap.get(Servo.class,"servo");
        servo1  = hardwareMap.get(Servo.class,"servo1");



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double servopower;
            double servopower1;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            //servopower    = Range.clip(drive + turn, 0, 1.0) ;
            //servopower1    = Range.clip(drive - turn, 0, 1.0) ;

            if(gamepad1.a){
                servo.setPosition(1);
                //sleep(2000);
            }
            if(gamepad1.b){
                servo1.setPosition(1);
            }
            if(gamepad1.x){
                servo1.setPosition(0.4);
            }
            if(gamepad1.y){
                servo.setPosition(0.4);
                //sleep(2000);
            }

//            telemetry.addData("servo1",servopower);
//            telemetry.addData("servo2",servopower1);
            telemetry.update();

        }
    }
}
