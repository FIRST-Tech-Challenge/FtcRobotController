package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.MecanumHardware.Servo_Close;
import static org.firstinspires.ftc.teamcode.MecanumHardware.Servo_Open;

@TeleOp(name="Launchor Test", group="Linear Opmode")
public class Launchor_motorTest extends LinearOpMode {

    // Declare OpMode members.
    /* Declare OpMode members */
    MecanumHardware robot = new MecanumHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Setup a variable for each drive wheel to save power level for telemetry
        double launchorPower = 1;
        boolean launchorOn = false;
        boolean newPress = true;

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
         //Turn on or off the launchor motor when y is pressed.
         if (gamepad1.y && !launchorOn && newPress) {
             //if (launchor.getPower() == 0) {
                 //runtime.reset();
             newPress = false;
             robot.launcher1.setPower(launchorPower);
             robot.launcher2.setPower(launchorPower);
                 launchorOn = true;
                    /*while (opModeIsActive() && !gamepad1.y) {
                        telemetry.addData("Launch", "Runtime: %2.5f S Elapsed", runtime.seconds());
                        telemetry.addData("Power", "Power: %2.5f", launchorPower);
                        telemetry.update();
                    }*/
                 telemetry.addData("Launch On", launchorOn);
                 telemetry.addData("Launch Power", robot.launcher1.getPower());
                 telemetry.addData("Launch Power", robot.launcher2.getPower());
                 telemetry.update();
             //}
         }

             if (gamepad1.y && launchorOn && newPress) {
                 robot.launcher1.setPower(0);
                 robot.launcher2.setPower(0);
                 newPress = false;
                 launchorOn = false;
                 telemetry.addData("Launch Off", launchorOn);
                 telemetry.addData("Launch Power", robot.launcher1.getPower());
                 telemetry.addData("Launch Power", robot.launcher2.getPower());
                 telemetry.update();
             }

             if(!gamepad1.y){
                 newPress = true;
             }

                /* just some redundant code written while playing around
             if (gamepad1.a) {
                robot.launcher1.setPower(0);
                robot.launcher2.setPower(0);
                launchorOn = false;
                telemetry.addData("Launch Off", launchorOn);
                telemetry.addData("Launch Power", robot.launcher1.getPower());
                telemetry.addData("Launch Power", robot.launcher2.getPower());
                telemetry.update();
            }
            */
            // When the dpad up is pressed increase the power by 5 seconds.
            if (gamepad1.dpad_up) {
                if (launchorPower < 1) {
                    launchorPower = launchorPower + 0.05;
                    if (launchorOn){
                        robot.launcher1.setPower(launchorPower);
                        robot.launcher2.setPower(launchorPower);
                        telemetry.addData("Launch On", launchorOn);
                        telemetry.addData("Launch Power", robot.launcher1.getPower());
                        telemetry.addData("Launch Power", robot.launcher2.getPower());
                        telemetry.update();}
                }
            }

            // When the dpad down is pressed decrease the power by 5 seconds.
            if (gamepad1.dpad_down) {
                if (launchorPower > 0.05) {
                    launchorPower = launchorPower - 0.05;
                    if (launchorOn){
                        robot.launcher1.setPower(launchorPower);
                        robot.launcher2.setPower(launchorPower);
                        telemetry.addData("Launch On", launchorOn);
                        telemetry.addData("Launch Power", robot.launcher1.getPower());
                        telemetry.addData("Launch Power", robot.launcher2.getPower());
                        telemetry.update();}
                }
            }

            if (gamepad1.right_bumper) {
                robot.triggerServo.setPosition(Servo_Open);
                runtime.reset();
            }

            if(robot.triggerServo.getPosition() == Servo_Open  && runtime.seconds() > 1.0){
                robot.triggerServo.setPosition(Servo_Close);
            }

         // Pause for 10 mS each cycle = update 100 times a second.
         sleep(100);
        }
    }
}



