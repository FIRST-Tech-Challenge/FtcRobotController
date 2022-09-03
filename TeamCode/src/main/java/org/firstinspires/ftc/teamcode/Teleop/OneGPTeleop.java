package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;

/**
 * Teleop w/ drivetrain, shooter(still in testing), and wobblegoal
 *
 * Current button mappings for gamepad 1 = right joystick for movement, left for turning,
 * x for slowmode on, a for slowmode off
 *
 * Current button mappings for gamepad 2 = y for shoot raiseOuttakeHigh goal, b for raiseOuttakeMid goal, a for raiseOuttakeLow,
 * x for moving servo back and forth, right trigger for turning on shooter motor, dpad up and down
 * for wobblegoal
 *
 * @author  Nathan
 * @version 2.0
 * @since   2020-Jan-13
 *
 */


@TeleOp(name = "OneGPTeleop ")
@Disabled
public class OneGPTeleop extends LinearOpMode {

    // new version of runopmode that supports inplaceturn slowmode and can toggle slowmode on and off with one button - tested by aiden jonathan ma
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        BlackoutRobot robot = new BlackoutRobot(BasicChassis.ChassisType.ENCODER, false ,false,90);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();
        //robot.navigateTeleOp();
        double magnitude;
        double angleInRadian;
        double angleInDegree;
        boolean slowMode = false;
        double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        int    CYCLE_MS    =   30;     // period of each cycle

        // Define class members
        Servo servo;

        double MAX_POS     =  1.0;     // Maximum rotational position
        double MIN_POS     =  0.0;     // Minimum rotational position

        double  power   = 0;
        boolean rampUp  = true;
        double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


        while (!isStopRequested()) {
            //ignore cuz we using two gamepad
            float left_trigger_1 = gamepad1.left_trigger;
            float right_trigger_1 = gamepad1.right_trigger;
            float right_trigger_2 = gamepad2.right_trigger;
            float left_stick_y_2 = -gamepad2.left_stick_y;
            float left_stick_x_2 = -gamepad2.left_stick_x;
            float right_stick_x_2 = -gamepad2.right_stick_x;
            boolean x_2 = gamepad2.x;
            boolean y_2 = gamepad2.y;


            if(!BlackoutRobot.isCorgi){
                angleInRadian = Math.atan2(left_stick_y_2*-1, left_stick_x_2*2);
            }
            else{
                angleInRadian = Math.atan2(left_stick_y_2, left_stick_x_2*-2);
            }
            angleInDegree = Math.toDegrees(angleInRadian);

            magnitude = Math.sqrt(Math.pow(left_stick_x_2, 2) + Math.sqrt(Math.pow(left_stick_y_2, 2)));
            robot.moveMultidirectional(left_stick_y_2, 0.95, (float)(right_stick_x_2*0.6), slowMode); // It is 0.95, because the robot DCs at full power.
        }
        idle();
    }
}