package org.firstinspires.ftc.teamcode.TeleOp;


/*
* This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
* This approach is very difficult because the same hardware class can be used by all of our teleop and autonomous
* OpModes without requiring many copy and paste operations. Once We define and test the hardware class with one OpMode,
* It will instantly be available to other OpModes.
*
* The real benefit of this approach is that as we tweak our robot hardware, we'd only need to make changes in ONE place
*  (the Hardware Class).
*
* The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
* In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
* OpMode object when it's created, so it can access all core OpMode functions.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="Concept: Robot Hardware Class", group="Robot")
@Disabled
public class RobotCentric extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    //Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){
        double axial      = 0;
        double lateral    = 0;
        double yaw        = 0;
        double arm        = 0;
        double handOffset = 0;

        //Initialize all hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        //Send telemetry message to signify robot waiting;
        //Wait for the game to start (driver presses PLAY)
        waitForStart();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            // since the joystick goes negative when pushed forward, we negate it
            //In this mode the Left stick moves the robot forward and back, the Right stick turns L and R.
            //This way it's also easy to just drive straight, or just turn.
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw= gamepad1.right_stick_x;

            // combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobotCentric(axial, lateral, yaw);

            /*
            *Use gamepad left and right bumpers to open and close the claw
            * Use the SERVO constants defined in RobotHArdware class.
            * Each time around the loop, the servos will move by a small amount.
            * Limit the total offset to half ot the full travel range
             */
            if(gamepad1.right_bumper){
                handOffset += robot.HAND_SPEED;}
            else if (gamepad1.left_bumper) {
                handOffset -= robot.HAND_SPEED;}
            handOffset = Range.clip(handOffset, -0.5, 0.5);

            //Move both servos to new position. Use RobotHardware class
            //robot.setHandPositions(handOffset);

            //Use gamepad buttons to move arm up (Y) and down (A)
            //Use the MOTOR constants defined in RobotHardware class.
            if(gamepad1.y){
                arm = robot.ARM_UP_POWER;}
            else if (gamepad1.a){
                arm =  robot.ARM_DOWN_POWER;}
            else{
                arm = 0;
            }

           // robot.setArmPower(arm);

            // Send telemetry messages to explain to controls and show robot status
           /* telemetry.addData("axial", "left_stick_y");
            * telemetry.addData("lateral", "left_stick_x");
            * telemetry.addData("yaw", "left_stick_x");
            */
        }
    }
}
