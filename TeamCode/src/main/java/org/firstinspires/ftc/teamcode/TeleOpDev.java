package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bosons.Hardware.Controller;
import com.bosons.Hardware.DriveTrain;
import com.bosons.Hardware.arm;
/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Dev")

public class TeleOpDev extends OpMode{
    // Declare HardWare.
    public Controller Driver = null;
    public DriveTrain driveTrain = null;
    //public arm Arm = null;

    double leftX  = 0.0;
    double leftY  = 0.0;
    double rightY = 0.0;

    /*
     * Code to run ONCE when the Driver hits INIT
     */
    @Override
    public void init () {
        //Arm = new arm(this,0.5);
        driveTrain = new DriveTrain(this,1.0);
        Driver = new Controller(gamepad1);

        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the Driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop () {

    }

    /*
     * Code to run ONCE when the Driver hits START
     */
    @Override
    public void start () {
        //driveTrain.DebugWheels(12.0,0.5);
    }

    /*
     * Code to run REPEATEDLY after the Driver hits START but before they hit STOP
     */
    @Override
    public void loop () {


        //if (Driver.onButtonPress(Controller.Button.dPadUp)){
        //    Arm.RunToTarget(2190);
        //}
        //if (Driver.onButtonPress(Controller.Button.dPadDown)) {
        //    Arm.RunToTarget(0);
        //}
        if(Driver.toggleButtonState(Controller.Button.y)){driveTrain.MotorPower = 0.1;}
        else{driveTrain.MotorPower = 1.0;}

        leftX = Driver.getAnalogValue(Controller.Joystick.LeftX);
        leftY = Driver.getAnalogValue(Controller.Joystick.LeftY);
        rightY = Driver.getAnalogValue(Controller.Joystick.RightY);

        driveTrain.KinematicMove(leftX,leftY,rightY);

        //telemetry.addData("Current Pip           | ",Arm.Pips);
        //telemetry.addData("RightArmMotor Power   | ",Arm.RightArmMotor.getPower());
        //telemetry.addData("RightArmMotor encoder | ",Arm.RightArmMotor.getCurrentPosition());
        //telemetry.addLine();
        //telemetry.addData("LeftArmMotor Power    | ",Arm.LeftArmMotor.getPower());
        //telemetry.addData("LeftArmMotor encoder  | ",Arm.LeftArmMotor.getCurrentPosition());

        //YOU NEED THESE FOR CONTROLLER AND SAFTEY CHECKS
        Driver.updateAll();
        //Arm.MotorCheck();
    }

    /*
     *   Code to run ONCE after the Driver hits STOP
     */
    @Override
    public void stop () {

    }

}
