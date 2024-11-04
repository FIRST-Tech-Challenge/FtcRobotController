package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bosons.Hardware.Controller;
import com.bosons.Hardware.DriveTrain;
import com.bosons.Hardware.Arm;
import com.bosons.Hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Config
@TeleOp(name="TeleOp", group="Dev")

public class TeleOpDev extends OpMode{
    // Declare HardWare.
    public Controller driverA = null;
    public DriveTrain driveTrain = null;
    public Arm arm = null;

    public static double radius_arm = 0;
    public static double theta_arm = -28;

    /*
     * Code to run ONCE when the Driver hits INIT
     */
    @Override
    public void init () {
        arm = new Arm(this,0.8);
        driveTrain = new DriveTrain(this);
        driverA = new Controller(gamepad1);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the Driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop () {
        //arm.setPositionPolar(radius_arm,theta_arm);
    }

    /*
     * Code to run ONCE when the Driver hits START
     */
    @Override
    public void start () {
        arm.setPositionPolar(radius_arm,theta_arm);
    }

    /*
     * Code to run REPEATEDLY after the Driver hits START but before they hit STOP
     */
    @Override
    public void loop () {


        if (driverA.onButtonPress(Controller.Button.dPadUp)){
            arm.extendToTarget(2190,0.5);
        }
        if (driverA.onButtonPress(Controller.Button.dPadDown)) {
            arm.extendToTarget(0,0.5);
        }
        if(driverA.toggleButtonState(Controller.Button.y)){
            driveTrain.setDrivePowerCoefficient(0.1);
            driveTrain.setTurnPowerCoefficient(0.1);
        }
        else{
            driveTrain.setDrivePowerCoefficient(1);
            driveTrain.setTurnPowerCoefficient(1);
        }

        //setting up controller input to drivetrain output ratios//
        double x = -driverA.getAnalogValue(Controller.Joystick.LeftX) * 1.5;
        double y = driverA.getAnalogValue(Controller.Joystick.LeftY) * 1.5;
        double turn = -driverA.getAnalogValue(Controller.Joystick.RightX)/1.2;

        double p = Math.sqrt((x * x) + (y * y));
        double theta = Math.atan2(y, x);
        driveTrain.drive(p , theta, turn); //updates the drivetrain inputs in the "DriveTrain" class
        //------------------------------------------------------//

        //arm.updatePidLoop();
        //arm.setPositionPolar(radius_arm,theta_arm);
        arm.updatePosition();



        //YOU NEED THESE FOR CONTROLLER AND SAFTEY CHECKS
        driverA.updateAll();
    }

    /*
     *   Code to run ONCE after the Driver hits STOP
     */
    @Override
    public void stop () {

    }

}
