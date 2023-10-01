/* go read HardwareDrive in the common folder first!! */

/* tell android studio that we're in this directory */
package org.firstinspires.ftc.teamcode.teleop;

/* import our packages */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotContainer;

/* 
woah :0 - this fancy thing is called a decorator and its only 
purpose is to tell the driver station that you can run this program
in the teleop menu; it also tells it its name
*/
@TeleOp(name = "Base Drive Complete", group = "Drive")
/* 
here, we're extending a class called LinearOpMode - LinearOpMode is 
provided internally by the ftc sdk and is what you use when you want the 
robot to move
*/
public class BaseDriveComplete extends LinearOpMode {

    /* 
    remember HardwareDrive? we're instatiating that class here; 
    everything that we use to control our robot will use this variable 
    */
    private final RobotContainer robot = new RobotContainer();
    private final ElapsedTime runtime = new ElapsedTime();

    /* just the robot's speed */
    double drivePower = 0.40;

    /* another decorater but idk what it does :/ */
    @Override
    /*
    this method is SUPER important!! it comes as a part of LinearOpMode
    so every class that extends LinearOpMode must have this
    method - basically, everything in this function will run when you press
    the "init" button on the driver station with this opmode selected
    */
    public void runOpMode() {

        telemetry.addData("Say", "Hello Driver"); // just saying hi by calling addData on the abstract telemetry member
        telemetry.update();
        runtime.reset(); // reset the little timer on the driver station
        waitForStart(); // basically nothing after this will run until start is pressed on the driver's station
        /*
        very important!! opModeIsActive is an abstract method that returns true if the opmode
        is currently running - so we're just saying run all of this code in an infinite loop
        until we stop running the opmode
        */
        while (opModeIsActive()) {
            UpdateTelemetry(); // update the telemetry
            if (gamepad1.right_bumper) drivePower = 1; // if the right bumper on the first controller is pressed, increase the speed
            DriveRobot(drivePower); // see the below method - we're just telling the robot to move at a certain speed
        }
    }

    /*
    super important method!! this tells the robot how to react to different controller inputs -
    it's pretty simple and just accepts a number on how fast the robot should go
    */
    private void DriveRobot(double drivePower) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1); // the amount of horizontal direction that the left joystick on the first controller is pointing (this is strafing the robot)
        double directionY = Math.pow(gamepad1.left_stick_y, 1); // the amount of vertical direction that the left joystick on the first controller is pointing (this is driving the robot)
        double directionR = -Math.pow(gamepad1.right_stick_x, 1); // the amount of horizontal direction that the right joystick on the first controller is pointing (this is rotating the robot)
        double liftPower = Math.pow(gamepad2.right_stick_y, 1); // the amount of vertical direction that the right joystick on the first controller is pointing (this is moving the lift)

        /* make it so that controller stick drift isn't a problem by eliminating small amounts of input */
        if (gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) directionX = 0;
        if (gamepad1.left_stick_y < 0.2 && gamepad1.left_stick_y > -0.2) directionY = 0;

        /* 
        some math involved here - all that we're doing is multiplying the overall
        direction of the input by the power setting each of the robot's motors to that.
        If the signs aren't what you expect, you're thinking correctly! We didn't set the
        directions of our drive motors correctly in RobotContainer in 2023, so we ended
        up making the coefficients weird.
        */
        robot.lf.setPower((directionY + directionR - directionX) * drivePower);
        robot.rf.setPower((-directionY + directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR + directionX) * drivePower);
        robot.rb.setPower((-directionY + directionR + directionX) * drivePower);

    }


    /* simple stuff, we're just giving the telemetry stack info about the position of our lift and the joysticks */
    private void UpdateTelemetry() {
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);telemetry.addData("g2.L", gamepad2.right_stick_y);
        telemetry.update();
    }

    /* 
    this is a little more advanced telemetry and is only run once
    this only has to do with the imu though so if you wna to learn about that 
    you can
    */

    String formatDegrees(double degrees) { return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees)); }
    String formatAngle(AngleUnit angleUnit, double angle) { return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)); }
}

