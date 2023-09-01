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
        composeTelemetry(); // prepare the robot's telemetry (just data that we want to get from it)
        robot.init(hardwareMap); // remember?? call the init method on robot and pass the abstract hardwareMap variable (abstract meaning it inherently comes with the class LinearOpMode)
        telemetry.addData("Say", "Hello Driver"); // just saying hi by calling addData on the abstract telemetry member
        runtime.reset(); // reset the little timer on the driver station
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // super important!! reset our lift encoder so the lift doesn't break itself
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // now just tell the lift to use its encoder
        waitForStart(); // basically nothing after this will run until start is pressed on the driver's station

        /* tell the imu to remember itself at the current position */
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        /* 
        very important!! opModeIsActive is an abstract method that returns true if the opmode
        is currently running - so we're just saying run all of this code in an infinite loop
        until we stop running the opmode
        */
        while (opModeIsActive()) {
            UpdateGripper(); // update our gripper
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

        int liftPos = robot.lift.getCurrentPosition(); // get the lift's current position based on the encoder

        /* 
        some fancy checking to make sure that the lift doesn't explode -
        if the elevator is already fully extended, make it so that it rises or lowers
        at an extremely slow rate
        */
        if (liftPos < Constants.elevatorPositionTop && gamepad2.right_stick_y < 0) robot.lift.setPower((gamepad2.left_stick_y) * 0.1 - -0.001);
        else if (liftPos > Constants.elevatorPositionBottom && gamepad2.right_stick_y > 0) robot.lift.setPower((gamepad2.left_stick_y) * 0.01);
        else robot.lift.setPower((liftPower - 0.001) * 1);
    }

    /* A simple method to update the gripper when the left or right triggers are pressed*/
    private void UpdateGripper() {
        if (gamepad2.left_trigger > 0.01) robot.serv0.setPower(0.22 * gamepad2.left_trigger - 0);
        else if (gamepad2.right_trigger > 0.01) robot.serv0.setPower(-0.16 * gamepad2.right_trigger + 0);
    }

    /* simple stuff, we're just giving the telemetry stack info about the position of our lift and the joysticks */
    private void UpdateTelemetry() {
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", robot.lift.getCurrentPosition());
        telemetry.addData("g2.L", gamepad2.right_stick_y);
        telemetry.update();
    }

    /* 
    this is a little more advanced telemetry and is only run once
    this only has to do with the imu though so if you wna to learn about that 
    you can
    */
    public void composeTelemetry() {
        telemetry.addAction(() -> {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.gravity = robot.imu.getGravity();
        });

        telemetry.addLine().addData("status", () -> robot.imu.getSystemStatus().toShortString()).addData("calib", () -> robot.imu.getCalibrationStatus().toString());
        telemetry.addLine().addData("heading", () -> formatAngle(robot.angles.angleUnit, robot.angles.firstAngle)).addData("roll", () -> formatAngle(robot.angles.angleUnit, robot.angles.secondAngle)).addData("pitch", () -> formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle));
        telemetry.addLine().addData("grvty", () -> robot.gravity.toString()).addData("mag", () -> String.format(Locale.getDefault(), "%.3f", Math.sqrt(robot.gravity.xAccel * robot.gravity.xAccel + robot.gravity.yAccel * robot.gravity.yAccel + robot.gravity.zAccel * robot.gravity.zAccel)));
    }
    String formatDegrees(double degrees) { return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees)); }
    String formatAngle(AngleUnit angleUnit, double angle) { return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)); }
}

