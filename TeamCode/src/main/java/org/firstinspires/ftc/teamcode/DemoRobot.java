package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Demo Robot", group="Iterative Opmode")
public class DemoRobot extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Declare the wheels
    //test
    private DcMotor LB = null; //Located on Control Hub- Motor port 0
    private DcMotor RB = null; //Located on Control Hub- Motor port 1
    private DcMotor ARM = null; // located on Control Hub- Motor port 2
    private double PowerFactor = 0.8f; //Max power available for wheels

    /*
      Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initiali ze the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        ARM = hardwareMap.get(DcMotor.class, "ARM");

        //gripper sensor for pulling arm down
        //touch  = hardwareMap.get(TouchSensor .class, "Touch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        ARM.setDirection(DcMotor.Direction.FORWARD);
   }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        //Code for gamepad1
        //Code for throttling the power factor
        PowerFactor = .8f;

        //Code for mecanum wheels
        double leftY = gamepad1.left_stick_y * PowerFactor;
        double rightY = gamepad1.right_stick_y * PowerFactor;
        double ARMPower = (gamepad1.right_trigger - gamepad1.left_trigger) * PowerFactor;

        // Send calculated power to wheels
        LB.setPower(leftY);
        RB.setPower(rightY);
        ARM.setPower(ARMPower);

        //Send telemetry data of the motor power for wheels
        telemetry.addData("Left Back Motor","Speed: "+ leftY);
        telemetry.addData("Right Back Motor","Speed: "+ rightY);
        telemetry.addData("ARM Motor", "Speed: "+ ARMPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            //Nothing in stop
    }
}