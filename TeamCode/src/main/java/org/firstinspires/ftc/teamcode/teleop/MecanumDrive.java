package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="Mecanum_Drive")
public class MecanumDrive extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private double maxSpeed;
    private GamepadEvents controller1;
    private boolean fieldCentricActive;

    private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        //Init phase
        maxSpeed = 0.5;
        //Initialize other variables
        fieldCentricActive = false;


        //Motor initialization
        frontLeft = hardwareMap.get(DcMotor.class,"FLM");
        backLeft = hardwareMap.get(DcMotor.class,"BLM");
        frontRight = hardwareMap.get(DcMotor.class,"FRM");
        backRight = hardwareMap.get(DcMotor.class,"BRM");

        //Reverse some motors
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection((DcMotorSimple.Direction.REVERSE));

        //Initialize gamepad object
        controller1 = new GamepadEvents(gamepad1);



        //Initialize the imu object
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addLine("Wait for Start");
        telemetry.update();
        waitForStart();
        //Start phase
        //This is the event loop
        while (!isStopRequested()) {

            //Input checks
            double forward = controller1.left_stick_y;
            double strafe = -controller1.left_stick_x;
            double rotate = -controller1.right_stick_x;

            if (controller1.a.onPress()){
                //Toggle Field Centric Drive
                fieldCentricActive = !fieldCentricActive;
            }

            if (controller1.b.onPress()){
                //Reset heading angle (Through IMU or encoders)
                imu.resetYaw();
            }


            if(fieldCentricActive){
                //Manipulate drive/strafe values
                double currentRotation = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double temp = forward * Math.cos(currentRotation) + strafe * Math.sin(currentRotation);
                strafe = -forward * Math.sin(currentRotation) + strafe * Math.cos(currentRotation);
                forward = temp;
            }

            //Activate Motors;
            frontLeft.setPower((forward + strafe + rotate)*maxSpeed);
            backLeft.setPower((forward - strafe + rotate)*maxSpeed);
            frontRight.setPower((forward - strafe - rotate)*maxSpeed);
            backRight.setPower((forward + strafe - rotate)*maxSpeed);

            //Display Telemetry information
            displayTelemetry();
            //Update controller information
            //Failing to add this into the event loop will mean
            //that all user inputs will not be read by the program
            controller1.update();
        }
        //End of program
    }

    //Displays Telemetry Data
    private void displayTelemetry(){
        telemetry.addLine("Use A to toggle field centric drive!");
        telemetry.addData("Field Centric Active: ", fieldCentricActive);
        if (fieldCentricActive){
            telemetry.addData("Bot rotation: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
        //Always update telemetry, otherwise it will not show up
        telemetry.update();
    }

}
