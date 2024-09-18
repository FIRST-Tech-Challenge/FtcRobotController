package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SparkOdo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="Mecanum_Drive")
public class MecanumDrive extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    private Lift lift;
    private double maxSpeed;
    private GamepadEvents controller1;
    private boolean fieldCentricActive;

    private IMU imu;

    private SparkOdo sparkOdo;

    private double CM_2_INCHES = 0.39370079;
    private double MM_2_INCHES = CM_2_INCHES * 0.1;
    private int TICKS_PER_ROT = 4096;
    private double WHEEL_DIAMETER = 35 * MM_2_INCHES; //35 mm
    private double INCHES_PER_ROTATION = WHEEL_DIAMETER * Math.PI; //Distance in inches per rotation
    private int TICKS_PER_INCH = (int)(TICKS_PER_ROT / INCHES_PER_ROTATION); //# of Ticks per Inch of distance
    @Override
    public void runOpMode() throws InterruptedException {
        //Init phase
        maxSpeed = 0.5;
        //Initialize other variables
        fieldCentricActive = false;



        sparkOdo = new SparkOdo(hardwareMap, telemetry);

        //Motor initialization
        frontLeft = hardwareMap.get(DcMotor.class,"FLM");
        backLeft = hardwareMap.get(DcMotor.class,"BLM");
        frontRight = hardwareMap.get(DcMotor.class,"FRM");
        backRight = hardwareMap.get(DcMotor.class,"BRM");

        lift = new Lift(hardwareMap);

        //Motor/Deadwheel Encoders Initialization
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            int target = lift.moveLift(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());
            telemetry.addData("TargetPos: ",target);
            telemetry.addData("CurrentPos:", lift.currentPos());

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

            if (controller1.y.onPress()){
                sparkOdo.resetOdo();
            }

            if (controller1.x.onPress()){
                sparkOdo.calibrateOdo();
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

        displaySparkData();

        SparkFunOTOS.Pose2D pos = sparkOdo.getPos();
        telemetry.addData("Rotational Difference:", Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - pos.h));
        displayDeadwheelData();
        //Always update telemetry, otherwise it will not show up
        telemetry.update();
    }

    public void displayDeadwheelData(){
        //FLM = Left Encoder
        //BLM = Center Encoder
        //FRM = Right Encoder
        telemetry.addData("Forward Distance: ", 1.0 * frontLeft.getCurrentPosition() / TICKS_PER_INCH);
        telemetry.addData("Strafe Distance", 1.0 * backLeft.getCurrentPosition() / TICKS_PER_INCH);
    }

    public void displaySparkData(){
        SparkFunOTOS.Pose2D pos = sparkOdo.getPos();

        telemetry.addLine();
        // Inform user of available controls
        telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
        telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
        telemetry.addLine();

        // Log the position to the telemetry
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);

        SparkFunOTOS.Pose2D total = sparkOdo.updateTotalDist();
        telemetry.addLine();
        telemetry.addData("X Total", total.x);
        telemetry.addData("Y Total", total.y);
        telemetry.addData("Heading angle Total", total.h);

    }

}
