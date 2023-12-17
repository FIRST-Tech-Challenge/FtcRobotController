package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="FieldOrientation", group="Linear Opmode")
@Disabled
public class FieldOrientation extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    private float speedMulti = 1f;
    private boolean canChangeSpeeds = true;
    private float looptime = 0f;

    public double pi = Math.PI;

    public double piOverTwo = Math.PI/2;

    private double AngleA = pi;
    //private double DeadzoneA = 0.12; original value of dead zone
    private double DeadzoneA = .06;

    private IMU imu;

    @Override
    public void runOpMode() {


        //Initialize hardware
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        imu = hardwareMap.get(IMU.class, "imu");


        //Set motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();
        runtime.reset();

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            looptime = looptime + 1;

            if(gamepad1.y && canChangeSpeeds){
                canChangeSpeeds = false;
                if (speedMulti == 0.4) {
                    speedMulti = 1;
                } else if (speedMulti == 1) {
                    speedMulti = 0.4f;
                }
            } else if (!gamepad1.y) {
                canChangeSpeeds = true;
            }

            telemetry.addData("motor PowerLB:", leftBackDrive.getPower());
            telemetry.addData("motor PowerLF:", leftFrontDrive.getPower());
            telemetry.addData("motor PowerRF:", rightFrontDrive.getPower());
            telemetry.addData("motor PowerRB:", rightBackDrive.getPower());



            //Angle returns -pi to pi, we need 0 to 2pi, so add pi to result
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double theta = orientation.getYaw(AngleUnit.RADIANS) + Math.PI;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value

            double horizontal =  gamepad1.left_stick_x;
            double vertical     =  gamepad1.left_stick_y * -1;
            double turnR = 0;
            double turnL = 0;


            /*if (gamepad1.b && theta % (Math.PI/2) > 0.01) {
                turn = (theta % (Math.PI/2)) * 2;
            } else if (!gamepad1.b) {
                turn = gamepad1.right_stick_x;
            }*/

            if (gamepad1.dpad_up && theta > AngleA + DeadzoneA) {
                turnR = (((theta - AngleA) - DeadzoneA) / -piOverTwo);
                //needs to end in positive value
                //turnR = 1;
            }else if (gamepad1.dpad_up && theta < AngleA - DeadzoneA) {
                //CURRENT VERSION of the auto lock code
                turnL = (((theta - AngleA) - DeadzoneA) / piOverTwo);
                //needs to end in negative value
                //turn = .65;
            } else if (!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.dpad_left) {
                turnR = 0;
                turnL = gamepad1.right_stick_x;
            }

            if (gamepad1.a) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press A (triangle) on Gamepad to reset\n");
            }


            double horizontalOut = (horizontal * Math.cos(theta)) - (vertical * Math.sin(theta));
            double verticalOut = (vertical * Math.cos(theta)) + (horizontal * Math.sin(theta));


            double leftFrontPower  = (verticalOut - horizontalOut + turnR - turnL) * speedMulti;
            double rightFrontPower = (verticalOut + horizontalOut - turnR + turnL) * speedMulti;
            double leftBackPower   = (verticalOut + horizontalOut + turnR - turnL) * speedMulti;
            double rightBackPower  = (verticalOut - horizontalOut - turnR + turnL) * speedMulti;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            telemetry.addData("Heading", "%.2f Radians", theta);

            telemetry.addData("Horizontal input", gamepad1.left_stick_x * -1);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y * -1);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.addData("Hertical out: ", verticalOut);
            telemetry.addData("Vorizontal out: ", horizontalOut);
            telemetry.addData("looptime", looptime);

            telemetry.update();
        }
    }}


