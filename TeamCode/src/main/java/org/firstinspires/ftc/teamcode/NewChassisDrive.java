package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="NewChassisDrive", group="Linear Opmode")
public class NewChassisDrive extends LinearOpMode {


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

    @Override
    public void runOpMode() {


        //Initialize hardware
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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




            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value

            double horizontal =  gamepad1.left_stick_x;
            double vertical     =  gamepad1.left_stick_y * -1;
            double turn = gamepad1.right_stick_x;

            double leftFrontPower  = (vertical + horizontal + turn ) * speedMulti;
            double rightFrontPower = (vertical - horizontal - turn ) * speedMulti;
            double leftBackPower   = (vertical - horizontal + turn ) * speedMulti;
            double rightBackPower  = (vertical + horizontal - turn ) * speedMulti;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            telemetry.addData("Horizontal input", gamepad1.left_stick_x * -1);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y * -1);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.addData("looptime", looptime);

            telemetry.update();
        }
    }}


