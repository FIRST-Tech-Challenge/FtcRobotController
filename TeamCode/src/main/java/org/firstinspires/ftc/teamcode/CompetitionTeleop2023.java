package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Competition Teleop", group="Iterative Opmode")
public class CompetitionTeleop2023 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Declare the wheels
    //test
    private DcMotor LF = null; //Located on Control Hub- Motor port 0
    private DcMotor RF = null; //Located on Control Hub- Motor port 2
    private DcMotor LB = null; //Located on Control Hub- Motor port 1
    private DcMotor RB = null; //Located on Control Hub- Motor port 3
    //Declare variables used for our arm lift
    private DcMotor arm = null; //Located on Expansion Hub- Motor port 0
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    //variable for Rev Touch Sensor
    private TouchSensor touch;

    private double PowerFactor = 0.8f; //Max power available for wheels
    private int maxEncode = 4200; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private int minEncode = 110;//Minimum so string on arm lift doesn't break and position 0
    private int pos1 = 1850; //Position 1
    private int pos2 = 3000; //Position 2
    private int desiredpos = 0; //Used as base for increasing arm position
    private double armPower = .7f;

    boolean changed = false; //Used for the gripper button code
    boolean changed2 = false; //Used for the code that allows the driver to alter speed
    boolean changed3 = false; //Used to toggle betweeen auto and manual mode for arm
    boolean gamebpush = false; //To go through intervals one at a time

    boolean touchIsPressed = false;

    /*
      Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initiali ze the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //gripper sensor for pulling arm down
        touch  = hardwareMap.get(TouchSensor .class, "Touch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);
        //Set arm up to use encoders
        arm.setPower(0);
        //Set arm up to brake
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        //Code for gamepad1
        //Code for throttling the power factor
        PowerFactor = (1 - gamepad1.right_trigger) *.8f;

        //Code for mecanum wheels
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * PowerFactor;
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = Math.pow(gamepad1.right_stick_x, 5.0)*(1-gamepad1.right_trigger);
        //double rightX = (-gamepad1.right_stick_x);
        LBPower = r * Math.cos(robotAngle) - rightX;
        RBPower = r * Math.sin(robotAngle) + rightX;
        LFPower = r * Math.sin(robotAngle) - rightX;
        RFPower = r * Math.cos(robotAngle) + rightX;

        // Send calculated power to wheels
        LB.setPower(LBPower);
        RB.setPower(RBPower);
        LF.setPower(LFPower);
        RF.setPower(RFPower);

        //Send telemetry data of the motor power for wheels
        telemetry.addData("Left Front Motor","Speed: "+ LFPower);
        telemetry.addData("Left Back Motor","Speed: "+ LBPower);
        telemetry.addData("Right Front Motor","Speed: "+RFPower);
        telemetry.addData("Right Back Motor","Speed: "+ RBPower);
        telemetry.addData("Arm Encoder Height","Height: "+arm.getCurrentPosition());

        //Code for gamepad2
        //Toggle auto and manual mode
        if (gamepad2.y) {
            changed3 = !changed3;
        }

        if (gamepad2.left_trigger >= .1)
        {
            touchIsPressed = false;
        }
        else if (!touch.isPressed())
        {
            touchIsPressed = true;
        }

        //Moves the arm up
        if (!changed3)
        {
            if (gamepad2.left_trigger >= .1 && arm.getCurrentPosition() < maxEncode)
            {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(gamepad2.left_trigger);
                telemetry.addData("arm ticks", arm.getCurrentPosition());
                telemetry.update();
                //Moves the arm down
            }
            else if (gamepad2.right_trigger >= .1 &&/* arm.getCurrentPosition() > minEncode &&*/ !touchIsPressed)
            {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-gamepad2.right_trigger);
            }
            else
            {
                arm.setPower(0);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else
        {
            //Allows the arm to stay in the location the drivers want
            if (!gamepad2.b && gamebpush) {
                desiredpos += 1;
                desiredpos &= 3;
                if (desiredpos == 1) {
                    arm.setTargetPosition(pos1);
                    //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(armPower);
                } else if (desiredpos == 2) {
                    arm.setTargetPosition(pos2);
                    //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(armPower);
                } else if (desiredpos == 3) {
                    arm.setTargetPosition(maxEncode);
                    //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(armPower);
                }
            }
            //Code to decrease height position
            else if (gamepad2.x) {
                desiredpos = 0;
                arm.setTargetPosition(minEncode);
                //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
            }
        }
        if (gamepad2.b)
        {
            gamebpush = true;
        }
        else{

            gamebpush = false;
        }
        //Code to increase height position

        //Allows the drivers to use a single button to open and close gripper
        if (gamepad2.a && !changed) {
            if (gripper.getPosition() == 0.2)
            {
                gripper.setPosition(.6);
                arm.setTargetPosition(150);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
            }
            else
                gripper.setPosition(0.2);
            changed = true;
        } else if (!gamepad2.a)
        {
            changed = false;
        }
    // Show the elapsed game time and wheel power.
            telemetry.addData("Status","Run Time: "+runtime.toString());
    //  telemetry.addData("positionTarget: ", "%.2f", positionTarget);
}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            //Nothing in stop
    }
}





