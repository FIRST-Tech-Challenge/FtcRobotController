package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="old teleop", group="Iterative Opmode")
public class oldteleop  extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    boolean changed = false; //Outside of loop()
    boolean changed2 = false;
    boolean changed3 = false;
    //public Servo turny = null;
    private CRServo spinspinducky = null;
    private CRServo intake = null;
    private DcMotor armboom = null;
    //private  DcMotor intakemotor = null;
    private double PowerFactor = 0.8;
    private Servo platform = null;
    //private Servo tester = null;
    double tgtPower = 0;
    private static final float BUCKETCLEAR = .65f;
    private static final float BUCKETDUMP = 0f;
    private static final float BUCKETIN = 1f;
    boolean spinthatduck = false;
    float dr_position = 0;
    boolean isRumbling = false;
    float dr_lock = .815f;

    //magnet thingy
    private Servo dr_magnet = null;
    private static DistanceSensor MMA = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        intake = hardwareMap.get(CRServo.class, "intake");
        spinspinducky = hardwareMap.get(CRServo.class, "spinspinducky");
        platform  = hardwareMap.get(Servo.class, "platform");
        armboom = hardwareMap.get(DcMotor.class, "armboom");
        dr_magnet = hardwareMap.get(Servo.class, "dr_magnet");
        MMA = hardwareMap.get(DistanceSensor.class, "MMA");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        armboom.setDirection(DcMotor.Direction.FORWARD);
        armboom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //  superPusher.setPosition(.6);
        //tester.setPosition(0.2);  // ring loader arm servo
        platform.setPosition(0);
        dr_magnet.setPosition(0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double LFPower;
    double RFPower;
    double LBPower;
    double RBPower;
    double rainbowPower;
    double rainbowPower2;
    double superShooterPower;
    double superShooterPower2;

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry


//        Out
        if (gamepad1.left_bumper&&!gamepad1.right_bumper) {
            //intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(-1);

        }
        else if(!gamepad1.left_bumper&&!gamepad1.right_bumper)
        {
            intake.setPower(0);
        }
        //In
        else if (gamepad1.right_bumper&&!gamepad1.left_bumper) {
            //intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1);
        }


        if(gamepad1.a && !changed) {
            if(platform.getPosition() == 0) platform.setPosition(.8);
            else platform.setPosition(0);
            changed = true;
        } else if(!gamepad1.a) changed = false;

        if(gamepad1.y && !changed2)
        {
            if(PowerFactor == 0.8f)
            {
                PowerFactor = 0.5f;
            }
            else
            {
                PowerFactor = (0.8f);
            }
            changed2 = true;
        }
        else if(!gamepad1.y)
        {
            changed2 = false;
        }

        if(gamepad2.y && !changed3)
        {
            if(dr_position == .815f)
            {
                dr_lock = 1f;
            }
            else
            {
                dr_lock = (0.815f);
            }
            changed3 = true;
        }
        else if(!gamepad1.y)
        {
            changed3 = false;
        }


        if (gamepad2.dpad_up)
        {
            dr_position += .005f;
            if (dr_position > dr_lock)
            {
                dr_position = dr_lock;
            }
            dr_magnet.setPosition(dr_position);
        }
        if (gamepad2.dpad_down)
        {
            dr_position -=.005f;
            dr_magnet.setPosition(dr_position);
        }

        //boom up
        if (gamepad2.left_trigger >= .1)
        {
            armboom.setPower(gamepad2.left_trigger/3);
        } else if (gamepad2.right_trigger >= .1) {
            armboom.setPower(-gamepad2.right_trigger/3);
        } else {
            armboom.setPower(0);
            armboom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad2.left_bumper){
            spinspinducky.setPower(1);
        }
        else if (gamepad2.right_bumper){
            spinspinducky.setPower(-1);
        }
        else {
            spinspinducky.setPower(0);
        }






        if (MMA.getDistance(DistanceUnit.CM)<3 && !isRumbling)
        {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            isRumbling=true;
        }
        else
        {
            if(MMA.getDistance(DistanceUnit.CM)>4 && isRumbling)
            {
                isRumbling=false;
            }
        }

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * PowerFactor;
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = Math.pow(gamepad1.right_stick_x,5.0);
        //double rightX = (-gamepad1.right_stick_x);
        LBPower = r * Math.cos(robotAngle) - rightX;
        RBPower = r * Math.sin(robotAngle) + rightX;
        LFPower = r * Math.sin(robotAngle) - rightX;
        RFPower = r * Math.cos(robotAngle) + rightX;
        /*
        LBPower = r * Math.cos(robotAngle) + rightX;
        RBPower = r * Math.sin(robotAngle) - rightX;
        LFPower = r * Math.sin(robotAngle) + rightX;
        RFPower = r * Math.cos(robotAngle) - rightX;
*/
        double tgtPower1 = 0;
        tgtPower1 = this.gamepad2.left_stick_y;
        //  rainbow.setPower(tgtPower1);
        //rainbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        tgtPower = -this.gamepad2.right_stick_y;
        //    superShooter.setPower(tgtPower);


        tgtPower = -this.gamepad2.right_stick_y;
        //  superShooter.setPower(Range.clip(tgtPower, 0, .65));


        //telemetry.addData("MotorPower", superShooter.getPower());
        telemetry.addData("Pos",dr_magnet.getPosition());
        telemetry.addData("Status", "Running");
        telemetry.addData("MMA", MMA.getDistance(DistanceUnit.CM));
        telemetry.update();
/*
        if (gamepad2.x) {
            superShooter.setPower(1);
            telemetry.addData("MotorPower", superShooter.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        } else {
            superShooter.setPower(0);
        }
        if (gamepad2.y) {
            superShooter.setPower(0.85);
            telemetry.addData("MotorPower", superShooter.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        else {
            superShooter.setPower(0);

            double tgtPower = 0;*/
           /*
            tgtPower = -this.gamepad2.right_stick_y;
            superShooter.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("MotorPower", superShooter.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
*/

        // Send calculated power to wheels
        LB.setPower(LBPower);
        RB.setPower(RBPower);
        LF.setPower(LFPower);
        RF.setPower(RFPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //  telemetry.addData("positionTarget: ", "%.2f", positionTarget);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}





