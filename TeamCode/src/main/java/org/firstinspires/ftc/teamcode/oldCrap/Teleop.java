package org.firstinspires.ftc.teamcode.oldCrap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Random;

//@TeleOp(name = "Into The Deep Teleop")
public class Teleop extends LinearOpMode {
    // Motors for the wheels
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;


    // Motors for the arm joints
    private DcMotor jointOne = null;
    private DcMotor jointTwo = null;

    // Servos for the claw and wrist
    private Servo claw = null;
    private Servo horizontalWrist = null;
    private Servo verticalWrist = null;

    // Sensors
    private ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;

    private final String[] puns = {"The robot comedian’s act was electric—it really generated a charge of laughter.", "Why did the robot break up with his GPS? It couldn't find a path to love.", "Did you hear about the robot that drank oil instead of water? It had a well-oiled system.", "The robot chef was wired for success—it always knew the best byte size for recipes.", "When the cleaning robot went on strike, it really swept the nation.", "It’s frowned upon to call a robot lazy; they prefer the term ‘power-saving mode.’", "The robot dance crew had some serious circuit moves.", "The robot politician always had a binary solution to every problem.", "That robot jazz band really knows how to improvise on the fly.", "When the robot detective retired, they dismantled the case files.", "The robot professor was an expert in artificial intelligence—it had a lot of bytes of knowledge.", "The robot penguin struggled to stay afloat in the digital world.", "The robot athlete had a drive to win in every circuit.", "Why did the robot open an ice cream shop? It wanted to serve up some byte-sized treats.", "The robot construction worker always nailed the job—it had a real ‘transformer’ in skills.", "The robot artist painted with such precision, it was truly an embodiment of pixel perfection.", "The robot gardener’s favorite task was to ‘reboot’ the garden beds.", "That robot banker can really save up ‘data‘ on making smart investments.", "The robot baker’s specialty was making ‘chip’ cookies that were out of this world.", "The robot ghosts managed to scare the circuits out of the unsuspecting humans.", "Why did the robot break up with his GPS? It couldn't find a path to love.", "The robot comedian’s act was electric—it really generated a charge of laughter.", "Did you hear about the robot that drank oil instead of water? It had a well-oiled system.", "The robot chef was wired for success—it always knew the best byte size for recipes.", "When the cleaning robot went on strike, it really swept the nation.", "It’s frowned upon to call a robot lazy; they prefer the term ‘power-saving mode.’", "The robot dance crew had some serious circuit moves.", "The robot politician always had a binary solution to every problem.", "That robot jazz band really knows how to improvise on the fly.", "When the robot detective retired, they dismantled the case files.", "The robot professor was an expert in artificial intelligence—it had a lot of bytes of knowledge.", "The robot penguin struggled to stay afloat in the digital world.", "The robot athlete had a drive to win in every circuit.", "Why did the robot open an ice cream shop? It wanted to serve up some byte-sized treats.", "The robot construction worker always nailed the job—it had a real ‘transformer’ in skills.", "The robot artist painted with such precision, it was truly an embodiment of pixel perfection.", "The robot gardener’s favorite task was to ‘reboot’ the garden beds.", "That robot banker can really save up ‘data‘ on making smart investments.", "The robot baker’s specialty was making ‘chip’ cookies that were out of this world.", "The robot ghosts managed to scare the circuits out of the unsuspecting humans.", "When the robot couldn't find a date, it said, “Looks like I’m stuck in a love circuit.”", "The robot chef was known for his electrifying grill skills, he was truly a “current” chef.", "The robot comedian’s performance was riveting, it really had the audience “wired” up.", "After the robot dance party, everyone agreed it was a “bot” of fun.", "The robot that loved to garden was always planting “byte”-size seeds.", "When the robot got a virus, it was a bit “malware”-functioning.", "The robot detective was on a “wire-tap” to solve the mystery.", "The robot magician’s disappearing act was truly “unbe-“bot”-able.", "The robot musician was a “transistor” in the making of music.", "The robot librarian organized its data with “byte”-sized information.", "The robot athlete was “circuit”-ing the track for the upcoming race.", "When the robot went to the beach, it made sure to wear its “transistor” hat.", "The robot banker saved all its money in a “circuit” account.", "The robot painter’s artwork was truly “mechanical.”", "The robot chef’s secret ingredient was a “transistor”-y flavor.", "The robot astronaut’s mission was to explore the “universe” of possibilities.", "The robot tailor was known for its precise “stitching” skills.", "The robot architect designed a “transformer”-ational building.", "The robot pirate sailed the “circuit”-ous seas in search of treasure.", "The robot poet’s words were truly “electric.”"};
    private final Random random = new Random();

    private final String randomPun = puns[random.nextInt(puns.length)];

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        jointOne = hardwareMap.get(DcMotor.class, "jointOne");
        jointTwo = hardwareMap.get(DcMotor.class, "jointTwo");

        claw = hardwareMap.get(Servo.class, "claw");
        horizontalWrist = hardwareMap.get(Servo.class, "horizontalWrist");
        verticalWrist = hardwareMap.get(Servo.class, "verticalWrist");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        jointOne.setDirection(DcMotor.Direction.REVERSE);
        jointTwo.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior for arm motors
        jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            // Update telemetry data
            telemetry.addData("Pun of the Day: ", randomPun);
            telemetry.addData("First Joint Position", jointOne.getCurrentPosition());
            telemetry.addData("Second Joint Position", jointTwo.getCurrentPosition());
            telemetry.update();

            // Control robot movement
            double vertical = 0.4 * gamepad1.left_stick_y;
            double strafe = -0.4 * gamepad1.left_stick_x;
            double turn = -0.35 * gamepad1.right_stick_x;
            robotMovement(vertical, strafe, turn);

            // Emergency brake
            if (gamepad2.left_stick_button || gamepad2.right_stick_button) {
                brake();
            }

            // Arm control with deadband
            double manualArmDeadband = 0.3;
            double jointOnePower = 0.6;
            double jointTwoPower = 0.6;
            newArm(manualArmDeadband, jointOnePower, jointTwoPower);

            // Wrist and claw control (placeholders)
            int Hpos1 = 100;
            int Hpos2 = 0;
            int Vpos1 = 100;
            int Vpos2 = 0;
            wrist(Hpos1, Hpos2, Vpos1, Vpos2);

            int open = 100;
            int close = 0;
            claw(open, close);
        }
    }

    // Moves the robot chassis
    public void robotMovement(double vertical, double strafe, double turn) {
        frontLeft.setPower(-vertical - strafe - turn);
        frontRight.setPower(-vertical + strafe + turn);
        backLeft.setPower(vertical + strafe - turn);
        backRight.setPower(vertical - strafe + turn);
    }

    // Emergency brake function
    public void brake() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    // Controls the wrist servos (placeholder)
    private void wrist(int Hpos1, int Hpos2, int Vpos1, int Vpos2) {
        // TODO: Implement wrist control logic
    }

    // Controls the claw servo (placeholder)
    private void claw(int open, int close) {
        // TODO: Implement claw control logic
    }

    // Controls arm (placeholder)
    public void newArm(double manualArmDeadband, double jointOnePower, double jointTwoPower) {
        // TODO: Implement arm control logic
    }

    // This is the previously commented-out arm movement code
    //public void armMovement(double manualArmPower, double manualArmDeadband, boolean manualMode) {
    //    if (Math.abs(manualArmPower) > manualArmDeadband) {
    //        jointOne.setPower(manualArmPower);
    //        if (!manualMode) {
    //            jointOne.setTargetPosition(jointOne.getCurrentPosition());
    //            jointOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //            jointOne.setPower(0.0);
    //            manualMode = true;
    //        } else {
    //            jointOne.setTargetPosition(jointOne.getCurrentPosition());
    //            jointOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //            jointOne.setPower(0.3);
    //            manualMode = false;
    //        }
    //    } else if (Math.abs(manualArmPower) < manualArmDeadband) {
    //        jointOne.setPower(0.1);
    //    }
    //}
}
