package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MotorTelemetry extends LinearOpMode {
    private double TouchPadInput = 0;

    public double LeftServo;
    public double RightServo;
    private double V4Bpos = .78;
    private double Flex = 0;
    private double Yaw = 0;
    @Override
    public void runOpMode(){

        DcMotor IntakeRight = hardwareMap.dcMotor.get("Intake Right"); // Ehub Port 0 // X Button To Position Automatically? // Joystick Up And Down?
        DcMotor IntakeLeft = hardwareMap.dcMotor.get("Intake Left");   // Ehub Port 1 // ----------------------------------
        DcMotor RightLift = hardwareMap.dcMotor.get("Right Lift");     // Ehub Port 2 // Triangle Button To Delivery Position
        DcMotor LeftLift = hardwareMap.dcMotor.get("Left Lift");       // Ehub Port 3 // ------------------------------------

        Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
        Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
        Servo IntakeV4B = hardwareMap.servo.get("Intake V4B");     // Chub Port 3 // Preset To Swing Out With X

        Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
        Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
        Servo OuttakeV4B = hardwareMap.servo.get("Outtake V4B");   // Ehub Port 2 // Preset With Triangle

        LeftServo = Flex - (.5 * Yaw);
        RightServo = Flex + (.5 * Yaw);

        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // Resets the position so it sets it's current position to 0
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Resets the position so it sets it's current position to 0

        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift.setTargetPosition(0);     // Makes sure it starts at the set 0
        RightLift.setTargetPosition(0);    // Makes sure it starts at the set 0

        IntakeRight.setTargetPosition(0);
        IntakeLeft.setTargetPosition(0);

        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Sets the mode so we can say to move the motor a certain amount of ticks
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Sets the mode so we can say to move the motor a certain amount of ticks

        IntakeRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftIntakeWrist.setPosition(LeftServo);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1
        IntakeV4B.setPosition(.78);   // Sets the intake virtual four bar to the starting position
        //RightIntakeV4B.setPosition(1);  // Sets the intake virtual four bar to the starting position

        OuttakeWrist.setPosition(0);    // Sets the outtake wrist to the starting position

        OuttakeV4B.setPosition(1);  // Sets the outtake virtual four bar to the starting position


        waitForStart();
        while(opModeIsActive()){

            IntakeV4B.setPosition(V4Bpos);
            //RightIntakeV4B.setPosition(V4Bpos);
            LeftServo = Flex - (.5 * Yaw); //Calculates required servo angles for combined flex and yaw motion
            RightServo = Flex + (.5 * Yaw);//^
            LeftIntakeWrist.setPosition(LeftServo); //Sets servos to calculated positions
            RightIntakeWrist.setPosition(RightServo); //^

            telemetry.addData("Right Intake Position", IntakeRight.getCurrentPosition());
            telemetry.addData("Left Intake Position", IntakeLeft.getCurrentPosition());

            telemetry.addData("Right Lift Position", RightLift.getCurrentPosition());
            telemetry.addData("Left Lift Position", LeftLift.getCurrentPosition());

            telemetry.update();
        }
    }
}
