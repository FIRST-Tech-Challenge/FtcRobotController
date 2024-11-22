package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class DiffyInverseKinematicTest extends LinearOpMode {
    private ElapsedTime IntakeClawTime = new ElapsedTime();
    private double TouchPadInput = .5;
    public double LeftServo = .5;
    public double RightServo = .5;
    private double V4Bpos = 1;
    private double Flex = .5;
    private double Yaw = .5;
    private enum V4Bstate{
        START,
        INTAKE,
        TRANSFER
    }
    V4Bstate state = V4Bstate.START;
    private boolean IntakeClawClosed = false;
    private boolean OuttakeActive = false;
    @Override
    public void runOpMode() throws InterruptedException {
        //Servo IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
        Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
        Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
        Servo RightIntakeV4B = hardwareMap.servo.get("Right Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
        Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
        LeftServo = Flex + (1/2)*Yaw;
        RightServo = Flex - (1/2)*Yaw;
        //IntakeClaw.setPosition(0);    // Closes Intake Claw
        LeftIntakeWrist.setPosition(LeftServo);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1
        LeftIntakeV4B.setPosition(1);   // Sets the intake virtual four bar to the starting position
        RightIntakeV4B.setPosition(1);  // Sets the intake virtual four bar to the starting position
        waitForStart();
        while (opModeIsActive()){
            /* Closes the intake claw
            if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && !IntakeClawClosed){ // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is open
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(1);  // Closes the intake claw
                IntakeClawClosed = true;    // Since the intake claw was closed we change this to stay accurate
            }
            // Opens the intake claw
            else if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && IntakeClawClosed){  // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is closed
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(0);  // Opens the intake claw
                IntakeClawClosed = false;   // Since the intake claw was opened we change this to stay accurate
            }*/
            switch (state){
                case START:
                    if (gamepad1.a){    // Transfer Position
                        V4Bpos = 1;
                        Flex = .25;
                        state = V4Bstate.INTAKE;
                    }
                    break;
                case INTAKE:
                    V4Bpos = .3;
                    Flex = gamepad1.left_stick_x;
                    if (gamepad1.touchpad_finger_1){   // Allows manual yaw control if a finger is on the touchpad
                        Yaw = gamepad1.touchpad_finger_1_x;     // Taking value from touchpad and saving as our desired yaw value
                    }
                    else {
                        Yaw = 0; //Sets yaw to 0 if no finger is detected on the touchpad
                    }
                    if (gamepad1.x){  //Transfer position
                        V4Bpos = 1;
                        Flex = .25;
                        state = V4Bstate.TRANSFER;
                    }
                    break;
                case TRANSFER:
                    state = V4Bstate.START;
                    break;
            }
            if (gamepad1.right_trigger > 0){
                V4Bpos = 0.5 - 0.5 * gamepad1.right_trigger;
                Flex = .7;
            }
            LeftServo = Flex - (.5*Yaw); //Calculates required servo angles for combined flex and yaw motion
            RightServo = Flex + (.5*Yaw);//^
            LeftIntakeWrist.setPosition(LeftServo); //Sets servos to calculated positions
            RightIntakeWrist.setPosition(RightServo); //^
            LeftIntakeV4B.setPosition(V4Bpos);
            RightIntakeV4B.setPosition(V4Bpos);
            telemetry.addData("Touch Pad Yaw Input", Yaw)
                    .addData("Flex Input", Flex)
                    .addData("Left Wrist Target", LeftServo)
                    .addData("Right Wrist Target", RightServo)
                    .addData("Left Wrist Actual", LeftIntakeWrist.getPosition())
                    .addData("Right Wrist Actual", RightIntakeWrist.getPosition())
                    .addData("Claw Closed?", IntakeClawClosed);
            telemetry.update();
        }
    }
}