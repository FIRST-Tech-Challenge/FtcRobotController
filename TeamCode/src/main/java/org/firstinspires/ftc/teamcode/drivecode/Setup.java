package org.firstinspires.ftc.teamcode.drivecode;// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import org.firstinspires.ftc.robotcore.external.State;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @TeleOp

// public class Setup extends LinearOpMode {
//     telemetry.addData("Status", "Initialized");
//         telemetry.update();

//         // Initialize the hardware variables. Note that the strings used here as parameters
//         // to 'get' must correspond to the names assigned during the robot configuration
//         // step (using the FTC Robot Controller app on the phone).
//         fruntRight = hardwareMap.get(DcMotor.class, "fruntRight");
//         fruntLeft = hardwareMap.get(DcMotor.class, "fruntLeft");
//         jarmy = hardwareMap.get(DcMotor.class, "jarmy");
//         backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        
//         //reset the zero position of the arm, and set it to not use power when not recieving power.
//         craneArm = hardwareMap.get(DcMotor.class, "craneArm");
//         //craneArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         craneArm.setDirection(DcMotor.Direction.REVERSE);
//         craneArm.setTargetPosition(0);
//         craneArm.setPower(0.2);
//         craneArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         craneArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


//         //sets the servo variables to the physical servos for the airplane launcher and claw servo
//         airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
//         leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
//         rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        
//         leftClawRotator = hardwareMap.get(Servo.class, "leftClawRotator");
//         rightClawRotator = hardwareMap.get(Servo.class, "rightClawRotator");
        
//         //resets the zero position of the drawer slide motor
//         hookLifter = hardwareMap.get(DcMotor.class, "hookLifter");
//         hookLifter.setDirection(DcMotor.Direction.REVERSE);
//         hookLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         hookLifter.setTargetPosition(0);
//         hookLifter.setPower(0.5);
//         hookLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
//         //resets the zero position of the drawer slide motor
//         robotLifter = hardwareMap.get(DcMotor.class, "robotLifter");
//         robotLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         robotLifter.setTargetPosition(0);
//         robotLifter.setPower(0.75);
//         robotLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
//         telemetry.update();

//         //Sets all motors to starting position
//         //airplaneLauncher.setPosition(0);
//         //craneArm.setTargetPosition(100);
//         sleep(1000);
//         // leftClawRotator.setPosition(0.0);
        
//         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//         // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//         fruntRight.setDirection(DcMotorSimple.Direction.FORWARD);
//         jarmy.setDirection(DcMotorSimple.Direction.FORWARD);
//         fruntLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//         backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
//         State curState = State.LIFT;
        
//         // Wait for the game to start (driver presses PLAY)
//         telemetry.update();
//         waitForStart();
        
// }