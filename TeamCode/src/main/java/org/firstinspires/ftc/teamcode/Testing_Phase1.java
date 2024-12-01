package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Drive Phase1")
public class Testing_Phase1 extends LinearOpMode {
    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable

    private final int MAX_TARGET_LIFT = 2825;	// Full extension limit of the lift slides
    private final int MAX_EXTENSION_LENGTH = 500;  // Full extension limit of the intake slides	

    private int LiftTarget = 0;
    private int ExtensionTarget = 0;

    private int LiftPower = 1;			// predetermined extending speed - could be a final at this point
    private double ExtensionPower = .75;		// predetermined extending speed - could be a final at this point

    private int precision = 2;                    // chassis motor power reduction factor 1
    private boolean intakeClawClosed = false;     // claw holder variable
    private boolean outtakeClawClosed = false;   // claw holder variable
	
    private ElapsedTime stateDelay = new ElapsedTime(); // possible delay timer for state change
    private ElapsedTime transferTime = new ElapsedTime(); // delay timer for transfer
    private ElapsedTime ClawTime = new ElapsedTime();     // delay timer for claw
	
    private double LeftServo;                // currently intake wrist servo left
    private double RightServo;               // currently intake wrist servo right
    private double V4Bpos = 1;              // currently intake V4B
    private double Flex = .1;               // currently intake wrist angle
    private double Yaw = 0;                 // currently intake wrist twist

    private enum V4Bstate {                 // states of intake/outake machine
        START,
        INTAKE,
        TRANSFER,
        OUTTAKE,
	CLIMB					//  TODO add OUTTAKE and CLIMB state  *******************************
    }

    V4Bstate state = V4Bstate.START;           // initial state of intake/outake servos

    @Override
    public void runOpMode() throws InterruptedException {
	// ************************************* MOTORS *******************************************************    
        DcMotor FrontRight = hardwareMap.dcMotor.get("Front Right");       // Chub Port 0 // Gamepad 1
        DcMotor BackRight = hardwareMap.dcMotor.get("Back Right");         // Chub Port 1 // Left Stick For Moving
        DcMotor FrontLeft = hardwareMap.dcMotor.get("Front Left");         // Chub Port 2 // Right Stick For Turning
        DcMotor BackLeft = hardwareMap.dcMotor.get("Back Left");           // Chub Port 3
	    DcMotor IntakeRight = hardwareMap.dcMotor.get("Intake Right");	   // Ehub Port 0
	    DcMotor IntakeLeft = hardwareMap.dcMotor.get("Intake Left");	   // Ehub Port 1
	    DcMotor RightLift = hardwareMap.dcMotor.get("Right Lift");	   // Ehub Port 2
	    DcMotor LeftLift = hardwareMap.dcMotor.get("Left Lift");	   // Ehub Port 3

	// ****************************************** SERVOS ****************************************************
	    Servo IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
    	Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Flex and Yaw controlled
        Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // IFlex and Yaw controlled
        Servo IntakeV4B = hardwareMap.servo.get("Intake V4B");   	      // Chub Port 3 // Preset To Swing Out With A
        //Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------

	    Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");         // Ehub Port 0
	    Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");	   // Ehub Port 1
        Servo OuttakeV4B = hardwareMap.servo.get("Outtake V4B");

		
        LeftServo = Flex - (.5 * Yaw);                                      // intake wrist servo calc
        RightServo = Flex + (.5 * Yaw);                                     // intake wrist servo calc

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);             // Reverses the direction the motor turns
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);           // Reverses the direction the motor turns

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // Sets the motor to be locked when stopped
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // Sets the motor to be locked when stopped
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped


        IntakeClaw.setPosition(0);                  // Closes Intake Claw
        LeftIntakeWrist.setPosition(LeftServo);     // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1
        IntakeV4B.setPosition(1);               // Sets the intake virtual four bar to the starting position

        // Beginning of opmode

        waitForStart();
        while (opModeIsActive()) {

            //*************************** DRIVE CONTROLS **************************************************
            // check for driving input
            double y = gamepad1.left_stick_y;         // Remember, this is reversed!
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.left_stick_x;       // Measures turning
            if (gamepad1.right_trigger >= 0.75) {     // Checks if the Right Trigger was pressed and if so it continues the stuff in the brackets
                y = -gamepad1.left_stick_y;           // Remember, this is reversed!
                x = gamepad1.right_stick_x * 1.1;     // Counteract imperfect strafing
                rx = gamepad1.left_stick_x;           // Measures turning
            }

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);    // calculate motor movement math and adjust according to lift height or manual precision mode selection

            // check for Turbo or Precision Mode
            if (gamepad1.left_bumper) {         // Left bumper is being pressed
                precision = 1;                  // set speed to full power - TURBO MODE
            } else if (gamepad1.right_bumper) { // Right bumper is being pressed
                precision = 4;                  // set speed to 1/4 power - PRECISION MODE
            } else {
                precision = 2;                  // reset default speed to half power
            }

            // calculate motor power
            denominator = denominator * precision;          // this adjusts motor speed to the desired precision level
            frontLeftPower = (y + x + rx) / denominator;    // Does math to convert stick movement to motor powers
            backLeftPower = (y - x + rx) / denominator;     // Does math to convert stick movement to motor powers
            frontRightPower = (y - x - rx) / denominator;   // Does math to convert stick movement to motor powers
            backRightPower = (y + x - rx) / denominator;    // Does math to convert stick movement to motor powers

            // State machine for intake and outake servo control

            switch (state) {                 //  These controls have been switched to gamepad2
                case START:
                    V4Bpos = .78;	            	// Bar up upon entering state
                    Flex = 0;		            // Wrist fully up upon entering state
                    if (gamepad2.a){        	// Switch to INTAKE Position with a button
                        state = V4Bstate.INTAKE;
                        stateDelay.reset();    // used in conjuction with stateDelay checking in each state
                    }
                    break;
                case INTAKE:    
                    //                  TODO        Proposed alternate code
                    //                              Create button controls and sequencing for TRANSFER and OUTTAKE cases
		    if (stateDelay.seconds() < .25 ){break;}			// state delay at entry to every state
                    V4Bpos = .3 * ( 1 - (gamepad2.right_trigger));	// Factors trigger value or returns to baseline position if no input
                    Flex = .6;					        // TODO - determine default Flex value for here
                    Yaw = 0 + gamepad2.touchpad_finger_1_x;	        // Taking value from touchpad and saving as our desired yaw value
		    if (gamepad2.b){					// b button full extension
			    ExtensionTarget = 490;
		    }
		    else if (gamepad2.y){				// y button full retraction and switch to TRANSFER state
			    ExtensionTarget = 1;
                            state = V4Bstate.TRANSFER;
                   	    stateDelay.reset();				// used in conjuction with stateDelay checking in each state
		    }
		    if (gamepad2.dpad_left && ExtensionTarget < (MAX_EXTENSION_LENGTH - 10)){	// checks for dpadinput to micro adjust extenstion out by 10
			ExtensionTarget = ExtensionTarget + 10;
		    }
		    else if(gamepad2.dpad_right && ExtensionTarget > 11){			// checks for dpadinput to micro adjust extenstion in by 10
			ExtensionTarget = ExtensionTarget - 10;
		    } 
                    if (gamepad2.right_bumper && ClawTime.seconds() >= .3 ){    			        // Toggle claw with rightbumper
                 		if (intakeClawClosed){            	// Determine claw position
		             		IntakeClaw.setPosition(.5);  	// Open claw if closed
    	             		}
	                  	else {  		
		                	IntakeClaw.setPosition(0);    	// Close claw if open
	                	}
	            		intakeClawClosed = !intakeClawClosed;	// Switch position status variable
			    	ClawTime.reset();			// reset claw timer
		            }
                    if (gamepad2.x){		                  // x button Transfer position
                        // retract intake slide
			    // set intakeV4B position up and wrist and claw to transfer alignment
			    // open outake claw and toggle holder variable
			    // set lift to low position
			    // lower outake claw onto specimen
			    // toggle claws and holder variable
			    
			state = V4Bstate.TRANSFER;
			stateDelay.reset();    // used in conjuction with stateDelay checking in each state    
                    }
                    break;
			    
                case TRANSFER:                          // TODO sequence positioning and claw open/close
		    if (stateDelay.seconds() < .25 ){break;}  // state delay at entry to every state
/*		    if (){

			    
                        stateDelay.reset();    // used in conjuction with stateDelay checking in each state
		    }			        // button activation for claw transfer
				*/	
                    break;
			    
		case OUTTAKE:                           // TODO   raising and scoring button functions   *********************
		    if (stateDelay.seconds() < .25 ){break;}  // state delay at entry to every state
                    if (gamepad2.a){             // High delivery preset
			LiftTarget = 2820;	// sets lift to high delivery level
			OuttakeV4B.setPosition(0);	// adjust v4b to deliver
			OuttakeWrist.setPosition(.7);	// adjust claw to deliver
		     }
		     if (gamepad2.x){             // Low delivery preset
			LiftTarget = 2820;	// sets lift to high delivery level
			OuttakeV4B.setPosition(0);	// adjust v4b to deliver
			OuttakeWrist.setPosition(.7);	// adjust claw to deliver
		     }
		     if (gamepad2.left_bumper && ClawTime.seconds() >= .3 ){    			        // Toggle claw with leftbumper
                 		if (outtakeClawClosed){            	// Determine claw position
		             		OuttakeClaw.setPosition(.5);  	// Open claw if closed
    	             		}
	                  	else {  		
		                	OuttakeClaw.setPosition(0);    	// Close claw if open
	                	}
	            		outtakeClawClosed = !outtakeClawClosed;	// Switch position status variable
			    	ClawTime.reset();			// reset claw timer
		    }
    		    if (gamepad2.dpad_up && LiftTarget < (MAX_TARGET_LIFT - 10)){	// checks for dpadinput to micro adjust lift up by 10
			LiftTarget = LiftTarget + 10;
		    }
		    else if(gamepad2.dpad_right && LiftTarget > 11){			// checks for dpadinput to micro adjust lift down by 10
			LiftTarget = LiftTarget - 10;
		    } 

        	//  if(which button to go back to intake state){
			    //   TODO    adjust v4bars and wrists to setup next intake and transfer sequence
			    state = V4Bstate.INTAKE;
	                    stateDelay.reset();    // used in conjuction with stateDelay checking in each state
            	//  }			    
                    break;
		case CLIMB:
		    if (stateDelay.seconds() < .25 ){break;}  // state delay at entry to every state
			//	 TODO incorporate climbing presets for buttons and lift motor positions
			    
		    break;
            }

            // Continuosly looping code outside of State Machine
	    
   	    if (gamepad1.right_trigger > 0 && IntakeLeft.getCurrentPosition() >= 50) {     // This righttrigger override is an alternate TRANSFER state selector
		    //  TODO   for this to be active at all times, care must be taken to address the lift position and should be called here prior to the state change
		    //  the button will change to INTAKE state but the state delay timer will keep the extension from operating for .25 seconds
   	             state = V4Bstate.INTAKE;         // changes to INTAKE state to allow manual adjustment
            }
            // Intake and Wrist positioning
            IntakeV4B.setPosition(V4Bpos);      // issue intake V4B servo position
            LeftServo = Flex - (.5 * Yaw); //Calculates required servo angles for combined flex and yaw motion
            RightServo = Flex + (.5 * Yaw);//^
            LeftIntakeWrist.setPosition(LeftServo); //Sets servos to calculated positions
            RightIntakeWrist.setPosition(RightServo); // ^
	    if (ExtensionTarget <= MAX_EXTENSION_LENGTH){
		IntakeLeft.setTargetPosition(ExtensionTarget);   // adjust intake to targetposition if within MAX boundary
		IntakeRight.setTargetPosition(ExtensionTarget);  // adjust intake to targetposition if within MAX boundary
		IntakeLeft.setPower(ExtensionPower);		 // issues power to Left Extension
		IntakeRight.setPower(ExtensionPower);		 // issues power to Right Extension
	    }

            // Lift Power and Management
	    if (!(LiftTarget > MAX_TARGET_LIFT)) {    // (TargetLift < MAX_TARGET_LIFT)
            RightLift.setTargetPosition(LiftTarget);   // adjust lift to targetposition if within MAX boundary
            LeftLift.setTargetPosition(LiftTarget);   // adjust lift to targetposition if within MAX boundary
            RightLift.setPower(LiftPower);           // issues power to RightLift
            LeftLift.setPower(LiftPower);           // issues power to LeftLift
        }

            // issue Drive Wheels motor power
            FrontLeft.setPower(frontLeftPower);    // Sets the front left wheel's power
            BackLeft.setPower(backLeftPower);     // Sets the back left wheel's power
            FrontRight.setPower(frontRightPower);  // Sets the front right wheel's power
            BackRight.setPower(backRightPower);   // Sets the back right wheel's power

            // Telemetry information
            telemetry.addData("Touch Pad Yaw Input", Yaw)
                     .addData("Flex Input", Flex)
                     .addData("Left Wrist Target", LeftServo)
                     .addData("Right Wrist Target", RightServo)
                     .addData("Left Wrist Actual", LeftIntakeWrist.getPosition())
                     .addData("Right Wrist Actual", RightIntakeWrist.getPosition());
            telemetry.update();
                 
       }
    }
}
