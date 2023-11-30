package org.firstinspires.ftc.teamcode.drivecode;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad.*;

import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Map;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import com.qualcomm.robotcore.hardware.TouchSensor;



@TeleOp(name="Driver Mode", group="Linear Opmode")
//@Disabled
public class DriverMode extends LinearOpMode {

    // Declare OpMode members.
    private ZeroPowerBehavior brake = ZeroPowerBehavior.BRAKE;
    private ZeroPowerBehavior floatt = ZeroPowerBehavior.FLOAT;
    private DcMotor fruntLeft, fruntRight, backLeft, jarmy;
    private DcMotor craneArm;
    private Servo airplaneLauncher;
    private Servo leftClawServo;
    private Servo rightClawServo;

    private Servo leftClawRotator;
    private Servo rightClawRotator;
    private DcMotor hookLifter;
    private DcMotor robotLifter;
    private float leftStickX;
    private float leftStickY;
    private float rightStickX;
    private float rightStickY;
    private float liftAmount;
    
    
    public enum State{
        LIFT,
        DOWN
    }
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fruntRight = hardwareMap.get(DcMotor.class, "fruntRight");
        fruntRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        fruntLeft = hardwareMap.get(DcMotor.class, "fruntLeft");
        fruntLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        jarmy = hardwareMap.get(DcMotor.class, "jarmy");
        jarmy.setDirection(DcMotorSimple.Direction.FORWARD);
        
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //reset the zero position of the arm, and set it to not use power when not recieving power.
        craneArm = hardwareMap.get(DcMotor.class, "craneArm");
        craneArm.setDirection(DcMotor.Direction.REVERSE);
        craneArm.setTargetPosition(0);
        craneArm.setPower(0.2);
        craneArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craneArm.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);


        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
        
        
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        
        leftClawRotator = hardwareMap.get(Servo.class, "leftClawRotator");
        rightClawRotator = hardwareMap.get(Servo.class, "rightClawRotator");
        
        //resets the zero position of the drawer slide motor
        hookLifter = hardwareMap.get(DcMotor.class, "hookLifter");
        hookLifter.setDirection(DcMotor.Direction.REVERSE);
        hookLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookLifter.setTargetPosition(0);
        hookLifter.setPower(0.5);
        hookLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //resets the zero position of the drawer slide motor
        robotLifter = hardwareMap.get(DcMotor.class, "robotLifter");
        robotLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotLifter.setTargetPosition(0);
        robotLifter.setPower(1);
        robotLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.update();

        //Sets all motors to starting position
        //airplaneLauncher.setPosition(0);
        //craneArm.setTargetPosition(100);
        sleep(1000);
        // leftClawRotator.setPosition(0.0);
        
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        
        State curState = State.LIFT;
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.update();
        waitForStart();
        
        if(opModeIsActive()){
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
               telemetry.addData("Status", "Running");

                fruntRight.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x) + -gamepad1.right_stick_x) *0.8);
                fruntLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x) * 0.8);
                jarmy.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) + -gamepad1.right_stick_x) *0.8);
                backLeft.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x) + gamepad1.right_stick_x) *0.8);
            
                telemetry.addData("hookLifter position:", hookLifter.getCurrentPosition());
            
                switch(curState){
                    case LIFT:
                        if(gamepad1.dpad_up){
                        moveVertically(hookLifter, 1300, 0.3);
                        robotLifter.setTargetPosition(6000);
                        curState = State.DOWN;
                    }
                    break;
                    case DOWN:
                        if(gamepad1.dpad_left){
                            robotLifter.setTargetPosition(-7000);
                            curState = State.LIFT;
                        }
                        else if(gamepad1.dpad_down){
                            moveVertically(hookLifter, 0, 0.3);
                            //curState = State.LIFT;
                        }
                        break;
                    default:
                        curState = State.LIFT;
                }
                
                
                /*
                Controller buttons use different labels, so
                A is the Blue Cross
                X is the Pink Square
                B is the Red Circle
                Y is the Green Triangle
                */
                if(gamepad2.a){
                    neutral();
                }else if(gamepad2.x){
                    placePixelHigh();
                }else if(gamepad2.b){
                    pickupPixel();
                }else if(gamepad2.y){
                    placePixelLow();
                }
                
                
                if(gamepad2.right_bumper){
                    openClaw();
                }else if(gamepad2.left_bumper){
                    closeClaw();
                }
                
                if(gamepad2.dpad_right){
                    airplaneLauncher.setPosition(1);
                }else if(gamepad2.dpad_up){
                    craneCounterWeight();
                }else if(gamepad2.dpad_left == true){
                    craneArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(1000);
                    craneArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    craneArm.setTargetPosition(0);
                    craneArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
    
                
                telemetry.update();
            
            }
        }
    }
    public void craneCounterWeight(){
        craneArm.setTargetPosition(1000);
        leftClawRotator.setPosition(1);
        rightClawRotator.setPosition(0);
    }
    public void placePixelLow(){
        craneArm.setTargetPosition(1440);
        leftClawRotator.setPosition(1);
        rightClawRotator.setPosition(0);
    }
    public void placePixelHigh(){
        craneArm.setTargetPosition(1200);
        leftClawRotator.setPosition(1);
        rightClawRotator.setPosition(0);
    }
    public void neutral(){
        craneArm.setTargetPosition(200);
        leftClawRotator.setPosition(1);
        rightClawRotator.setPosition(0);
    }
    public void pickupPixel(){
        craneArm.setTargetPosition(0);
        rightClawRotator.setPosition(0.38);
        leftClawRotator.setPosition(0.62);
    }
    public void openClaw(){
        leftClawServo.setPosition(1);
        rightClawServo.setPosition(0);
    }
    
    public void closeClaw() {
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(1);
    }
    
    public void moveVertically(DcMotor mot, int position, double power){
        mot.setPower(0);
        mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot.setTargetPosition(0);
        mot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mot.setPower(0);
        mot.setTargetPosition(position);
        mot.setPower(power);
    }
}
