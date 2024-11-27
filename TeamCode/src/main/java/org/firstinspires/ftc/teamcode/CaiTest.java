package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class CaiTest extends LinearOpMode {
    //Hand Servos
    private Servo hand_rotation_servo;
    private Servo hand_grip_servo;
    //Arm Motors
    private DcMotor arm_rotator_motor;
    private DcMotor arm_extender_motor;
    //Wheels
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor front_right_wheel;
    private DcMotor back_right_wheel;


    //Road Runner Dashboard
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // Runtime modifiable values
    // IF YOU CHANGE TELL PEOPLE!!! vvvvv (people might stab you if you don't)

    // Runtime modifiable values should be public static
    public static int armVelocity;
    public static double triggerModifier = 0.005;
    /*
    private final int swap  = -329;

    private double up_speed = 0.7;
    private double down_speed = 0.7;
    */
    // IF YOU CHANGE TELL PEOPLE!!! ^^^^^

    // misc vars
    private int arm_target;

    // tps calculations
    private int runs = 0;
    private ElapsedTime time;
    private double tps = 0;

    //main loop
    @Override
    public void runOpMode() throws InterruptedException {
        //setting up motors
        initialize_hand();
        initialize_arm();
        initialize_wheels(options.Set1);
        //here in case it results in movement (no movement allowed during init TeleOp)
        arm_rotator_motor.setTargetPosition(0);
        arm_rotator_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wait for start
        waitForStart();
        if(opModeIsActive()){
            //set up timimgs
            time = new ElapsedTime();
            time.reset();
            while(opModeIsActive()){
                update_driving();
                update_grip();
                update_arm_rotation();
                update_hand_rotation();
                display_data();
                update_arm_extension();
                runs++;
                if (time.seconds() > 1) {
                    tps = runs / time.seconds();
                    runs = 0;
                    time.reset();
                }
            }
        }
    }
    /**
     * initialize the hands servos
     * */
    public void initialize_hand(){
        hand_rotation_servo = hardwareMap.get(Servo.class, "hand_rotator");
        hand_grip_servo = hardwareMap.get(Servo.class, "hand_servo");
        hand_rotation_servo.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * initialize the arms motors
     */
    public void initialize_arm(){
        arm_rotator_motor = hardwareMap.get(DcMotor.class, "arm_control");
        arm_extender_motor = hardwareMap.get(DcMotor.class, "arm_extender");
        arm_rotator_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm_rotator_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_rotator_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * initialize the wheels
     */
    public void initialize_wheels(options option){
        front_left_wheel = hardwareMap.get(DcMotor.class, "front_left_wheel");
        back_left_wheel = hardwareMap.get(DcMotor.class, "back_left_wheel");
        front_right_wheel = hardwareMap.get(DcMotor.class, "front_right_wheel");
        back_right_wheel = hardwareMap.get(DcMotor.class, "back_right_wheel");
        switch (option){
            case Set1:
                back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
                back_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
                front_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
                front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case Set2:
                back_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
                back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
                front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
                front_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            default:
                break;
        }

    }

    /**
     * displays data
     */
    public void display_data(){
        //Hand
        dashboardTelemetry.addData("Hand Grip", hand_grip_servo.getPosition());
        dashboardTelemetry.addData("Hand Rotation", hand_rotation_servo.getPosition());
        telemetry.addData("Hand Grip", hand_grip_servo.getPosition());
        //Arm
        dashboardTelemetry.addData("Arm Extension", arm_extender_motor.getCurrentPosition());
        dashboardTelemetry.addData("Arm Rotation", arm_rotator_motor.getCurrentPosition());
        dashboardTelemetry.addData("Arm Target", arm_target);
        //Vars that should be changeable (please)
        dashboardTelemetry.addData("Trigger Modifier", triggerModifier);
        dashboardTelemetry.addData("Arm velocity", armVelocity);
        //other
        telemetry.addData("hand grip", hand_grip_servo.getPosition());
        telemetry.addData("Hand Rotation", hand_rotation_servo.getPosition());
        telemetry.addData("Arm Position", arm_rotator_motor.getCurrentPosition());
        telemetry.addData("left stick y", gamepad2.left_stick_y);
        telemetry.addData("motor set position to move to", arm_rotator_motor.getTargetPosition());
        dashboardTelemetry.addData("tps", tps);

        //update telemetry
        dashboardTelemetry.update();
        telemetry.update();
    }
    public void update_driving(){
        double left_stick_x = gamepad1.left_stick_x * -1;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;
        back_left_wheel.setPower((((left_stick_y - left_stick_x)*-1) + right_stick_x)*0.5);
        back_right_wheel.setPower((((left_stick_y + left_stick_x)*-1) - right_stick_x)*0.5);
        front_left_wheel.setPower((((left_stick_y + left_stick_x)*-1) + right_stick_x)*0.5);
        front_right_wheel.setPower((((left_stick_y - left_stick_x)*-1) - right_stick_x)*0.5);
    }
    private int tacos = 0;
    public void update_arm_rotation(){
        //arm_rotator_motor.setPower(gamepad2.left_stick_y*((gamepad2.left_stick_y > 0 ? 0.7 : 0.6)) *-1);

        if(tacos == 40) {
            arm_target = (int) (arm_rotator_motor.getTargetPosition() + ((gamepad2.left_stick_y * -10)));

            arm_rotator_motor.setTargetPosition(arm_target);
            arm_rotator_motor.setPower(1);
            arm_rotator_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            tacos++;
        }
        /*if(arm_rotator_motor.getCurrentPosition() >= swap){
            if (gamepad2.left_stick_y > 0){
                arm_rotator_motor.setPower(gamepad2.left_stick_y*0.5 *-1);
            }else if (gamepad2.left_stick_y < 0){
                arm_rotator_motor.setPower(gamepad2.left_stick_y*0.7 *-1);
            }
        } else if (arm_rotator_motor.getCurrentPosition() < swap){
            if (gamepad2.left_stick_y > 0){
                arm_rotator_motor.setPower(gamepad2.left_stick_y*0.7 *-1);
            }else if (gamepad2.left_stick_y < 0){
                arm_rotator_motor.setPower(gamepad2.left_stick_y*0.5 *-1);
            }
        }*/

    }
    public void update_grip(){
        /*if (gamepad2.left_bumper){
            hand_grip_servo.setPosition(hand_grip_servo.getPosition() + triggerModifier);

        } else if (gamepad2.right_bumper){
            hand_grip_servo.setPosition(hand_grip_servo.getPosition() - triggerModifier);
        }*/

        //0 open 0.75 closed
        if(gamepad2.a){
            hand_grip_servo.setPosition(hand_grip_servo.getPosition() - 0.01);
        }else if (gamepad2.b){
            hand_grip_servo.setPosition(hand_grip_servo.getPosition() + 0.01);
        }
    }
    public void update_hand_rotation(){
        if (gamepad2.left_trigger ==1 || gamepad2.right_trigger == 1){


            hand_rotation_servo.setPosition(hand_rotation_servo.getPosition() + (gamepad2.left_trigger + (gamepad2.right_trigger)*-1)*triggerModifier);
        }

    }
    public void update_arm_extension(){
        arm_extender_motor.setPower(gamepad2.right_stick_y * -1);
    }
    // Please Explain this code - @GoldStar184
    //lets you quickly change what set of items you want to use
    // currently only used for setting up wheel directions
    private enum options{
        Set1,
        Set2
    }
}