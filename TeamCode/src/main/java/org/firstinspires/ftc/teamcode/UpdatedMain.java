package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class UpdatedMain extends LinearOpMode {
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
    public static double arm_less_zero = 0.9;
    public static double arm_more_zero = 0.9;
    public static double triggerModifier = 0.005;
    public static int armUp = -316;
    // IF YOU CHANGE TELL PEOPLE!!! ^^^^^
    //main loop
    @Override
    public void runOpMode() throws InterruptedException {
        //setting up motors
        initialize_hand();
        initialize_arm();
        initialize_wheels(options.Set1);
        //wait for start
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                update_driving();
                update_grip();
                update_arm_rotation();
                update_hand_rotation();
                display_data();
                update_arm_extension();
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

        //Arm
        dashboardTelemetry.addData("Arm Extension", arm_extender_motor.getCurrentPosition());
        dashboardTelemetry.addData("Arm Rotation", arm_rotator_motor.getCurrentPosition());

        //Vars that should be changeable (please)
        dashboardTelemetry.addData("Trigger Modifier", triggerModifier);
        dashboardTelemetry.addData("Arm > 0 Modifier", arm_more_zero);
        dashboardTelemetry.addData("Arm < 0 Modifier", arm_less_zero);

        //other
        telemetry.addData("hand grip", hand_grip_servo.getPosition());
        telemetry.addData("Hand Rotation", hand_rotation_servo.getPosition());

        //update telemetry
        dashboardTelemetry.update();
        telemetry.update();
    }
    public void update_driving(){
        double left_stick_x = gamepad1.left_stick_x * -1;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;
        back_left_wheel.setPower(((left_stick_y - left_stick_x)*-1) + right_stick_x);
        back_right_wheel.setPower(((left_stick_y + left_stick_x)*-1) - right_stick_x);
        front_left_wheel.setPower(((left_stick_y + left_stick_x)*-1) + right_stick_x);
        front_right_wheel.setPower(((left_stick_y - left_stick_x)*-1) - right_stick_x);
    }
    public void update_arm_rotation(){
        //TODO: Extend this over multiple lines

        arm_rotator_motor.setPower(gamepad2.left_stick_y*((gamepad2.left_stick_y > armUp ? arm_less_zero : arm_more_zero)) *-1);
        //TODO: When we get a encoder on this use set pos and not use power ever again
    }
    public void update_grip(){/*
        if (gamepad2.left_bumper){
            if (hand_grip_servo.getPosition() < 1) {
                hand_grip_servo.setPosition(hand_grip_servo.getPosition() + 0.005);
            }

        } else if (gamepad2.right_bumper){
            if (hand_grip_servo.getPosition() > 0) {
                hand_grip_servo.setPosition(hand_grip_servo.getPosition() - 0.005);
            }
        }*/
        if (gamepad2.left_bumper){
            hand_grip_servo.setPosition(hand_grip_servo.getPosition() + triggerModifier);

        } else if (gamepad2.right_bumper){
            hand_grip_servo.setPosition(hand_grip_servo.getPosition() - triggerModifier);
        }
    }
    public void update_hand_rotation(){
        if (gamepad2.left_trigger ==1 || gamepad2.right_trigger == 1){

            //TODO: Separate this code to multiple lines and comment it.
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
