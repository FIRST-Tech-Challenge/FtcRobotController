package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class UpdatedMain extends LinearOpMode {
    //Hand Servos
    private Servo hand_rotation_servo;
    private Servo hand_grip_servo;
    //Arm Motors
    private DcMotor arm_rotator_motor;
    private DcMotor arm_extender_motor;//not in use yet
    //Wheels
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor front_right_wheel;
    private DcMotor back_right_wheel;
    private final int delay = 10;
    private int point;
    private boolean claw_gripped = false;
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
            //resets encoder so arm current position is now the start
            arm_rotator_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //sets target position so the mode can be set
            arm_rotator_motor.setTargetPosition(0);
            //sets Run Mode to RUN_TO_POSITION
            arm_rotator_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(opModeIsActive()){
                update_driving();
                update_grip();
                update_arm_rotation(options.Set2);
                update_hand_rotato();
                display_data();
                telemetry.update();
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
        telemetry.addData("hand grip", hand_grip_servo.getPosition());
        telemetry.addData("Hand Rotation", hand_rotation_servo.getPosition());
    }
    public void update_driving(){
        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;
        back_left_wheel.setPower(((left_stick_y - left_stick_x)*-1) + right_stick_x);
        back_right_wheel.setPower(((left_stick_y + left_stick_x)*-1) - right_stick_x);
        front_left_wheel.setPower(((left_stick_y + left_stick_x)*-1) + right_stick_x);
        front_right_wheel.setPower(((left_stick_y - left_stick_x)*-1) - right_stick_x);
    }
    public void update_arm_rotation(options option){

        switch (option){
            case Set1:
                arm_rotator_motor.setPower(gamepad2.left_stick_y*((gamepad2.left_stick_y > 0 ? 0.8 : 0.9) - (gamepad2.right_bumper ? 0.1f : 0)) *-1);
                //arm_rotator_motor.setPower(gamepad2.left_stick_y);
                break;
            case Set2:
                arm_rotator_motor.setPower(1);
                if (gamepad2.left_stick_y > 0 ){
                    if(point == delay){
                        point = 0;
                        arm_rotator_motor.setTargetPosition(arm_rotator_motor.getCurrentPosition() + 1);
                        arm_rotator_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        telemetry.addData("running arm motor", true);
                    }
                } else if(gamepad2.left_stick_y < 0){
                    if(point == delay){
                        point = 0;
                        arm_rotator_motor.setTargetPosition(arm_rotator_motor.getCurrentPosition() - 1);
                        arm_rotator_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        telemetry.addData("running arm motor", true);
                    }
                }
                break;
            default:
                System.err.println("option not available");
                break;
        }
        point++;
    }
    public void update_grip(){

        if (gamepad2.left_bumper){
            if (hand_grip_servo.getPosition() < 0.65) {
                hand_grip_servo.setPosition(hand_grip_servo.getPosition() + 0.005);
            }

        } else if (gamepad2.right_bumper){
            if (hand_grip_servo.getPosition() > 0) {
                hand_grip_servo.setPosition(hand_grip_servo.getPosition() - 0.005);
            }
        }
    }
    public void update_hand_rotato(){
        if (gamepad2.left_trigger ==1 || gamepad2.right_trigger == 1){
            hand_rotation_servo.setPosition(hand_rotation_servo.getPosition() + (gamepad2.left_trigger + (gamepad2.right_trigger)*-1)*0.005);
        }

    }
    private enum options{
        Set1,
        Set2
    }
}
