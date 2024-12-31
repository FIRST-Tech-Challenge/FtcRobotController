
package org.firstinspires.ftc.teamcode.theKingIsBack;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class yourWelcome extends OpMode {
    public Servo gripperServo;
    public Servo wristServo;
    public Servo leftArmServo;
    public Servo rightArmServo;
    public Servo deliveryDriveServo;
    public Servo deliveryGripperServo;
    public DcMotor intakeSlide;
    public DcMotor deliverySlideL;
    public DcMotor deliverySlideR;

    public AnalogInput wristEncoder;
    public AnalogInput leftArmEncoder;
    public AnalogInput rightArmEncoder;
    public AnalogInput deliveryDriveEncoder;
    /*
    Motor Tick Positions:
    Intake Slide In: 0
    DeliverySlideL Down: 0
    DeliverySlideR Down: 0

    Intake Slide Out: 76
    DeliverySlideL Up: -2200
    DeliverySlideR Up: 2200


    Servo Directions:
    leftArmServo: 0 is in, 1 is pickup, 0.1 transfer, 0.9 intake
    rightArmServo: 1 is pickup, 0 is in
    wristServo:
    GripperServo: 0.5 horizontal to robot, 0.05-0.1 is vertical pickup
    */

    @Override
    public void init() {
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");
        deliverySlideL = hardwareMap.get(DcMotor.class, "deliverySlideL");
        deliverySlideR = hardwareMap.get(DcMotor.class, "DeliverySlideR");

        wristEncoder= hardwareMap.get(AnalogInput.class, "wristEncoder");
        leftArmEncoder= hardwareMap.get(AnalogInput.class, "leftArmEncoder");
        rightArmEncoder= hardwareMap.get(AnalogInput.class, "rightArmEncoder");
        deliveryDriveEncoder= hardwareMap.get(AnalogInput.class, "deliveryDriveEncoder");

        //Set Encoder and Brake
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        deliverySlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        deliverySlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deliverySlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deliverySlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deliverySlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deliverySlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wrist position needs tuned
        wristServo.setPosition(0.4);
    }

    @Override
    public void loop() {
        telemetry.addLine("Intake Slide: " + intakeSlide.getCurrentPosition());
        telemetry.addLine("Delivery Slide L: " + deliverySlideL.getCurrentPosition());
        telemetry.addLine("Delivery Slide R: " + deliverySlideR.getCurrentPosition());

        //intake slides extend
        if(gamepad1.dpad_up){
            intakeSlide.setPower(0.1);
            intakeSlide.setTargetPosition(70);
            intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //intake slides in
        if(gamepad1.dpad_down){
            intakeSlide.setPower(0.1);
            intakeSlide.setTargetPosition(3);
            intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //arm rotate to transfer
        if(gamepad1.dpad_left){
            rightArmServo.setPosition(0.92);
            leftArmServo.setPosition(0.92);
        }
        //arm rotate to pickup
        if(gamepad1.dpad_right){
            rightArmServo.setPosition(0);
            leftArmServo.setPosition(0);
        }
        //grippers close needs programming of servo
        if(gamepad1.right_bumper){
            //gripperServo.setPosition(0);
        }
        //grippers open
        if(gamepad1.right_bumper){
            //gripperServo.setPosition(0.55);
        }

    }
}
