package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public DcMotor motorFR; // Front Right
    public DcMotor motorFL; // Front Left
    public DcMotor motorBR; // Back Right
    public DcMotor motorBL; // Back Left

    public DcMotor motorSlider; // Slider
    public DcMotor motorArm; // Elbow or Arm
    public DcMotor motorHanging; // Hanging

    public Servo servoCR; // Claw Right
    public Servo servoCL; // Claw left
    public Servo servoDrone; // Drone


    public Robot(HardwareMap hardwareMap) {
        motorFL = hardwareMap.get(DcMotor.class, "front_left");
        motorFR = hardwareMap.get(DcMotor.class, "front_right");
        motorBL = hardwareMap.get(DcMotor.class, "back_left");
        motorBR = hardwareMap.get(DcMotor.class, "back_right");
        motorSlider = hardwareMap.get(DcMotor.class, "sliders");
        motorArm = hardwareMap.get(DcMotor.class, "elbow");
        motorHanging = hardwareMap.get(DcMotor.class, "hanging");
        servoCL = hardwareMap.get(Servo.class, "claw_left");
        servoCR = hardwareMap.get(Servo.class, "claw_right");
        servoDrone = hardwareMap.get(Servo.class, "done");
    }

}
