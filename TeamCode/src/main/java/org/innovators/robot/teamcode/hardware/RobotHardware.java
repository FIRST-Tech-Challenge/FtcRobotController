package org.innovators.robot.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor armExtensionMotor = null;
    public DcMotor geckoWheelMotor = null;
    public Servo armServo = null;
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Two motor wheel driving
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        armServo = hwMap.get(Servo.class, "arm_servo");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // arm, armExtension and gecko wheel
        armServo = hwMap.get(Servo.class, "arm_servo");
        armExtensionMotor = hwMap.get(DcMotor.class, "arm_extension_motor");
        geckoWheelMotor = hwMap.get(DcMotor.class, "gecko_wheel_motor");

        armServo.setPosition(0.0);
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        geckoWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

