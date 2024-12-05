package org.firstinspires.ftc.teamcode.subsystems;




import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;


public class DriveSubsystem{

    protected DcMotor left_motor;
    protected DcMotor right_motor;

    public void init( HardwareMap hwMap) {
        left_motor = hwMap.dcMotor.get("left_motor");
        right_motor = hwMap.dcMotor.get("right_motor");
//        test_servo = hwMap.servo.get("test_servo");

        left_motor.setDirection(DcMotor.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Drive(double drive, double turn) {
        double leftPower = drive - turn;
        double rightPower = drive + turn;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower = leftPower / max;
            rightPower = rightPower / max;
        }

        left_motor.setPower(leftPower);
        right_motor.setPower(rightPower);
    }


}
