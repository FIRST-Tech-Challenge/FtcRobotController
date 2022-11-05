/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo arm_left;
    private Servo arm_right;

    Intake(HardwareMap hardwareMap, Telemetry telemetry){
        arm_left = hardwareMap.get(Servo.class, "arm_left");
        arm_right = hardwareMap.get(Servo.class, "arm_right");
    }

    public void moveArmPosition(double pos){
        arm_left.setPosition(pos);
        arm_right.setPosition(1-pos);
    }


}
*/