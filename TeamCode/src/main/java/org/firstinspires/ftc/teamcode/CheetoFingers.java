package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class CheetoFingers {

    DcMotorEx RM;
    DcMotorEx LM;
    ServoImplEx FINGER_RS;
    ServoImplEx FINGER_LS;
    ServoImplEx Forearm_RS;
    ServoImplEx Forearm_LS;

    double armJoint;

    CheetoFingers(HardwareMap hm){

        RM = hm.get(DcMotorEx.class, "armMotor");
        LM = hm.get(DcMotorEx.class, "Odometry_Pod_Right");
        FINGER_RS = hm.get(ServoImplEx.class, "Right_Servo_Finger");
        FINGER_LS = hm.get(ServoImplEx.class, "Left_Servo_Finger");
        Forearm_LS = hm.get(ServoImplEx.class, "Left_Servo_Forearm");
        Forearm_RS = hm.get( ServoImplEx.class, "Right_Servo_Forearm");

        FINGER_LS.setDirection(ServoImplEx.Direction.REVERSE);

        RM.setDirection(DcMotorEx.Direction.REVERSE);
        LM.setDirection(DcMotorEx.Direction.REVERSE);

        RM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armJoint = 0.25;

    }

    public void gamepadInputsArm(Gamepad gmpad){

        RM.setPower(gmpad.right_trigger);
        LM.setPower(gmpad.right_trigger);

        RM.setPower(-gmpad.left_trigger);
        LM.setPower(-gmpad.left_trigger);

        if(gmpad.left_bumper){

            if(armJoint < 0.5) {
                armJoint += 0.025;
            }

        }
        if(gmpad.right_bumper){

            if(armJoint > 0) {
                armJoint -= 0.010;
            }

        }

        FINGER_RS.setPosition(armJoint);
        FINGER_LS.setPosition(armJoint);

    }




}
