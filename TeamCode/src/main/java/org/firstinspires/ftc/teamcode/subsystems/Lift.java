package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

    /**
     * Primary constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize motors.
     * @param nameLift [String] Name of the lift motor assigned in the configuration.
     */
    public Lift(HardwareMap hw, String nameLift){
        lift = hw.get(DcMotor.class, nameLift);
    }
    public void moveLift(double power){
        lift.setPower(power);
    }
//    public int getPosition(){
//        return encoder.getPositionAndVelocity().position;
//    }
}
