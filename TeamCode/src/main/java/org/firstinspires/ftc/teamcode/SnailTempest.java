package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class SnailTempest  extends OpMode {
    
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    DcMotor LiftLeft;
    DcMotor LiftRight;
    Double Lift_power = 0.84;

    Servo ServoLeft;
    Servo ServoRight;

    Servo ServoHingeLeft;
    Servo ServoHingeRight;

    Servo ServoDump;

    public void init(HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");

        ServoHingeLeft = hardwareMap.get(Servo.class, "HingeLeft");
        ServoHingeRight = hardwareMap.get(Servo.class, "HingeRight");

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        LiftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
        LiftRight = hardwareMap.get(DcMotor.class, "LiftRight");
        LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);


        ServoLeft = hardwareMap.get(Servo.class, "ServoClawLeft");
        ServoRight = hardwareMap.get(Servo.class, "ServoClawRight");

        ServoDump = hardwareMap.get(Servo.class, "ServoDump");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}







