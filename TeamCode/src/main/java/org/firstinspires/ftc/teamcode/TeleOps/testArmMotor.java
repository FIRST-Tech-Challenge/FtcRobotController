package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple; //hello
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.OdoPod;
import org.firstinspires.ftc.teamcode.MechanismTemplates.PoleAlignment;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.SignalEdgeDetector;

@Config
@TeleOp(name = "testArmMotor")
public class testArmMotor extends OpMode {
    private Motor armMotor;

    @Override
    public void init() {
        armMotor = new Motor(hardwareMap, "ARM", Motor.GoBILDA.RPM_84);
    }

    @Override
    public void loop() {
        while(gamepad1.a){
         armMotor.set(0.5);
        }

        while(gamepad1.b){
            armMotor.set(-0.5);
        }
    }
}