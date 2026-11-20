package org.firstinspires.ftc.teamcode.BioBuzzTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.Intake;

@TeleOp
public class IntakeTesting extends OpMode  {
    Intake intake=new Intake();

    @Override
    public void init() {
        intake.init(hardwareMap);

    }
    @Override
    public void loop(){
        int a=1;
        intake.setNectarServoSpeed(-1*a);
        intake.setPollenServoSpeed(1 * a);
        intake.setbackIntakeMotorSpeed(-1 * a);
        intake.setFrontIntakeMotorSpeed(-1*a);
        if(gamepad1.a){
            a *= -1;
        }
    }
}
