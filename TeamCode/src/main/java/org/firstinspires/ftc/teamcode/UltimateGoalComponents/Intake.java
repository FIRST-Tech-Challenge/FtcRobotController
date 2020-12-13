package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class Intake extends RobotComponent {

    DcMotor flyWheelShooter;

    public Intake(robotBase BASE) {
        super(BASE);

        flyWheelShooter = base.getMapper().mapMotor("flyWheel");
    }

    public void suck(double power){
        flyWheelShooter.setPower(power);
    }
    @Override
    public void stop() {
        flyWheelShooter.setPower(0);
    }
}
