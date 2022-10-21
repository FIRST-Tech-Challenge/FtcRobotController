package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.INTAKING;
import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.OUTTAKING;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ArmSystem;

@Autonomous (name = "beambreaking", group = "Autonomous")
public class BeamBreakerOpMode extends OpMode {

    private ElapsedTime time;
    private ArmSystem.Intake intake;

    /** Initialization */
    public void init() {
        time = new ElapsedTime();
        DigitalChannel beambreaker = hardwareMap.get(DigitalChannel.class, "beambreaker");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intake = new ArmSystem.Intake(intakeMotor, beambreaker);
    }

    public void loop(){

        if(gamepad1.a || intake.getState() == INTAKING){
            intake.intake();
        }

        if(gamepad1.b || intake.getState() == OUTTAKING){
            intake.outtake();
        }

    }

}

