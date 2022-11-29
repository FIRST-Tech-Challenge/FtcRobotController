package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestMotorSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final DcMotorSimple mTestMotor;
    private int mCompteur;


    private double mGetAscenseur1;
    private double mGetAscenseur2;
    private double mSetAscenseur;


    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    // Pour suivre la position sur le terrain. Donnée par Vuforia.
    private double mPositionX = 0;
    private double mPositionY = 0;
    private double mRotationZ = 0;

    public TestMotorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mTestMotor = mHardwareMap.get(DcMotorSimple.class,"TestMotor");



       // mSetAscenseur = mMoteurAscenseur1.setTargetPosition();
//setpositionAscenseur = mMoteurAscenseur1.setposition et mMoteurAScenseur2.setposition (THÉORIQUE)
    }


    @Override
    public void periodic() {

    }


    public void stop() {
        mTestMotor.setPower(0);
    }
    public void test() {
        mTestMotor.setPower(1.0);
    }

}

