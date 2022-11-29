package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;

public class BrasSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final DcMotor mMoteurAscenseur1;
    private final DcMotor mMoteurAscenseur2;


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

    public BrasSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mMoteurAscenseur1 = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur 1");
        mMoteurAscenseur2 = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur 2");

        mMoteurAscenseur1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurAscenseur1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mMoteurAscenseur2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurAscenseur2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        mGetAscenseur1 = mMoteurAscenseur1.getCurrentPosition();
        mGetAscenseur2 = mMoteurAscenseur2.getCurrentPosition();

       // mSetAscenseur = mMoteurAscenseur1.setTargetPosition();
//setpositionAscenseur = mMoteurAscenseur1.setposition et mMoteurAScenseur2.setposition (THÉORIQUE)
    }


    @Override
    public void periodic() {

    }


    public void stop() {
        mMoteurAscenseur1.setPower(0);
        mMoteurAscenseur2.setPower(0);
    }

    public void FonctionAscenseur(int positionAscenseur) {

        switch (positionAscenseur) {
            case 1 :
            case 2 :
            case 3 :
        }

    }
}

