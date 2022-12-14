package org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.MV.MVTurnTowardsPoint;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;


// facut de Vlad

// intoarce robotul catre destinatie
// duce robotul spre destinatie
@Deprecated
public class SistemXY {



    PositionCalculator positionCalculator;
    MVTurnTowardsPoint MVTurnTowardsPoint;
    RotationDetector rotationDetector;
    Move move;
    Rotate rotate;
    //MoveAutocorrectTest moveAutocorrect;
    Telemetry telemetry;


    public SistemXY(PositionCalculator _PositionCalculator, MVTurnTowardsPoint _MV_TurnTowardsPoint,
                    RotationDetector _RotationDetector, Move _Move, Rotate _Rotate, Telemetry _telemetry)
    {
        positionCalculator = _PositionCalculator;
        MVTurnTowardsPoint = _MV_TurnTowardsPoint;
        rotationDetector = _RotationDetector;
        move = _Move;
        rotate = _Rotate;
        //moveAutocorrect = new MoveAutocorrectTest(_RotationDetector, _Move);
        telemetry = _telemetry;

    }

    void GoTo(MVVariables.Vector2 Destination){
        int Unghi = (int) MVTurnTowardsPoint.AngleCalculator(Destinatie, positionCalculator.Position, rotationDetector.ReturnPositiveRotation());
        while(rotationDetector.WaitForRotation(Unghi))
        {
            telemetry.addLine("Unghi:"+Unghi);
            TelemetryData();
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop(); // aceasta linie e doar de siguranta.
        // verificam care distanta e mai mare; spre x sau spre y si doar atunci cand atinge distanta aia se va opri

        while((Math.abs(Destinatie.x - positionCalculator.Position.x)) >= 0.01){
            if((Math.abs(Destinatie.x - positionCalculator.Position.x)) >= 0.025) {
                move.MoveFull(1);
                TelemetryData();

            }
            else{
                move.MoveRaw(1, 0.2);
                TelemetryData();

            }

        }
        move.MoveStop();
    }

    void TelemetryData(){
        telemetry.addLine("Current Angle: "+ GenerateAngle());
        GivePosition(positionCalculator.Position);
        //dataLogger.writeData(getRuntime());
        //telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
        telemetry.addLine("Current Rotation: "+ rotationDetector.ReturnPositiveRotation());
        telemetry.update();
    }


    boolean Rotation=true;
    MVVariables.Vector2 Destinatie;
    MVVariables.Vector2 PozitieCurenta;
    int Unghi;

    public void Reset(){
        Rotation = true;
        Destinatie = new MVVariables.Vector2(0, 0);
        PozitieCurenta = positionCalculator.Position;
        Unghi=0;
    }

    public void GivePosition(MVVariables.Vector2 _Pos){
        PozitieCurenta = _Pos;

    }

    public boolean IsX(){
        return (Math.abs(Destinatie.x - PozitieCurenta.x) > 0.08);
    }

    public boolean IsY(){
        return (Math.abs(Destinatie.y - PozitieCurenta.y) > 0.08);
    }

    public void SetDestination(double x, double y){
        Destinatie = new MVVariables.Vector2(x, y);
        //PozitieCurenta = new Variables.Vector2(positionCalculator.Position.x, positionCalculator.Position.y);
        GenerateAngle();
    }

    public int GenerateAngle(){
        return Unghi = (int) MVTurnTowardsPoint.AngleCalculator(Destinatie, PozitieCurenta, rotationDetector.ReturnPositiveRotation());
    }

    public boolean RotateToDestination(){
        if (rotationDetector.WaitForRotation(Unghi)) {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
            return true;
        } else {
            rotate.MoveStop();
            return false;
        }

    }

    public boolean GoTowardsDestinationX(){
        if (!(Math.abs(Destinatie.x - positionCalculator.Position.x) <= 0.05)) {

            //positionCalculator.NewUpdate();
            //moveAutocorrect.GiveStartingAngle(Unghi);
            move.MoveFull(1);
            return true;
        }
        else {
            move.MoveStop();
            return false;
        }
    }

    public boolean GoTowardsDestinationY(){
        if(!(Math.abs(Destinatie.y - positionCalculator.Position.y) <= 0.05)){
            //moveAutocorrect.GiveStartingAngle(Unghi);
            move.MoveFull(1);
            return true;

        }
        else {
            move.MoveStop();
            return false;
        }
    }


    // roteste robotul catre punct, iar cand e directionat spre destinatie, porneste spre destinatie si se opreste cat mai aproape de ea
    public boolean GoTo(double x, double y){
        SetDestination(x, y);
        if(Rotation) {
            if (rotationDetector.WaitForRotation(Unghi)) {
                rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
                Rotation = true;
            } else {
                rotate.MoveStop();
                Rotation = false;
            }
            return false;
        }
        else {
            if (!(Math.abs(Destinatie.x - positionCalculator.Position.x) <= 0.05)) {

                //positionCalculator.NewUpdate();
                //moveAutocorrect.GiveStartingAngle(Unghi);
                move.MoveFull(1);
                return false;
            }
            //else if(!(Math.abs(Destinatie.y - positionCalculator.Position.y) <= 0.05)){
            //moveAutocorrect.GiveStartingAngle(Unghi);
            //move.MoveFull(1);
            //return false;
            //}
            else {
                move.MoveStop();
                return true;
            }
        }

        //Rotation=true;
        //return true;

    }

    // returneaza toate datele pentru debug
    public String TestData(){
        return "Pozitie curenta: "+ PozitieCurenta.ReturnData() + "\n" + "Destinatie: "+ Destinatie.ReturnData();
    }

}
