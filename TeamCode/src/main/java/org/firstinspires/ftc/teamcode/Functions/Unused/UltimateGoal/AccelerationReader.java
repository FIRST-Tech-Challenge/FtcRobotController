package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;
import org.firstinspires.ftc.teamcode.Functions.Move;

import java.io.IOException;

@Deprecated
public class AccelerationReader {



    /* ========================
    *      TEMPLATE CLASA
    * ========================
    * CLASA ASTA SE FOLOSESTE ASTFEL:
    *
    accelerationReader.Reset(directie, distanta)
    do
    {
      sleep(accelerationReader.Go());
    }while(accelerationReader.IsNotDone());
    *
     */



    AccelerationDetector accelerationDetector;
    Move move;

    public AccelerationReader(BNO055IMU _Gyro, Move _move) throws IOException {
        accelerationDetector = new AccelerationDetector(_Gyro);
        move=_move;
    }

    int CurrentDirection=-1;
    double CurrentDistance=-1;
    int PassedDistance=-2;

//se ruleaza inainte de functia go()
    public void Reset(int _Direction, double _Distance){
        CurrentDirection=_Direction;
        CurrentDistance=_Distance;
        PassedDistance=0;
    }

    // functia asta face urmatoarele chestii:
    //
    // - returneaza 1 milisecunde cand e gata si 10 milisecunda cand mai are de lucru
    // - calculeaza distanta parcursa dinamic in comparatie cu directia pe care o ia
    // - se baga intr-un sleep intr-un while ca sa nu avem probleme cu crashuri
    public long Go(){
            if(PassedDistance<CurrentDistance) {
                move.MoveRaw(CurrentDirection, 0.1);
                switch (CurrentDirection) {
                    case 1:
                    case 2:
                        PassedDistance += accelerationDetector.realAccel.x;
                        break;
                    case 3:
                    case 4:
                        PassedDistance += accelerationDetector.realAccel.y;
                        break;
                }
                return 10;
            }
            else{
                move.MoveStop();
                return 1;
            }

    }

    public boolean IsNotDone(){
        return PassedDistance<CurrentDistance;
    }

    public String ReturnData(){
        return getClass().getName() + " - CurrentDirection: " + CurrentDirection + "\n  CurrentDistance: "
                + CurrentDistance + "\n PassedDistance: " + PassedDistance + "\n IsNotDone: " + IsNotDone();
    }

}
