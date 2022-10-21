package org.firstinspires.ftc.teamcode.Functions;

import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;

public class MoveDirectJoystick {

    Move move;
    RotationDetector rotationDetector;

    public MoveDirectJoystick(Move _MV, RotationDetector _rotationDetector) {
        move = _MV;
        rotationDetector = _rotationDetector;
    }

    // o sa ne mutam direct diagonal, putem seta un offset mai tarziu
    public String MoveDirect(double x, double y){

        move.MoveOne(1, -y);
        move.MoveOne(4, -y);
        move.MoveOne(2, -x);
        move.MoveOne(3, x);
//        leftMotor.setPower(-power);
//        leftMotorBack.setPower(-power);
//        rightMotorBack.setPower(power);
//        rightMotor.setPower(power);
        // 1 - front left, 2 - back left, 3 - front right, 4 - back right

        return "Move Direct\n" +
                        "move.MoveOne(1, "+y+") = " +move.ReadMotor(1) +"\n"+
                        "move.MoveOne(4, "+y+") = " +move.ReadMotor(4) +"\n"+
                        "move.MoveOne(2, "+x+") = " +move.ReadMotor(2) +"\n"+
                        "move.MoveOne(3, "+x+") = " +move.ReadMotor(3);


    }


    // calculat cu offset la stanga
    public String MoveSimple(double x, double y){
        double Offset=Math.PI/4;
        MVVariables.Vector2 Joystick = new MVVariables.Vector2(x, y);
        Joystick.Rotate(Offset);

        return "Joystick.RotateRadians("+Offset+")="+Joystick.RotateRadians(Offset)+
                " MoveSimple with MVVariables.Vector2.Rotate\n" +

                "move.MoveOne(1, Joystick.y="+Joystick.y+") = " +move.ReadMotor(1) +"\n"+
                "move.MoveOne(4, y="+y+") = " +move.ReadMotor(4) +"\n"+
                "move.MoveOne(2, Joystick.x="+Joystick.x+") = " +move.ReadMotor(2) +"\n"+
                "move.MoveOne(3, x="+x+") = " +move.ReadMotor(3)+


                "\n"+MoveDirect(Joystick.x, Joystick.y);
    }

    // move simple dar cu calculare de ofset dinamic
    public String MoveAuto(float x, float y){
        double Offset=-Math.PI/4;
        MVVariables.Vector2 Joystick = new MVVariables.Vector2(x, y);
        Joystick.Rotate(Offset);

        return "Joystick.RotateRadians("+Offset+")="+Joystick.RotateRadians(Offset)+
                "MoveAuto with "+ rotationDetector.ReturnPositiveRotation() +"\n" +

                "move.MoveOne(1, Joystick.y="+Joystick.y+") = " +move.ReadMotor(1) +"\n"+
                "move.MoveOne(4, y="+y+") = " +move.ReadMotor(4) +"\n"+
                "move.MoveOne(2, Joystick.x="+Joystick.x+") = " +move.ReadMotor(2) +"\n"+
                "move.MoveOne(3, x="+x+") = " +move.ReadMotor(3)+

                "\n"+MoveDirect(Joystick.x, Joystick.y);
    }





}























// OLD CALCULATIONS
/* suma |x|+|y|=1 intotdeauna, putem sa folosim asta.
Ca sa il mutam pe A la dreapta pana la (0, 1) x creste la 0 iar y creste la 1
Ca sa il mutam pe B la stanga pana la (0, 1) x scade la 0 iar y creste la 1
daca adaugam sa zicem 0.5, tot ce e peste 1 trebuie scazut

0, 1 + 0.5 = 0.5, 1.5-1 = 0.5, 0.5
(0.5 0.5) - 0.5 = 0, 0


                              (0,1)
              A(-0.5, 0.5)   |    B(0.5, 0.5)
                        |\   |   /|
                        | \  |  / |
                        |  \ | /  |
           (-1, 0)      |   \|/   |           (1, 0)
             ------------->>-*-<<--------------
                             | mut axa y cu 0.5
                             |
                             |
                             |
                             |
                              (0,-1)
 */
