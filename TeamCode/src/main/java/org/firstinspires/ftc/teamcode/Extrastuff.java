package org.firstinspires.ftc.teamcode;

public class Extrastuff {
}
///////////////////////////////////////////

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(2300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(1900);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(100);

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(2590);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(markerPosition == 3) {
            robot.liftLeft.setTargetPosition(-270);
            robot.liftRight.setTargetPosition(-270);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(400);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 2){

            robot.liftLeft.setTargetPosition(-250);
            robot.liftRight.setTargetPosition(-250);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }else if(markerPosition == 1){
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 1){
            robot.liftLeft.setTargetPosition(-564);
            robot.liftRight.setTargetPosition(-564);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(60);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
        }

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1030);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(markerPosition == 3) {
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(730);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 2){

            robot.liftLeft.setTargetPosition(-250);
            robot.liftRight.setTargetPosition(-250);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }else if(markerPosition == 1){
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 1){
            robot.liftLeft.setTargetPosition(-564);
            robot.liftRight.setTargetPosition(-564);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(60);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
        }


        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(300);*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//new redduck spin

        /*robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftPosition -= 250;
        frontRightPosition += 250;
        backLeftPosition += 250;
        backRightPosition -= 250;
        //going out from wall

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);

//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.frontLeft.setPower(0);


        frontLeftPosition += 600;
        frontRightPosition -= 600;
        backLeftPosition += 600;
        backRightPosition -= 600;
        //turing

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        robot.frontLeft.setPower(0.30);
//        robot.frontRight.setPower(0.30);
//        robot.backLeft.setPower(0.30);
//        robot.backRight.setPower(0.30);

        sleep(1200);

//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.frontLeft.setPower(0);

        frontLeftPosition += 850;
        frontRightPosition -= 850;
        backLeftPosition -= 850;
        backRightPosition += 850;
        //going toward the spinning


        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        robot.frontLeft.setPower(0.30);
//        robot.frontRight.setPower(0.30);
//        robot.backLeft.setPower(0.30);
//        robot.backRight.setPower(0.30);

        sleep(2500);

//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.frontLeft.setPower(0);

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        frontLeftPosition -= 800;
        frontRightPosition += 800;
        backLeftPosition += 800;
        backRightPosition -= 800;
        //going back to the square


        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        robot.frontLeft.setPower(0.30);
//        robot.frontRight.setPower(0.30);
//        robot.backLeft.setPower(0.30);
//        robot.backRight.setPower(0.30);

        sleep(1200);

//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.frontLeft.setPower(0);

        frontLeftPosition += 350;
        frontRightPosition += 350;
        backLeftPosition += 350;
        backRightPosition += 350;
        //going backward


        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        robot.frontLeft.setPower(0.30);
//        robot.frontRight.setPower(0.30);
//        robot.backLeft.setPower(0.30);
//        robot.backRight.setPower(0.30);

        sleep(1200);

//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.frontLeft.setPower(0);

       frontLeftPosition -= 850;
       frontRightPosition += 850;
       backLeftPosition -= 850;
       backRightPosition += 850;
       //turning to the shipping hub


       robot.frontLeft.setTargetPosition(frontLeftPosition);
       robot.frontRight.setTargetPosition(frontRightPosition);
       robot.backLeft.setTargetPosition(backLeftPosition);
       robot.backRight.setTargetPosition(backRightPosition);


       robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//       robot.frontLeft.setPower(0.30);
//       robot.frontRight.setPower(0.30);
//       robot.backLeft.setPower(0.30);
//       robot.backRight.setPower(0.30);

       sleep(2500);

//       robot.frontRight.setPower(0);
//       robot.backLeft.setPower(0);
//       robot.backRight.setPower(0);
//       robot.frontLeft.setPower(0);

       robot.liftLeft.setTargetPosition(-290);
       robot.liftRight.setTargetPosition(-290);
       robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


       robot.liftLeft.setPower(0.20);
       robot.liftRight.setPower(0.20);

       sleep(2020);

       frontLeftPosition -= 290;
       frontRightPosition -= 290;
       backLeftPosition -= 290;
       backRightPosition -= 290;
       //moving more toward the shipping hub


       robot.frontLeft.setTargetPosition(frontLeftPosition);
       robot.frontRight.setTargetPosition(frontRightPosition);
       robot.backLeft.setTargetPosition(backLeftPosition);
       robot.backRight.setTargetPosition(backRightPosition);


       robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//       robot.frontLeft.setPower(0.30);
//       robot.frontRight.setPower(0.30);
//       robot.backLeft.setPower(0.30);
//       robot.backRight.setPower(0.30);

       sleep(1200);

//       robot.frontRight.setPower(0);
//       robot.backLeft.setPower(0);
//       robot.backRight.setPower(0);
//       robot.frontLeft.setPower(0);

       robot.gatherServo.setPower(0.4);
       sleep(2200);
       robot.gatherServo.setPower(0);

       sleep(2200);

        frontLeftPosition += 270;
        frontRightPosition -= 270;
        backLeftPosition += 270;
        backRightPosition -= 270;
        //turing back for the wall


        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1200);

        robot.liftLeft.setTargetPosition(0);
        robot.liftRight.setTargetPosition(0);

        frontLeftPosition += 885;
        frontRightPosition += 885;
        backLeftPosition += 885;
        backRightPosition += 885;
        //going back


        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//red duck spin with encoders
        /*frontLeftPosition -= 120;
        frontRightPosition += 120;
        backLeftPosition += 120;
        backRightPosition -= 120;
        //going out from wall

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);

        frontLeftPosition -= 600;
        frontRightPosition -= 600;
        backLeftPosition -= 600;
        backRightPosition -= 600;
        //moving toward the spinner

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.25);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.backRight.setPower(0.25);

        sleep(1500);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        frontLeftPosition -= 730;
        frontRightPosition += 730;
        backLeftPosition += 730;
        backRightPosition -= 730;
        //moving sideways to the box

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);


        frontLeftPosition += 420;
        frontRightPosition += 420;
        backLeftPosition += 420;
        backRightPosition += 420;
        //going backwards toward the shipping hub

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);

        frontLeftPosition -= 1150;
        frontRightPosition += 1150;
        backLeftPosition -= 1150;
        backRightPosition += 1150;
        //spinning toward the shipping

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);

        robot.liftLeft.setTargetPosition(-290);
        robot.liftRight.setTargetPosition(-290);
        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.liftLeft.setPower(0.20);
        robot.liftRight.setPower(0.20);

        sleep(2020);


       robot.liftLeft.setTargetPosition(-290);
       robot.liftRight.setTargetPosition(-290);
       robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


       robot.liftLeft.setPower(0.20);
       robot.liftRight.setPower(0.20);

       sleep(2020);

       frontLeftPosition -= 430;
       frontRightPosition -= 430;
       backLeftPosition -= 430;
       backRightPosition -= 430;
       //going foward toward the shipping hub

       robot.frontLeft.setTargetPosition(frontLeftPosition);
       robot.frontRight.setTargetPosition(frontRightPosition);
       robot.backLeft.setTargetPosition(backLeftPosition);
       robot.backRight.setTargetPosition(backRightPosition);

       robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       robot.frontLeft.setPower(0.30);
       robot.frontRight.setPower(0.30);
       robot.backLeft.setPower(0.30);
       robot.backRight.setPower(0.30);

       sleep(1200);

       robot.gatherServo.setPower(0.4);
       sleep(2200);
       robot.gatherServo.setPower(0);

       robot.liftLeft.setTargetPosition(0);
       robot.liftRight.setTargetPosition(0);

       frontLeftPosition -= 150;
       frontRightPosition += 150;
       backLeftPosition -= 150;
       backRightPosition += 150;
       //turing to go into red square

       robot.frontLeft.setTargetPosition(frontLeftPosition);
       robot.frontRight.setTargetPosition(frontRightPosition);
       robot.backLeft.setTargetPosition(backLeftPosition);
       robot.backRight.setTargetPosition(backRightPosition);

       robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       robot.frontLeft.setPower(0.30);
       robot.frontRight.setPower(0.30);
       robot.backLeft.setPower(0.30);
       robot.backRight.setPower(0.30);

       sleep(1500);

       frontLeftPosition += 1000;
       frontRightPosition += 1000;
       backLeftPosition += 1000;
       backRightPosition += 1000;
       //going backwards to go into red square

       robot.frontLeft.setTargetPosition(frontLeftPosition);
       robot.frontRight.setTargetPosition(frontRightPosition);
       robot.backLeft.setTargetPosition(backLeftPosition);
       robot.backRight.setTargetPosition(backRightPosition);

       robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       robot.frontLeft.setPower(0.30);
       robot.frontRight.setPower(0.30);
       robot.backLeft.setPower(0.30);
       robot.backRight.setPower(0.30);

       sleep(2000);*/






















        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1030);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        if(markerPosition == 3) {
            robot.liftLeft.setTargetPosition(-290);
            robot.liftRight.setTargetPosition(-290);
            robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.liftLeft.setPower(0.20);
            robot.liftRight.setPower(0.20);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(350);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 2){

            robot.liftLeft.setTargetPosition(-250);
            robot.liftRight.setTargetPosition(-250);
            robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }else if(markerPosition == 1){
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);
            robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 1){
            robot.liftLeft.setTargetPosition(-564);
            robot.liftRight.setTargetPosition(-564);
            robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(60);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
//Blue Duck Spinner 2.0
/*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1030);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(markerPosition == 3) {
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(730);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 2){

            robot.liftLeft.setTargetPosition(-250);
            robot.liftRight.setTargetPosition(-250);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }else if(markerPosition == 1){
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 1){
            robot.liftLeft.setTargetPosition(-564);
            robot.liftRight.setTargetPosition(-564);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(60);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
        }


        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(300);*/

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(2300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.25);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(-0.25);
        robot.backRight.setPower(0.25);
        sleep(3300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(100);

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(2500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(markerPosition == 3) {
            robot.liftLeft.setTargetPosition(-290);
            robot.liftRight.setTargetPosition(-29 0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(400);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 2){

            robot.liftLeft.setTargetPosition(-250);
            robot.liftRight.setTargetPosition(-250);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }else if(markerPosition == 1){
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 1){
            robot.liftLeft.setTargetPosition(-564);
            robot.liftRight.setTargetPosition(-564);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(60);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
        }

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Red Duck Spin

 /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.3);
        sleep(720);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(-0.3);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);










        /*if(updatedRecognitions.size() >= 1) {
            //tfod.setZoom(1.0, 16 / 19);
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
            currenttime = runtime.seconds();
            while(opModeIsActive() && (runtime.seconds() - currenttime < 2)){
                telemetry.addData("before", "listupdate");
                telemetry.update();
                sleep(1000);
                updatedRecognitions = tfod.getUpdatedRecognitions();
                telemetry.addData("after","listupdate");
                telemetry.update();
                sleep(1000);

                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
            }
            if(updatedRecognitions.size() >= 1) {
                markerPosition = 2;
                telemetry.addData("Duck in", "number 2");
                telemetry.update();
            } else {
                markerPosition = 1;
                telemetry.addData("Duck in", "number 1");
                telemetry.update();
            }

        } else {
            markerPosition = 3;
            telemetry.addData("Duck in", "number 3");
            telemetry.update();
        }sleep(5000);

        /*if(markerPosition == 3){
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(650);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        if(markerPosition == 1) {
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(10);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        } else if(markerPosition == 2){
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(150);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(20);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }else {
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(100);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(30);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }

        robot.gatherServo.setPower(0.25);
        sleep(100);
        robot.gatherServo.setPower(0);

        if(markerPosition == 3){
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        if(markerPosition == 1) {
            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(10);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        } else if(markerPosition == 2){
            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(20);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(150);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        }else {

            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(30);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(100);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        }*/












        /*if(markerPosition == 2) {
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep();
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//this makes it go foward if it reconizes the maker/duck
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(650);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
//this turns it
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(50);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//makes it go foward
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.5);
            robot.liftRight.setPower(0.5);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

        } else if(markerPosition == 1) {
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(5000);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//moves forward if it finds marker in 3
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(650);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            robot.liftLeft.setPower(-0.3);
            robot.liftRight.setPower(-0.3);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

        }else if(markerPosition == 3){
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(100);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(50);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            //if it does not spot it in 3 it proceds like it would if its at 1
        }*\
         */





//robot.gatherServo.setPower(0.5);
//sleep(1200);
//robot.gatherServo.setPower(0);


        /*}else if(updatedRecognitions.size() == 0) {
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(-TURN_SPEED);
            Trobot.backLeft.setPower(TURN_SPEED);
            sleep(650);*\

         */


//robot.frontLeft.setPower(TURN_SPEED);
//robot.frontRight.setPower(-TURN_SPEED);
//robot.backRight(-TURN_SPEED);
//robot.backLeft(TURN_SPEED);
//sleep()

//robot.frontLeft.setPower(-TURN_SPEED)
//robot.frontRight.setPower(TURN_SPEED)
//robot.backRight.setPower(-TURN_SPEED)
//robot.backLeft.setPower(TURN_SPEED)
//sleep()

//robot.frontLeft.setPower(TURN_SPEED)
//robot.frontRight.setPower(TURN_SPEED)
//robot.backRight.setPower(TURN_SPEED)
//robot.backLeft.setPower(TURN_SPEED);


// Step 1:  Drive forward for 3 seconds
//robot.frontLeft.setPower(FORWARD_SPEED);
//robot.frontRight.setPower(FORWARD_SPEED);
//robot.backLeft.setPower(FORWARD_SPEED);
//robot.backRight.setPower(FORWARD_SPEED);
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        robot.frontLeft.setPower(TURN_SPEED);
        robot.frontRight.setPower(-TURN_SPEED);
        robot.backRight.setPower(-TURN_SPEED);
        robot.backLeft.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        robot.frontLeft.setPower(-FORWARD_SPEED);
        robot.frontRight.setPower(-FORWARD_SPEED);
        robot.backRight.setPower(-FORWARD_SPEED);
        robot.backLeft.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);*/


//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f ;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 320;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Test Auto Stuff

//red warhouse
        /*robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1300);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(-TURN_SPEED);
        robot.backLeft.setPower(TURN_SPEED);
        sleep(2400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(375);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/
// blue warhouse
        /*robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1300);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(TURN_SPEED);
        robot.frontRight.setPower(-TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(2400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(375);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/

//Duck spinning red
        /*robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(560);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(750);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(0.40);
        robot.frontRight.setPower(-0.40);
        robot.backLeft.setPower(-0.40);
        robot.backRight.setPower(0.40);
        sleep(1020);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
//new stuff
        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(800);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.shuteServo.setPower(0.3);
        sleep(1200);
        robot.shuteServo.setPower(0);

        robot.gatherServo.setPower(-0.4);
        sleep(2000);
        robot.gatherServo.setPower(0);

        robot.shuteServo.setPower(-0.3);
        sleep(1220);
        robot.shuteServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(800);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*\

         */
//go left
        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //go forward
        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(940);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(1100);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(420);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(950);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(2300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.shuteServo.setPower(-0.3);
        sleep(2360);
        robot.shuteServo.setPower(0);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


        robot.shuteServo.setPower(0.3);
        sleep(2360);
        robot.shuteServo.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(1500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);



       /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(600);
        robot.frontLeft.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1100);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.shuteServo.setPower(-0.3);
        sleep(1510);
        robot.shuteServo.setPower(0);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.shuteServo.setPower(0.3);
        sleep(1510);
        robot.shuteServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);





        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1120);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);





        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(960);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.25);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.backRight.setPower(0.25);
        sleep(420);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1120);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

         robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);









        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.3);
        sleep(720);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(-0.3);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/
//this is where you uncome
        /*robot.frontLeft.setPower(-0.30);
        sleep(3000);
        robot.frontLeft.setPower(0);

        robot.frontLeft.setPower(0.30);
        sleep(3000);
        robot.frontLeft.setPower(0);

        robot.frontRight.setPower(-0.30);
        sleep(3000);
        robot.frontRight.setPower(0);

        robot.frontRight.setPower(0.30);
        sleep(3000);
        robot.frontRight.setPower(0);

        robot.backLeft.setPower(-.30);
        sleep(3000);
        robot.backLeft.setPower(0);

        robot.backLeft.setPower(.30);
        sleep(3000);
        robot.backLeft.setPower(0);

        robot.backRight.setPower(-0.3);
        sleep(3000);
        robot.backRight.setPower(0);

        robot.backRight.setPower(0.3);
        sleep(3000);
        robot.backRight.setPower(0);

        robot.spinServo.setPower(0.5);
        sleep(3000);
        robot.backRight.setPower(0);

        robot.liftRight.setPower(-0.5);
        robot.liftLeft.setPower(-0.5);
        sleep(3000);
        robot.liftRight.setPower(0);
        robot.liftLeft.setPower(0);

        robot.liftRight.setPower(0.5);
        robot.liftLeft.setPower(0.5);
        sleep(3000);
        robot.liftRight.setPower(0);
        robot.liftLeft.setPower(0);*/
//This is where to uncoment











//wtf
        /*if(updatedRecognitions.size() >= 1) {
            //tfod.setZoom(1.0, 16 / 19);
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
            currenttime = runtime.seconds();
            while(opModeIsActive() && (runtime.seconds() - currenttime < 2)){
                telemetry.addData("before", "listupdate");
                telemetry.update();
                sleep(1000);
                updatedRecognitions = tfod.getUpdatedRecognitions();
                telemetry.addData("after","listupdate");
                telemetry.update();
                sleep(1000);

                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
            }
            if(updatedRecognitions.size() >= 1) {
                markerPosition = 2;
                telemetry.addData("Duck in", "number 2");
                telemetry.update();
            } else {
                markerPosition = 1;
                telemetry.addData("Duck in", "number 1");
                telemetry.update();
            }

        } else {
            markerPosition = 3;
            telemetry.addData("Duck in", "number 3");
            telemetry.update();
        }sleep(5000);

        /*if(markerPosition == 3){
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(650);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        if(markerPosition == 1) {
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(10);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        } else if(markerPosition == 2){
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(150);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(20);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }else {
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(100);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(30);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }

        robot.gatherServo.setPower(0.25);
        sleep(100);
        robot.gatherServo.setPower(0);

        if(markerPosition == 3){
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        if(markerPosition == 1) {
            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(10);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        } else if(markerPosition == 2){
            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(20);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(150);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        }else {

            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(30);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(100);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        }*/

        /*robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.frontRight.setPower(0.5);
        sleep(30000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);*/











        /*if(markerPosition == 2) {
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep();
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//this makes it go foward if it reconizes the maker/duck
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(650);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
//this turns it
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(50);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//makes it go foward
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.5);
            robot.liftRight.setPower(0.5);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

        } else if(markerPosition == 1) {
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(5000);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//moves forward if it finds marker in 3
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(650);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            robot.liftLeft.setPower(-0.3);
            robot.liftRight.setPower(-0.3);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

        }else if(markerPosition == 3){
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(100);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(50);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            //if it does not spot it in 3 it proceds like it would if its at 1
        }*\
         */
//Testing marker position 2
        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);



        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.liftLeft.setTargetPosition(-387);
        robot.liftRight.setTargetPosition(-387);

        robot.liftLeft.setPower(0.30);
        robot.liftRight.setPower(0.30);

        sleep(2000);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.liftLeft.setTargetPosition(0);
        robot.liftRight.setTargetPosition(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(300);*/

//Marker postion 3

//this is the top levle

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);



        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.liftLeft.setTargetPosition(-370);
        robot.liftRight.setTargetPosition(-370);

        robot.liftLeft.setPower(0.30);
        robot.liftRight.setPower(0.30);

        sleep(2000);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.liftLeft.setTargetPosition(0);
        robot.liftRight.setTargetPosition(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(300);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(900);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/

//red duck spin
        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1030);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(markerPosition == 3) {
        robot.liftLeft.setTargetPosition(-200);
        robot.liftRight.setTargetPosition(-200);

        robot.liftLeft.setPower(0.30);
        robot.liftRight.setPower(0.30);

        sleep(2020);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(730);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.liftLeft.setTargetPosition(0);
        robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 2){

            robot.liftLeft.setTargetPosition(-250);
            robot.liftRight.setTargetPosition(-250);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }else if(markerPosition == 1){
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 1){
            robot.liftLeft.setTargetPosition(-564);
            robot.liftRight.setTargetPosition(-564);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(60);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
        }


        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(300);

        //Marker postion 3// this is the top levle

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);



        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.liftLeft.setTargetPosition(-400)
        robot.liftRight.setTargetPosition(-400);

        robot.liftLeft.setPower(0.30);
        robot.liftRight.setPower(0.30);

        sleep(2000);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.liftLeft.setTargetPosition(0);
        robot.liftRight.setTargetPosition(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(300);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(900);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //move out from the wall

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(940);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //moving forward toward the duck spin

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(1100);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //turing to aline with the duck spin

        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(420);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //moving foward so the spinner hits the spin

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);
        //the spinner spining to knock off the duck

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1100);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //moving backward away from the spinner

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(225);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //turing to aline with the wall

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //moving sideways to get in position

        robot.frontLeft.setPower(0.25);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.backRight.setPower(0.25);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        //going forward to hit the wall in the square

        //new stuff

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.20);
        robot.frontRight.setPower(-0.20);
        robot.backLeft.setPower(-0.20);
        robot.backRight.setPower(-0.20);
        sleep(2000);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.40);
        robot.frontRight.setPower(0.40);
        robot.backLeft.setPower(0.40);
        robot.backRight.setPower(-0.40);
        sleep(1030);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(2000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(markerPosition == 3) {
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(730);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 2){

            robot.liftLeft.setTargetPosition(-250);
            robot.liftRight.setTargetPosition(-250);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }else if(markerPosition == 1){
            robot.liftLeft.setTargetPosition(-200);
            robot.liftRight.setTargetPosition(-200);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(80);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        } else if (markerPosition == 1){
            robot.liftLeft.setTargetPosition(-564);
            robot.liftRight.setTargetPosition(-564);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(60);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.30);
            robot.liftRight.setPower(0.30);

            sleep(2020);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.gatherServo.setPower(-0.4);
            sleep(2200);
            robot.gatherServo.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
        }


        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(1600);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        sleep(300);*/

//robot.gatherServo.setPower(0.5);
//sleep(1200);
//robot.gatherServo.setPower(0);


        /*}else if(updatedRecognitions.size() == 0) {
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(-TURN_SPEED);
            Trobot.backLeft.setPower(TURN_SPEED);
            sleep(650);*\

         */


//robot.frontLeft.setPower(TURN_SPEED);
//robot.frontRight.setPower(-TURN_SPEED);
//robot.backRight(-TURN_SPEED);
//robot.backLeft(TURN_SPEED);
//sleep()

//robot.frontLeft.setPower(-TURN_SPEED)
//robot.frontRight.setPower(TURN_SPEED)
//robot.backRight.setPower(-TURN_SPEED)
//robot.backLeft.setPower(TURN_SPEED)
//sleep()

//robot.frontLeft.setPower(TURN_SPEED)
//robot.frontRight.setPower(TURN_SPEED)
//robot.backRight.setPower(TURN_SPEED)
//robot.backLeft.setPower(TURN_SPEED);


// Step 1:  Drive forward for 3 seconds
//robot.frontLeft.setPower(FORWARD_SPEED);
//robot.frontRight.setPower(FORWARD_SPEED);
//robot.backLeft.setPower(FORWARD_SPEED);
//robot.backRight.setPower(FORWARD_SPEED);
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        robot.frontLeft.setPower(TURN_SPEED);
        robot.frontRight.setPower(-TURN_SPEED);
        robot.backRight.setPower(-TURN_SPEED);
        robot.backLeft.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        robot.frontLeft.setPower(-FORWARD_SPEED);
        robot.frontRight.setPower(-FORWARD_SPEED);
        robot.backRight.setPower(-FORWARD_SPEED);
        robot.backLeft.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);*/

