/*
 * Tests Tensor Flow. If Tensor Flow detects
 * 4 rings, the robot should move forward.
 * If it detects 1 ring, the robot should
 * move left. Otherwise, the robot should move
 * right.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2020-November-5
 * @status: Not fully working
 */

package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.ObjectDetection.TensorFlow;

import java.util.ArrayList;

@Autonomous(name= "TensorFlowTest ", group="Tests: ")
//@Disabled
public class TensorFlowTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        TensorFlow tensorFlow = new TensorFlow(this);

        waitForStart();

//        tensorFlow.moveTensorFlowServo(0.75);


//        sleep(5000);

        int rings=-1; //For DEBUGGING, replace with "int rings= -1;"
        int i=0;

        int numOfTime4Rings=0;
        int numOfTime1Ring=0;
        int numOfTime0Rings=0;

        int arraySize = 11;
        ArrayList<Integer> NumberOfRings = new ArrayList<>(arraySize);

        for (int index = 0; index<arraySize; index++) {
            tensorFlow.runTensorFlow();
            sleep(10);
            rings = tensorFlow.getNumberOfRings();
            NumberOfRings.add(index, rings);
//            telemetry.addData("11 Number of Rings: ", "i=%4d %d", i++, rings);
//            telemetry.update();
//            sleep(100);
        }
        sleep(2000);

        telemetry.addData("Number of Rings: ", "%d %d %d %d %d %d", NumberOfRings.get(0),NumberOfRings.get(1),NumberOfRings.get(2),NumberOfRings.get(3),NumberOfRings.get(4), NumberOfRings.get(5));
        telemetry.addData("Number of Rings: ", "%d %d %d %d %d", NumberOfRings.get(6),NumberOfRings.get(7),NumberOfRings.get(8),NumberOfRings.get(9),NumberOfRings.get(10));
        telemetry.update();
        sleep(2000);

//        boolean ifActive = this.opModeIsActive();
//        telemetry.addData("ifActive", ifActive);
//        telemetry.update();
//        sleep(1000);
//        boolean ifActive2 = opModeIsActive();
//        telemetry.addData("ifActive2", ifActive2);
//        telemetry.update();
//        sleep(1000);
//        telemetry.addData("ifStopRequested", isStopRequested());
//        telemetry.update();
//        sleep(1000);

        while(opModeIsActive() && !isStopRequested()) {

            tensorFlow.runTensorFlow();
            NumberOfRings.remove(0);
            NumberOfRings.add(tensorFlow.getNumberOfRings());
            telemetry.addData("Number of Rings: ", "%d %d %d %d %d %d", NumberOfRings.get(0),NumberOfRings.get(1),NumberOfRings.get(2),NumberOfRings.get(3),NumberOfRings.get(4), NumberOfRings.get(5));
            telemetry.addData("Number of Rings: ", "%d %d %d %d %d", NumberOfRings.get(6),NumberOfRings.get(7),NumberOfRings.get(8),NumberOfRings.get(9),NumberOfRings.get(10));

            for (i = 0; i<arraySize; i++) {
                if (NumberOfRings.get(i) == 4) {
                    numOfTime4Rings++;
                } else if (NumberOfRings.get(i) == 1) {
                    numOfTime1Ring++;
                } else {
                    numOfTime0Rings++;
                }
            }

            telemetry.addData("Rings Summary: ", "4-rings: %2d 1-ring: %2d 0-rings: %2d", numOfTime4Rings, numOfTime1Ring, numOfTime0Rings);

            if (numOfTime4Rings>numOfTime1Ring && numOfTime4Rings>=numOfTime0Rings){
                rings = 4;
            } else if (numOfTime1Ring>numOfTime4Rings && numOfTime1Ring>=numOfTime0Rings) {
                rings = 1;
            } else {
                rings = 0;
            }

            telemetry.addData("FinalNumOfRings: ", rings);
            telemetry.update();
            sleep(100);
        }

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        tensorFlow.stopTensorFlow();

        telemetry.addData("NumberOfRings: ", 0);
        telemetry.update();
        sleep(2000);
        stop();
    }
}
