package org.firstinspires.ftc.robotcontroller.internal;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

@Autonomous(name="AI Test", group="Linear Opmode")
//@Disabled
public class NetTest extends LinearOpMode {
    WolfNet ai = new WolfNet(4, 2, 2, "Weights", 0.5);
    double num1 = 0;
    double num2 = 0;
    double num3 = 0;
    double num4 = 0;
    double[] errors;
    FileWriter debug_file;

    @Override
    public void runOpMode() {

        ai.ResetWeights();

        Random random = new Random();

        try{
            new FileWriter(Environment.getExternalStorageDirectory().getPath()+"/Debug.txt", false).close();
            debug_file = new FileWriter(Environment.getExternalStorageDirectory().getPath()+"/Debug.txt", true);
        }catch(IOException e){

        }

        waitForStart();

        while (opModeIsActive()) {

            num1 = random.nextDouble() * 2 - 1;
            num2 = random.nextDouble() * 2 - 1;
            num3 = random.nextDouble() * 2 - 1;
            num4 = random.nextDouble() * 2 - 1;

            double[][] in = {{num1, num2, num3, num4}};
            double[][] correct = {{num1 * num2, num3 * num4}};
            ai.Train(in, correct);

            try{
                debug_file.write("[" + num1 + ", " + num2 + ", " + num3 + ", " + num4 + "], [" +
                        correct[0][0] + ", " + correct[0][1] +"], [" +
                        ai.output.layer[0][0] + ", " + ai.output.layer[0][1] + "], [" +
                        Matrix.error(ai.output.layer, correct)[0][0] + ", " + Matrix.error(ai.output.layer, correct)[0][1] + "], " +
                        ai.output.train_count + "\n");
            }catch(IOException e){

            }

            telemetry.addData("Error", Matrix.error(ai.output.layer, correct)[0][0]);
            telemetry.addData("Time", ai.output.train_count);
            telemetry.addData("Learning Rate", ai.learningRate);
            telemetry.update();
        }
        try{
            debug_file.close();
        }catch(IOException e){

        }
    }
}