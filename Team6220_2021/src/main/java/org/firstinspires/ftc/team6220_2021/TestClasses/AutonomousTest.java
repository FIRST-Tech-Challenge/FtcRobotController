package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_2021.MasterAutonomous;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // This is used just like TSE detection, but with cubes/ducks/spheres instead of the TSE.
                // Used to detect freight for cycling to the alliance shipping hub in autonomous
                /*if (tfod != null) {
                    tfod.activate();
                    tfod.setZoom(2.0, 16.0/9.0);

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                            i++;

                            if (recognition.getLabel().equals("TSE")) {
                                double TSELocation = (recognition.getLeft() + recognition.getRight()) / 2.0;

                                if (TSELocation > 200.0 && TSELocation <= 333.0) {
                                    barcode = 0;
                                    telemetry.addData("barcode: ", barcode);
                                } else if (TSELocation > 333.0 && TSELocation <= 467.0) {
                                    barcode = 1;
                                    telemetry.addData("barcode: ", barcode);
                                } else if (TSELocation > 467.0 && TSELocation <= 600.0) {
                                    barcode = 2;
                                    telemetry.addData("barcode: ", barcode);
                                }
                            }
                        }
                        telemetry.update();
                    }
                }*/
            }
        }
    }
}