package org.firstinspires.ftc.teamcode.main.utils.autonomous.image;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Initial Object Detection", group = "Concept")
public class InitialDetection extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker"
    };

    private static final String OBJECT_TO_IDENT = "Duck";

    private boolean objectIdent = false;
    private boolean objectIdentSTRICT = false;

    private String VUFORIA_KEY;

    private TFLITE_Wrapper tflite;

    private ArrayList<InitialPositions> PossiblePositions;

    private boolean isZoomed = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        PossiblePositions = new ArrayList();
        PossiblePositions.add(InitialPositions.POS1);
        PossiblePositions.add(InitialPositions.POS2);
        PossiblePositions.add(InitialPositions.POS3);


        VUFORIA_KEY = Resources.Misc.VuforiaKey;

        tflite = new TFLITE_Wrapper(hardwareMap);
        tflite.TFOD_MODEL_ASSET = TFOD_MODEL_ASSET;
        tflite.LABELS = LABELS;

        tflite.init();
        if (tflite.tfod != null) {
            tflite.activate();

            // tflite.setZoom(1.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tflite.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tflite.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.addData(String.format("  angle (%d)", i),
                                    String.format("%.03f", recognition.estimateAngleToObject(AngleUnit.DEGREES)));
                            i++;

                            if (!objectIdent || !objectIdentSTRICT){
                                if (InitialPositions.POS1.evalPos((int) recognition.getLeft())) {
                                    if (recognition.getLabel().equals(OBJECT_TO_IDENT)) {
                                        PossiblePositions = new ArrayList();
                                        PossiblePositions.add(InitialPositions.POS1);
                                        objectIdentSTRICT = true;
                                    } else if (PossiblePositions.contains(InitialPositions.POS1)) {
                                        PossiblePositions.remove(InitialPositions.POS1);
                                    }
                                } else if (InitialPositions.POS2.evalPos((int) recognition.getLeft())) {
                                    if (recognition.getLabel().equals(OBJECT_TO_IDENT)) {
                                        PossiblePositions = new ArrayList();
                                        PossiblePositions.add(InitialPositions.POS2);
                                        objectIdentSTRICT = true;
                                    } else if (PossiblePositions.contains(InitialPositions.POS1)) {
                                        PossiblePositions.remove(InitialPositions.POS2);
                                    }
                                } else if (InitialPositions.POS3.evalPos((int) recognition.getLeft())) {
                                    if (recognition.getLabel().equals(OBJECT_TO_IDENT)) {
                                        PossiblePositions = new ArrayList();
                                        PossiblePositions.add(InitialPositions.POS3);
                                        objectIdentSTRICT = true;
                                    } else if (PossiblePositions.contains(InitialPositions.POS1)) {
                                        PossiblePositions.remove(InitialPositions.POS3);
                                    }
                                }
                            }
                        }
                        if (PossiblePositions.size() == 1) {
                            objectIdent = true;
                            telemetry.addLine("Object Identified!");
                            telemetry.addData("Position: ", PossiblePositions.get(0));
                        }
                        else objectIdent = false;

                        StringBuilder strPositions = new StringBuilder();
                        for (InitialPositions positions : PossiblePositions) {
                            strPositions.append(positions.getName()).append(" ");
                        }
                        telemetry.addData("Possible Positions", strPositions.toString());

                        telemetry.update();

                        if (gamepad1.cross) {
                            PossiblePositions = new ArrayList();
                            PossiblePositions.add(InitialPositions.POS1);
                            PossiblePositions.add(InitialPositions.POS2);
                            PossiblePositions.add(InitialPositions.POS3);
                            objectIdent = false;
                        }
                        if (gamepad1.circle) {
                            if (isZoomed) { tflite.setZoom(1.0, 16.0/9.0);}
                            else { tflite.setZoom(1.5, 16.0/9.0); }

                            isZoomed = !isZoomed;

                            sleep(100);
                        }
                    }
                }
            }
        }
    }
}
