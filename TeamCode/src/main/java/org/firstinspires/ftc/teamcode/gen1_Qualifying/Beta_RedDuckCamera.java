
package org.firstinspires.ftc.teamcode.gen1_Qualifying;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous (preselectTeleOp = "Beta_TeleOp")
public class Beta_RedDuckCamera extends LinearOpMode {
  /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    private static final String VUFORIA_KEY =
            "AVsEI0//////AAABmfbnDsRuiEqQsFcH7Lyqo3QuISqpJopGJxX15CNQ6JRwa6IhhZMdS346pkBRyp94aOkulPOzR+MQZ84lQbPclj/UW9I95nliTUyCT+Ie8Bw9qNN5X5Cv4sBkIAyNJpUXfxMjUf/5Hw098czCJ1HTiVoqUVB+AGNgZ6tLD4AGqbv/ftucrrA/nnzT045vPyCZCKujFStiBc1Hkab9Y96FE5wHPrfBeCrq8nYd0T+mB3eaCO3kUahLaqyjhFAyQnQCpa1oOzsqbDARsc5FuIcACzdzOFSG+LAEOj+Bgb7Nm+jJQHn8rYsCJi94aoSptfsgXSR0N2pTwhM3eQZEf9g6m1fbUJH18eCFJWwS1FVr3U+6\n";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        //Defines motors and servos
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        Servo armMotor = hardwareMap.get(Servo.class,"armMotor");
        DcMotor elementIntake = hardwareMap.get(DcMotor.class, "elementIntake");
        Servo trapdoor = hardwareMap.get(Servo.class,"trapdoor");

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            //Sets zoom and aspect ratio
            tfod.setZoom(1.0, 30.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;

                        //Autonomous program the robot runs after scanning the duck on the field
                          if (updatedRecognitions.isEmpty()) {
                              if (recognition.getLeft() < 400 && recognition.getLabel() == "Duck") {
                                  telemetry.addLine("Middle");
                                  break;
                              }
                              if (recognition.getRight() > 400 && recognition.getLabel() == "Duck") {
                                  telemetry.addLine("Right");
                                  break;
                              }
                          } else {
                              telemetry.addLine("Nothing-Left");
                          }

                      }
                      telemetry.update();
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


     // Initialize the TensorFlow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
