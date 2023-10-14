package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.x

public enum class CupLocation {
    LEFT,
    CENTER,
    RIGHT,
    UNKNOWN
}
@TeleOp(name="TensorFlow Test 1", group="Concept")
class TensorFlowTest1: DriveMethods(){
    override fun runOpMode() {
        initVision(Variables.VisionProcessors.TFOD)

        telemetry.addLine("TFODInitiated")
        telemetry.update()
        waitForStart()
        var cupLocation: CupLocation = CupLocation.UNKNOWN
        while (opModeIsActive()) {
            telemetry.addLine("Cool stuff is happening maybe")
            addDataToTelemetry()
            sleep(100)
            for (recognition in getDetectionsTFOD()!!) {
                if (recognition.label == "cup") {
                    val x = ((recognition.left + recognition.right) / 2).toDouble()
                    val y = ((recognition.top + recognition.bottom) / 2).toDouble()

                    if (x <= 640/3) {
                        cupLocation = CupLocation.LEFT

                    } else if (x > 640/3 && x < 1280/3) {
                        cupLocation = CupLocation.CENTER
                    } else if (x > 1280/3 && x <= 1280){
                        cupLocation = CupLocation.RIGHT
                    }
                }
            }
            telemetry.addLine("The cup is at $cupLocation")
            telemetry.update()
        }

    }


    /**
     * Initialize the TensorFlow Object Detection processor.
     */



    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

    //Left = 156/265
    //Right = 530/262



}

