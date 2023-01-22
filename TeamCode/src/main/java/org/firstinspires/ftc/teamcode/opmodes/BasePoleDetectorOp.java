//package org.firstinspires.ftc.teamcode.opmodes;
//
//import android.annotation.SuppressLint;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcodekt.components.Camera;
//import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto;
//
//@Autonomous
//public class BasePoleDetectorOp extends RogueBaseAuto {
//
//    /**
//     * Override this method and place your code here.
//     * <p>
//     * Please do not swallow the InterruptedException, as it is used in cases
//     * where the op mode needs to be terminated early.
//     *
//     * @throws InterruptedException
//     */
//    @SuppressLint("DefaultLocale")
//    @Override
//    public void executeOrder66() {
//        setPoleDetectorAsPipeline();
//
//        while (!opModeIsActive()){
//            telemetry.addData("Angle", Math.toDegrees(getBot().getCamera().getPoleDetector().getPoleAngle()));
//            telemetry.update();
//        }
//    }
//
//    public void setPoleDetectorAsPipeline(){
//        Camera cam = getBot().getCamera();
//        cam.setPipeline(cam.getPoleDetector());
//    }
//}
