package autofunctions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import autofunctions.TerraCV;

public class TerraCVHandler {
    OpenCvInternalCamera2 phoneCam;
    public TerraCV terraCV = new TerraCV();


    public void init(HardwareMap hwMap){
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(terraCV);
        phoneCam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
        phoneCam.setFlashlightEnabled(true);
    }
    public void init(){
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(terraCV);
        phoneCam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
        phoneCam.setFlashlightEnabled(true);
    }
    public TerraCV.RingNum getRingNum(){
        return terraCV.ringNum;
    }

    public void stop(){
        phoneCam.stopStreaming();
    }
}
