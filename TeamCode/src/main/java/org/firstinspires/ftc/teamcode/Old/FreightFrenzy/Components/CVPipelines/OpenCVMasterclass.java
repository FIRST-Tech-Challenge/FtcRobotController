package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.CVPipelines;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.BlueWarehouseScam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
@Config

public class OpenCVMasterclass {
    OpenCvWebcam backWebcam;
    OpenCvWebcam frontWebcam;
    public OpenCVMasterclass() {
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
//                .splitLayoutForMultipleViewports(
//                        cameraMonitorViewId, //The container we're splitting
//                        2, //The number of sub-containers to create
//                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
        backWebcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "BackWebcam"), cameraMonitorViewId);
//        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "FrontWebcam"), viewportContainerIds[0]);
    }
    public int BlueTeamElem(){
        BlueTeamElem opencv = new BlueTeamElem();
        double starttime= op.getRuntime();
        backWebcam.setPipeline(opencv);
        backWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                backWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(backWebcam, 30);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        op.telemetry.addLine("Waiting for start");
        op.telemetry.update();
        while(!op.isStarted()||op.getRuntime()-starttime<5){
            op.sleep(50);
        }
        backWebcam.stopStreaming();
        if(opencv.getLocation()== BlueTeamElem.Location.NOT_FOUND) {
            return 2;
        }
        else if(opencv.getLocation()== BlueTeamElem.Location.MID) {
            return 0;
        }
        else if(opencv.getLocation()== BlueTeamElem.Location.LEFT) {
            return 1;
        }
        else{
            return 0;
        }
    }
    public int RedTeamElem(){
        RedTeamElem opencv = new RedTeamElem();
        double starttime= op.getRuntime();
        backWebcam.setPipeline(opencv);
        backWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                backWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        op.telemetry.addLine("Waiting for start");
        op.telemetry.update();
        while(!op.isStarted()){
            op.sleep(100);
        }
        backWebcam.stopStreaming();
        if(opencv.getLocation()== RedTeamElem.Location.NOT_FOUND) {
            return 2;
        }
        else if(opencv.getLocation()== RedTeamElem.Location.MID) {
            return 1;
        }
        else if(opencv.getLocation()== RedTeamElem.Location.LEFT) {
            return 0;
        }
        else{
            return 3;
        }
    }
    public double[] BlueWarehouseScam(){
        BlueWarehouseScam opencv = new BlueWarehouseScam();
        backWebcam.setPipeline(opencv);
        backWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                backWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        op.telemetry.addLine("Waiting for start");
        op.telemetry.update();
        op.waitForStart();
        backWebcam.stopStreaming();
        ArrayList<double[]> pos=opencv.getLocation();
        ArrayList<double[]> returnVal = new ArrayList<double[]>();
        double[] mindists = new double[pos.size()];
        for(int i=0;i<pos.size();i++){
            mindists[i]=1000;
            int tooSimilar = 1000;
            for(int j=0;j<pos.size();j++){
                double distance = pow(pos.get(i)[0]-pos.get(j)[0],2)+pow(pos.get(i)[1]-pos.get(j)[1],2);
                if(distance<2){
                    tooSimilar = j;
                    break;
                }
                else if(distance<mindists[i]){
                    mindists[i]=distance;
                }
            }
            if(tooSimilar==1000){
                returnVal.add(pos.get(i));
            }
            else if(tooSimilar>i){
                returnVal.add(pos.get(i));
                mindists[i]=1000;
            }
        }
        for(int i=0;i<returnVal.size();i++){
            for(int j=i+1;j<returnVal.size();j++){
                double[] dists= {returnVal.get(i)[0]*returnVal.get(i)[0]+returnVal.get(i)[1]*returnVal.get(i)[1],returnVal.get(j)[0]*returnVal.get(j)[0]+returnVal.get(j)[1]*returnVal.get(j)[1]};
                if(dists[0]>dists[1]){
                    double[] temp = returnVal.get(i);
                    double tempdist = mindists[i];
                    mindists[i]=mindists[j];
                    mindists[j]=tempdist;
                    returnVal.set(i,returnVal.get(j));
                    returnVal.set(j, temp);
                }
            }
        }
        for(int i=0;i<returnVal.size();i++){
            if(mindists[i]>3){
                return returnVal.get(i);
            }
        }


        return new double[]{};
    }

}
