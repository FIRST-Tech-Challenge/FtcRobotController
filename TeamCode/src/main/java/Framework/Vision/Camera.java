package Framework.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera {
	private OpenCvCamera camera;

	private LinearOpMode mode;
	private boolean open = false;

	public Camera(HardwareMap hw, String name, OpenCvPipeline pipeline, LinearOpMode m) {
		mode = m;

		WebcamName webcamName = hw.get(WebcamName.class, name);

		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

		mode.telemetry.addData("Camera", "Opening");
		mode.telemetry.update();
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				camera.startStreaming(1280, 720);
				camera.setPipeline(pipeline);
				mode.telemetry.addData("Status", "Camera running");
				open = true;
			}

			@Override
			public void onError(int errorCode) {
				mode.telemetry.addData("Error", "Camera failed to open");
				mode.telemetry.update();
			}
		});
	}

	public boolean isOpen() {
		return open;
	}

	public void stop() {
		camera.stopStreaming();
	}
}