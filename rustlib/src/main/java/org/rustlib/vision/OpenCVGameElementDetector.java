package org.rustlib.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.rustlib.commandsystem.Subsystem;
import org.rustlib.core.RobotBase;
import org.rustlib.core.RobotBase.GameElementLocation;
import org.rustlib.rustboard.Rustboard;

import java.util.ArrayList;

public class OpenCVGameElementDetector extends Subsystem {
    private final OpenCvWebcam detectorCam;
    private final GameElementDetectorPipeline pipeline;
    private final ArrayList<Integer> locationHistory = new ArrayList<>();
    private final int frameAveragingCount;
    private GameElementLocation gameElementLocation = RobotBase.GameElementLocation.LEFT;
    private boolean streaming;

    private OpenCVGameElementDetector(Builder builder) {
        detectorCam = OpenCvCameraFactory.getInstance().createWebcam(builder.hardwareMap.get(WebcamName.class, builder.cameraName));
        detectorCam.setPipeline(builder.detectorPipeline);
        pipeline = builder.detectorPipeline;
        frameAveragingCount = builder.frameAveragingCount;
        if (builder.closeOnOpModeStart) {
            RobotBase.onOpModeStart(() -> closePipeline());
        }
        detectorCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                detectorCam.startStreaming(builder.streamSize.width, builder.streamSize.height);
                streaming = true;
                Rustboard.log("OpenCV game element detector camera opened.");
            }

            @Override
            public void onError(int errorCode) {
                Rustboard.log("OpenCV game element detector camera failed to open.");
            }
        });
    }

    public static SetHardwareMap getBuilder() {
        return new Builder();
    }

    public GameElementLocation getElementLocation() {
        return gameElementLocation;
    }

    public void closePipeline() {
        detectorCam.stopStreaming();
        streaming = false;
    }

    @Override
    public void periodic() {
        if (streaming) {
            locationHistory.add(pipeline.getGameElementLocation().ordinal());
            int sum = 0;
            int i = 0;
            while (i < locationHistory.size() && i < frameAveragingCount) {
                sum += locationHistory.get(locationHistory.size() - i - 1);
                i++;
            }
            gameElementLocation = RobotBase.GameElementLocation.values()[Math.round((float) sum / (float) i)];
        }
    }

    public interface SetHardwareMap {
        SetCameraName setHardwareMap(HardwareMap hardwareMap);
    }

    public interface SetCameraName {
        SetStreamSize setCameraName(String cameraName);
    }

    public interface SetStreamSize {
        SetDetectorPipeline setStreamSize(StreamDimension streamSize);
    }

    public interface SetDetectorPipeline {
        Builder setDetectorPipeline(GameElementDetectorPipeline detectorPipeline);
    }

    public static class Builder implements SetHardwareMap, SetCameraName, SetStreamSize, SetDetectorPipeline {
        private HardwareMap hardwareMap;
        private String cameraName;
        private StreamDimension streamSize;
        private GameElementDetectorPipeline detectorPipeline;
        private int frameAveragingCount = 1;
        private boolean closeOnOpModeStart;

        private Builder() {

        }

        @Override
        public SetCameraName setHardwareMap(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            return this;
        }

        @Override
        public SetStreamSize setCameraName(String cameraName) {
            this.cameraName = cameraName;
            return this;
        }

        @Override
        public SetDetectorPipeline setStreamSize(StreamDimension streamSize) {
            this.streamSize = streamSize;
            return this;
        }

        @Override
        public Builder setDetectorPipeline(GameElementDetectorPipeline detectorPipeline) {
            this.detectorPipeline = detectorPipeline;
            return this;
        }

        public Builder setFrameAveragingCount(int frameAveragingCount) {
            this.frameAveragingCount = frameAveragingCount;
            return this;
        }

        public Builder closePipelineOnOpModeStart() {
            closeOnOpModeStart = true;
            return this;
        }

        public OpenCVGameElementDetector build() {
            return new OpenCVGameElementDetector(this);
        }
    }

    public static class StreamDimension {
        public static final StreamDimension HIGH_DEF = new StreamDimension(1280, 720);
        public static final StreamDimension STANDARD_DEF = new StreamDimension(640, 480);
        public static final StreamDimension LOW_DEF = new StreamDimension(320, 240);

        public final int width;
        public final int height;

        public StreamDimension(int width, int height) {
            this.width = width;
            this.height = height;
        }
    }
}
