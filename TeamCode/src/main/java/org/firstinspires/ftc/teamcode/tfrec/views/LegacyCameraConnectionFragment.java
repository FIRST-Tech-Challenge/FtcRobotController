package org.firstinspires.ftc.teamcode.tfrec.views;

import android.annotation.SuppressLint;
import android.app.Fragment;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.ViewGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.tfrec.utils.ImageUtils;

import java.io.IOException;
import java.util.List;

@SuppressLint("ValidFragment")
public class LegacyCameraConnectionFragment extends Fragment {
    /**
     * Conversion from screen rotation to JPEG orientation.
     */
    private static final SparseIntArray ORIENTATIONS = new SparseIntArray();
    private static final String TAG = "Fragment";

    static {
        ORIENTATIONS.append(Surface.ROTATION_0, 90);
        ORIENTATIONS.append(Surface.ROTATION_90, 0);
        ORIENTATIONS.append(Surface.ROTATION_180, 270);
        ORIENTATIONS.append(Surface.ROTATION_270, 180);
    }

    private Camera camera;
    private Camera.PreviewCallback imageListener;
    private Size desiredSize;
    private Telemetry telemetry;
    /**
     * The layout identifier to inflate for this Fragment.
     */
    private int layout;
    /**
     * An {@link AutoFitTextureView} for camera preview.
     */
    private AutoFitTextureView textureView;
    /**
     * {@link TextureView.SurfaceTextureListener} handles several lifecycle events on a {@link
     * TextureView}.
     */
    private final TextureView.SurfaceTextureListener surfaceTextureListener =
            new TextureView.SurfaceTextureListener() {
                @Override
                public void onSurfaceTextureAvailable(
                        final SurfaceTexture texture, final int width, final int height) {
                    Log.d("Listener", "Starting texture listener");
                    //This variable is required for the fix for the Android media center issue when the camera fails to initialize
                    final TextureView.SurfaceTextureListener listener = this;
                    int index = getCameraId();
                    Log.d("cameraIndex", String.valueOf(index));
                    camera = Camera.open(index);

                    camera.setErrorCallback(new Camera.ErrorCallback() {
                        @Override
                        public void onError(int i, Camera camera) {
                            Log.e("Listener", String.format("Camera error %d. Width: %d, Height: %d", i, width, height));
                            //Fix for the Android media center issue when the camera fails to initialize
                            if (i == 100){
                                Log.d("Listener", "Camera error is 100");
                                stopCamera();
                                Log.d("Listener", "Stopped and released the camera");
                                listener.onSurfaceTextureAvailable(texture, width, height);
                            }
                        }
                    });

                    try {
//                        camera.stopPreview();
//                        camera.release();
                        Camera.Parameters parameters = camera.getParameters();
                        List<String> focusModes = parameters.getSupportedFocusModes();
                        if (focusModes != null
                                && focusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE)) {
                            parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
                        }
                        List<Camera.Size> cameraSizes = parameters.getSupportedPreviewSizes();
                        Size[] sizes = new Size[cameraSizes.size()];
                        int i = 0;
                        for (Camera.Size size : cameraSizes) {
                            sizes[i++] = new Size(size.width, size.height);
                        }
                        Size previewSize =
                                CameraConnectionFragment.chooseOptimalSize(
                                        sizes, desiredSize.getWidth(), desiredSize.getHeight(), telemetry);
                        parameters.setPreviewSize(previewSize.getWidth(), previewSize.getHeight());
                        camera.setDisplayOrientation(90);
                        camera.setParameters(parameters);
                        camera.setPreviewTexture(texture);
                    } catch (IOException exception) {
                        Log.e("Listener", "Error when starting texture listener", exception);
                        camera.release();
                    }

                    camera.setPreviewCallbackWithBuffer(imageListener);
                    Camera.Size s = camera.getParameters().getPreviewSize();
                    camera.addCallbackBuffer(new byte[ImageUtils.getYUVByteSize(s.height, s.width)]);

                    textureView.setAspectRatio(s.height, s.width);

                    camera.startPreview();
                    Log.d("Listener", "Camera preview started by texture listener");
                }

                @Override
                public void onSurfaceTextureSizeChanged(
                        final SurfaceTexture texture, final int width, final int height) {
                }

                @Override
                public boolean onSurfaceTextureDestroyed(final SurfaceTexture texture) {
                    return true;
                }

                @Override
                public void onSurfaceTextureUpdated(final SurfaceTexture texture) {
                }
            };
    /**
     * An additional thread for running tasks that shouldn't block the UI.
     */
    private HandlerThread backgroundThread;

    @SuppressLint("ValidFragment")
    public LegacyCameraConnectionFragment(
            final Camera.PreviewCallback imageListener, final int layout, final Size desiredSize, Telemetry t) {
        this.imageListener = imageListener;
        this.layout = layout;
        this.desiredSize = desiredSize;
        telemetry = t;
    }

    @Override
    public View onCreateView(
            final LayoutInflater inflater, final ViewGroup container, final Bundle savedInstanceState) {
        return inflater.inflate(layout, container, false);
    }

    @Override
    public void onViewCreated(final View view, final Bundle savedInstanceState) {
        textureView = (AutoFitTextureView) view.findViewById(R.id.texture);
    }

    @Override
    public void onActivityCreated(final Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
    }

    @Override
    public void onResume() {
        super.onResume();
        startBackgroundThread();
        // When the screen is turned off and turned back on, the SurfaceTexture is already
        // available, and "onSurfaceTextureAvailable" will not be called. In that case, we can open
        // a camera and start preview from here (otherwise, we wait until the surface is ready in
        // the SurfaceTextureListener).
        Log.d(TAG, "Starting texture preview");
        if (textureView.isAvailable()) {
            if (camera != null) {
                camera.startPreview();
                Log.d(TAG, "Started preview");
            }
            else{
                Log.e(TAG, "Failed to start preview. Camera unavailable");
            }
        } else {
            Log.d(TAG, "Texture not available yet. Starting the listener...");
            textureView.setSurfaceTextureListener(surfaceTextureListener);
        }
    }

    @Override
    public void onPause() {
        stopCamera();
        stopBackgroundThread();
        super.onPause();
    }

    /**
     * Starts a background thread and its {@link Handler}.
     */
    private void startBackgroundThread() {
        backgroundThread = new HandlerThread("CameraBackground");
        backgroundThread.start();
    }

    /**
     * Stops the background thread and its {@link Handler}.
     */
    private void stopBackgroundThread() {
        backgroundThread.quitSafely();
        try {
            backgroundThread.join();
            backgroundThread = null;
        } catch (final InterruptedException e) {
            Log.e(TAG, "stopBackgroundThread", e);
        }
    }

    protected void stopCamera() {
        if (camera != null) {
            camera.stopPreview();
            camera.setPreviewCallback(null);
            camera.release();
            camera = null;
        }
    }

    private int getCameraId() {
        Camera.CameraInfo ci = new Camera.CameraInfo();
        for (int i = 0; i < Camera.getNumberOfCameras(); i++) {
            Camera.getCameraInfo(i, ci);
            if (ci.facing == Camera.CameraInfo.CAMERA_FACING_BACK) return i;
        }
        return -1; // No camera found
    }
}