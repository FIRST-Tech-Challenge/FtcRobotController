package com.kalipsorobotics.tensorflow;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.provider.MediaStore;
import android.util.Log;
import android.widget.Button;
import android.widget.ImageView;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import org.firstinspires.ftc.teamcode.R;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;

public class MainActivity extends AppCompatActivity {

    private static final int CAMERA_REQUEST = 1888;
    private static final String TAG = "RobotDetector"; // Use this tag for logging
    private Interpreter tfliteInterpreter;
    private ImageView imageView;

    @SuppressLint("MissingInflatedId")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Button captureButton = findViewById(R.id.captureButton);
        imageView = findViewById(R.id.imageView);

        try {
            tfliteInterpreter = new Interpreter(loadModelFile("robotv2_model.tflite"));
        } catch (IOException e) {
            e.printStackTrace();
            Log.e(TAG, "Failed to load model.");
        }

        captureButton.setOnClickListener(v -> openCamera());
    }

    private void openCamera() {
        Intent cameraIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
        startActivityForResult(cameraIntent, CAMERA_REQUEST);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == CAMERA_REQUEST && resultCode == RESULT_OK && data != null) {
            Bitmap photo = (Bitmap) data.getExtras().get("data");
            imageView.setImageBitmap(photo);

            // Resize and preprocess the image
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(photo, 224, 224, true);
            float[][] output = runModel(resizedBitmap);

            // Log the result
            if (output[0][0] > 0.5) { // Assuming model outputs [1, 1] with "robot probability"
                Log.d(TAG, "Robot detected!");
            } else {
                Log.d(TAG, "No robot detected.");
            }
        }
    }

    private ByteBuffer preprocessImage(Bitmap bitmap) {
        // Convert image to ByteBuffer in model input format
        int imageSize = 224; // Model input size
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(4 * imageSize * imageSize * 3); // 3 channels (RGB)
        byteBuffer.order(ByteOrder.nativeOrder());

        int[] pixels = new int[imageSize * imageSize];
        bitmap.getPixels(pixels, 0, imageSize, 0, 0, imageSize, imageSize);

        for (int pixel : pixels) {
            int r = (pixel >> 16) & 0xFF;
            int g = (pixel >> 8) & 0xFF;
            int b = pixel & 0xFF;

            // Normalize the pixel values to [0, 1]
            byteBuffer.putFloat(r / 255.0f);
            byteBuffer.putFloat(g / 255.0f);
            byteBuffer.putFloat(b / 255.0f);
        }

        return byteBuffer;
    }

    private float[][] runModel(Bitmap bitmap) {
        // Preprocess the image
        ByteBuffer inputBuffer = preprocessImage(bitmap);

        // Define output buffer
        float[][] output = new float[1][1]; // Assuming the model outputs a single probability

        // Run inference
        tfliteInterpreter.run(inputBuffer, output);

        return output;
    }

    private ByteBuffer loadModelFile(String modelPath) throws IOException {
        FileInputStream inputStream = new FileInputStream(getAssets().openFd(modelPath).getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = getAssets().openFd(modelPath).getStartOffset();
        long declaredLength = getAssets().openFd(modelPath).getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }
}