//package com.example.loadimagefromfile;
//
//import android.content.Intent;
//import android.graphics.Bitmap;
//import android.graphics.BitmapFactory;
//import android.net.Uri;
//import android.os.Bundle;
//import android.widget.Button;
//import android.widget.ImageView;
//
//import androidx.annotation.Nullable;
//import androidx.appcompat.app.AppCompatActivity;
//
//import org.firstinspires.ftc.teamcode.R;
//
//import java.io.IOException;
//
//public class MainActivity extends AppCompatActivity {
//
//    private static final int PICK_IMAGE_REQUEST = 1;
//
//    private ImageView imageView;
//
//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);
//
//        Button buttonSelectImage = findViewById(R.id.buttonSelectImage);
//        imageView = findViewById(R.id.imageView);
//
//        buttonSelectImage.setOnClickListener(v -> openFilePicker());
//    }
//
//    private void openFilePicker() {
//        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
//        intent.setType("image/*");
//        intent.addCategory(Intent.CATEGORY_OPENABLE);
//        startActivityForResult(intent, PICK_IMAGE_REQUEST);
//    }
//
//    @Override
//    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
//        super.onActivityResult(requestCode, resultCode, data);
//
//        if (requestCode == PICK_IMAGE_REQUEST && resultCode == RESULT_OK && data != null) {
//            Uri imageUri = data.getData();
//            try {
//                // Load the image as a Bitmap
//                Bitmap bitmap = getBitmapFromUri(imageUri);
//                // Display the image in the ImageView
//                imageView.setImageBitmap(bitmap);
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }
//    }
//
//    private Bitmap getBitmapFromUri(Uri uri) throws IOException {
//        return BitmapFactory.decodeStream(getContentResolver().openInputStream(uri));
//    }
//}
