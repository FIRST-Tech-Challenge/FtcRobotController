## Usage

1. Import the class
2. Initialize the class
3. Run the `process` method while passing a `Bitmap` image
4. The function returns an `Array` of `ObjectDetected` class (custom defined).
       This object class has two attributes: `label` (The name of the detected object) and
       `boundingBox` (The area of the object)

**Importing**
```java
import org.firstinspires.ftc.teamcode.imgproc.ImgProc;

ImgProc test_var = new ImgProc();
```

## build.gradle

The following code needs to be added to `build.gradle`

**Project build.gradle file**
```groovy
buildscript {
    ext.kotlin_version = '1.4.32'
    dependencies {
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}
```

**Module-level build.gradle file (when necessary)**
```groovy
plugins {
    id 'kotlin-android'
}

dependencies {
    implementation 'androidx.core:core-ktx:1.3.2'
    implementation 'org.tensorflow:tensorflow-lite-task-vision:0.2.0'
    implementation "org.jetbrains.kotlin:kotlin-stdlib:$kotlin_version"
}
```