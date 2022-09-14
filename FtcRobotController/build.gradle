import java.text.SimpleDateFormat

//
// build.gradle in FtcRobotController
//
apply plugin: 'com.android.library'

android {

    defaultConfig {
        minSdkVersion 24
        //noinspection ExpiredTargetSdkVersion
        targetSdkVersion 28
        buildConfigField "String", "APP_BUILD_TIME", '"' + (new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.ROOT).format(new Date())) + '"'
    }

    compileSdkVersion 29

    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8
        targetCompatibility JavaVersion.VERSION_1_8
    }
    namespace = 'com.qualcomm.ftcrobotcontroller'
}

apply from: '../build.dependencies.gradle'
