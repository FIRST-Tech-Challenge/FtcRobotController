To install roadrunner steps:
1. install Andriod Studio - see https://developer.android.com/studio
2. download the ZIP code of the RoadRunnerQUICKSTART - see https://github.com/Iris-TheRainbow/RoadRunnerQuickstart15031
3. open Andriod Studio and then click open and select the extracted zip folder from step 2


general links: 
https://learnroadrunner.com/quickstart-overview.html#what-s-feedforward


add this, maybe this works (to the "build.dependencies.grandle"): 
repositories {
    mavenCentral()
    google() // Needed for androidx
    maven { url = 'https://maven.brott.dev/' }
}

dependencies {
    implementation 'org.firstinspires.ftc:Inspection:9.1.0'
    implementation 'org.firstinspires.ftc:Blocks:9.1.0'
    implementation 'org.firstinspires.ftc:Tfod:9.1.0'
    implementation 'org.firstinspires.ftc:RobotCore:9.1.0'
    implementation 'org.firstinspires.ftc:RobotServer:9.1.0'
    implementation 'org.firstinspires.ftc:OnBotJava:9.1.0'
    implementation 'org.firstinspires.ftc:Hardware:9.1.0'
    implementation 'org.firstinspires.ftc:FtcCommon:9.1.0'
    implementation 'org.firstinspires.ftc:Vision:9.1.0'
    implementation 'org.firstinspires.ftc:gameAssets-CenterStage:1.0.0'
    implementation 'org.tensorflow:tensorflow-lite-task-vision:0.4.3'
    runtimeOnly 'org.tensorflow:tensorflow-lite:2.12.0'
    implementation 'androidx.appcompat:appcompat:1.2.0' // change to implementation 'androidx.appcompat:appcompat:1.6.1' if this doesn't work the first time

    implementation('com.acmerobotics.dashboard:dashboard:0.4.15') {
        exclude group: 'org.firstinspires.ftc'
    }

}

שגגגש
