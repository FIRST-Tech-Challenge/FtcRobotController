import java.text.SimpleDateFormat
import java.util.Date

plugins {
    id(libs.plugins.android.library.get().pluginId)
}

dependencies {
    implementation(libs.bundles.ftc)
    implementation(libs.androidx.appcompat)
}

android {
    namespace = "com.qualcomm.ftcrobotcontroller"

    compileSdk = 35

    defaultConfig {
        minSdk = 24
        lint.targetSdk = 28
        
        buildConfigField(
            "String",
            "APP_BUILD_TIME",
            "\"${SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ").format(Date())}\""
        )
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    buildFeatures {
        buildConfig = true
    }
}

