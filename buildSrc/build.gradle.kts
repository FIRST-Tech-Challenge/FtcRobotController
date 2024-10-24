
plugins {
    `kotlin-dsl`
    `kotlin-dsl-precompiled-script-plugins`
}

dependencies {
    implementation(gradleApi())
    implementation(libs.android.build)
    implementation(libs.android.gradle.api)
    constraints{
        implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk7:2.0.20")
        implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk8:2.0.20")
    }
}

