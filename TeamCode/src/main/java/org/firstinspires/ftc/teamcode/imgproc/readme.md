## Using these files

Because these files were made with `Kotlin`, then they have to be
imported to Java like the code below.

*This code sample is not exact and needs to be adopted for other use*


## Code Sample
**Kotlin file**
```
package org.example

class Util

fun getTime() { /*...*/ }
```

**Java**
```
new org.example.Util();
org.example.AppKt.getTime();
```
[Source](https://kotlinlang.org/docs/java-to-kotlin-interop.html)

## build.gradle

The following code needs to be added to `build.gradle`

**Project build.gradle file**
```groovy
buildscript {
    ext.kotlin_version = '1.4.10'
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
    implementation "org.jetbrains.kotlin:kotlin-stdlib:$kotlin_version"
}
```
