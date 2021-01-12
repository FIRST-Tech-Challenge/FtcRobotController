# Handling FTC Secrets

## What secrets are

Secrets are things like api keys that you don't want to be made public. 

## Creating the FTC Secrets Class

When using a library like Vuforia where there is an api key you need to keep it hiden. This can be done with the following steps.

### **Step 1**
Create a file in the folder called `ftcsecrets.java` **it must be called exactly this**.

### **Step 2**
Copy the following code into this new file:
```java
package org.firstinspires.ftc.teamcode;

public class ftcsecrets {
    public static class secrets {
        public static String VUFORIA_KEY = "ENTER VUFORIA KEY HERE";
    }
}
```

### **Step 2b**
For any other key that the team may need create another public static class

## **DO NOT COMMIT THIS FILE IT SHOULD BE IGNORED BY THE ``.gitignore`` FILE**
