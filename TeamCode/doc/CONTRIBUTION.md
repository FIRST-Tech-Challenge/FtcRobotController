# Steps to Contribute to Git Repo
## Downloading Project
On a windows, open CommandPrompt. Locate a folder in which you want to store
this project by using <strong> cd </strong> followed by the path to the folder.
After pressing <b> enter</b>, run this line:

<code>git clone [https://github.com/DishaVai/FtcRobotController.git](https://github.com/DishaVai/FtcRobotController.git)</code>

## System Setup
The code supports Java version 17.0.4 and Gradle version 8.3.

### Java Setup
Make sure to download Java version Java SE Development Kit 17.0.4 from the Java
archive, which you can find here: https://www.oracle.com/java/technologies/javase/jdk17-archive-downloads.html.

Set up Java_Home and Path variables using the instructions provided here:
https://docs.oracle.com/en/java/javase/17/install/overview-jdk-installation.html.

### Gradle Setup
Download Gradle version 8.3 here: https://gradle.org/releases/. Download
<b> Binary Only</b>. Follow the instructions to install gradle 
and set path variables found here: https://docs.gradle.org/8.3/userguide/installation.html#windows_installation.

## Building Project
### Intellij Setup
<ul>
<li> In Intellij go to <b> File > Open </b> and then select the directory
where you cloned the git project.</li>
<li>Go to <b>File > Project Structure</b> and under project set the SDK level to 17
 (the JDK that you installed).</li>
</ul>

### Android Studio Setup
<ul>
<li> In Android Studio go to <b> File > Open </b> and then select the directory
where you cloned the git project.</li>
<li> Go to <b>File > Project Structure</b>. Select <b>SDK Location</b>, then click
the blue link that says <b>JDK location was moved to Gradle Settings</b> at the bottom.
In Gradle Settings, set the Grade JDK to JAVA_HOME (the variable that you set during
Java installation).
</li>
</ul>

## Building Gradle
Open the terminal in your IDE and make sure you are in the path FtcRobotController.
Then run the command:

<code> .\gradlew build </code>

Once the build is completed, <span style = "color: green;"> BUILD SUCCESSFUL </span>
should show.

## Creating Pull Requests
Follow these steps to make changes to the project.
<ol>
<li><b>Ensure presence in the master branch:</b> In the terminal, run
<code>git checkout master</code>.</li>
<li><b>Get the latest from git remote:</b> Run <code> git pull</code></li>
<li><b>Create your own branch to make changes:</b> Run 
<code>git checkout -b &lt;your branch name&gt; </code></li>
<li><b>Make changes.</b></li>
<li><b>Commit to your branch locally often:</b> Run the command
<code>git commit -am "&lt;commit message&gt;"</code></li>
<li><b>Build project:</b> Once you are finished making changes, run
<code>\.gradlew build</code>.</li>
<li><b>Fix any errors that come up:</b> After you have fixed all the issues,
 commit again.</li>
<li><b>Push changes to remote:</b> Run <code>git push --set-upstream origin &lt;your branch name&gt;</code></li>
<li><b>You can push as many times as you want:</b> If more changes are necessary
for your branch, follow the steps starting from step 4 through 7. When the code is ready
to push, run <code>git push</code>. Note: There is no need to set upstream as you did in step eight
because it is already set.</li>
<li><b>Create pull request:</b> Go to this repository on github.com and click on <span style = "background-color:green; color:white;"> Compare & pull
request</span>.</li>
<li><b>Finish:</b> Make sure all the changed files are present, then click <span style = "background-color:green; color:white;"> Create Pull Request </span></li>
</ol>