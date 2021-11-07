
## Caution
No Team-specific code should be placed or modified in this ``.../samples`` folder.

Samples should be Copied from here, and then Pasted into the team's 
[/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](../../../../../../../../../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode)
 folder, using the Android Studio cut and paste commands.  This automatically changes all file and
class names to be consistent.  From there, the sample can be modified to suit the team's needs.

For more detailed instructions see the /teamcode readme.  

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Hardware:	This is NOT an OpMode, but a helper class that is used to describe
            one particular robot's hardware configuration:   eg: For the K9 or Pushbot.
            Look at any Pushbot sample to see how this can be used in an OpMode.
            Teams can copy one of these to their team folder to create their own robot definition.

Pushbot:	This is a Sample OpMode that uses the Pushbot robot hardware as a base.
            It may be used to provide some standard baseline Pushbot OpModes, or
            to demonstrate how a particular sensor or concept can be used directly on the
            Pushbot chassis.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot. 

Library:    This is a class, or set of classes used to implement some strategy.
            These will typically NOT implement a full OpMode.  Instead they will be included
            by an OpMode to provide some stand-alone capability.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Hardware class names are constructed as:  Hardware - Robot type
* Pushbot class names are constructed as:   Pushbot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype
* Library class names are constructed as:   Library - Topic - OpModetype

