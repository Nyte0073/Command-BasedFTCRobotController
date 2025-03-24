
# Instructions For Using This Repository For FTC Programming

Hello, and thank you for downloading this FTC repository. In this README file, you will learn the necessary things to be able to use 
this repository for programming your FTC bot and how to cope and to deal with all the unique Java functions, methods and classes specific to the
FTC SDK. And with that, let's begin.

# Setting up the repository

The FTC SDK built-into this FTC repository uses specific gradle versions, so you have to modify your Gradle settings in Android Studio and also
your gradle settings in some of your Gradle Files located in your Gradle Scripts area to be able to use the right version of the FTC Android Studio that
is compatible with your robot and its control hub.

# Downloading the correct version of Java

The FTC SDK uses Java 8 and a gradle classpath version 7.3.0 for successfully doing project builds, and these versions work the best with Java version 17,
which is exactly the version of Java that I use with this setup. Your computer won't come with Java 17 downloaded automatically, so you will have to 
download it externally. Here is a link to a website the that will allow to directly download the Java 17 JDK: https://aka.ms/download-jdk/microsoft-jdk-17.0.14-windows-x64.zip.

That download is for Windows, but if your using any other operating system, use this link and choose the correct ZIP file to download for your operating
system. And remember, pick from the ones with the x64 installer. Once you have the zip file downloaded, extract it into a folder and then remember the path
of the JDK's location because you will need it in a minute.

Once you have the folder extracted, go into and Android Studio and tap on the Settings option (the gear icon), and go to 
BUILD, EXECUTION, DEPLOYMENT > BUILD TOOLS > GRADLE, and then in the "Gradle JDK" option, select the path to where you extracted the JDK for Java 17.
Then, once you have done that, click "Ok".

# Selecting the right gradle version

Right now, if you go to Gradle Scripts in your project ToolBar area, and you click on build.gradle (Project: FtcRobotController-master), you should that the 
value of the classpath variable is set to 7.3.0. If the classpath value is not set to that number, then please change it to that number.

Your classpath variable should look like this:

`classpath 'com.android.tools.build:gradle:7.3.0'`

# Selecting the right wrapper-properties gradle version

Right now, if you go gradle.wrapper-properties in your Gradle Scripts and then check the value of the "distributionUrl" property, then the value of the bin
gradle version should be 7.4.2. If the value of the property is not 7.4.2, then please change it that number. Your "distributionUrl" property should
look like this:

`distributionUrl=https\://services.gradle.org/distributions/gradle-7.4.2-bin.zip`

After you have done all these things successfully, you can then re-sync your gradle to apply all the changes you have made. And with that, you should be good
to go!




