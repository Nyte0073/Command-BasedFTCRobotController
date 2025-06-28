package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class HolonomicVector {

    /**The reference to the robot's holonomic drivetrain.*/
    HDrive drivetrain;

    /**The different {@code Motor} arrays for initializing {@code Motor}'s for if the robot will be driving using an
     * X-pattern or H-pattern holonomic drive.*/
    Motor[] HDriveMotors, XDriveMotors;

    /**The angles that the wheels on a H-pattern holonomic drive
     * will be positioned at on the robot.*/
    double leftAngle, rightAngle, slideAngle;

    /**Constructs a new {@code HolonomicVector()} with initialized {@code Motor}'s for the H-pattern and X-pattern holonomic drive,
     * and the initialized left, right and slide {@code Motor} angles. This constructor also initializes the {@code drivetrain} object to fit
     * an H-pattern or X-patter holonomic drivetrain depending on the value of the {@code driveType} String parameter.*/
    public HolonomicVector(Motor[] HDriveMotors, Motor[] XDriveMotors, double leftAngle, double rightAngle, double slideAngle, String driveType) {
        this.HDriveMotors = HDriveMotors;
        this.XDriveMotors = XDriveMotors;
        this.leftAngle = leftAngle;
        this.rightAngle = rightAngle;
        this.slideAngle = slideAngle;

        switch(driveType) {
            case "HDrive":
                drivetrain = new HDrive(HDriveMotors[0], HDriveMotors[1], HDriveMotors[2], leftAngle, rightAngle, slideAngle);
                break;

            case "XDrive":
                drivetrain = new HDrive(XDriveMotors[0], XDriveMotors[1], XDriveMotors[2], XDriveMotors[3]);
                break;
        }
    }
}
