package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class HolonomicVector {
    HDrive drivetrain;

    Motor[] HDriveMotors, XDriveMotors;

    double leftAngle, rightAngle, slideAngle;

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
