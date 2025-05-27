package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;

public class RobotVector {
    public int imuHeadingInDegrees;
    public double forwardPower, sidePower, turningVector;
   public  boolean fieldOriented, turningLeft;

    public RobotVector(int imuHeadingInDegrees, double forwardPower, double sidePower, double turningVector, boolean fieldOriented, boolean turningLeft) {
        this.imuHeadingInDegrees = imuHeadingInDegrees;
        this.forwardPower  = forwardPower;
        this.sidePower = sidePower;
        this.turningVector = turningVector;
        this.fieldOriented = fieldOriented;
        this.turningLeft = turningLeft;
    }

    public double degreesToRadians() {
        return Math.toRadians(imuHeadingInDegrees);
    }

    public double calculateForwardVector() {
        return Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;
    }


}
