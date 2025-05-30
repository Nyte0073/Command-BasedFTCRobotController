package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;

public class RobotVector {
    /**The IMU orientation of the robot, in degrees.*/
    public int imuHeadingInDegrees;
    /**The forward power (y-coordinate) driver input from the driver's controller.*/
    public double forwardPower,
    /**The side power (x-coordinate) input from the driver's controller.*/
    sidePower,
    /**The power the human driver wants the robot to rotate left/right at.*/
    turningVector;
    /**Boolean state of whether the robot will drive field-oriented or robot-oriented.*/
    public boolean fieldOriented,
    /**Boolean state of whether the robot is rotating left or right on the spot.*/
    turningLeft;

    /**Constructs a new {@code RobotVector()} with initialized {@code imuHeadingInDegrees}, {@code forwardPower}, {@code sidePower},
     * {@code turningVector}, {@code fieldOriented} and {@code turningLeft} variables.*/
    public RobotVector(int imuHeadingInDegrees, double forwardPower, double sidePower, double turningVector, boolean fieldOriented, boolean turningLeft) {
        this.imuHeadingInDegrees = imuHeadingInDegrees;
        this.forwardPower  = forwardPower;
        this.sidePower = sidePower;
        this.turningVector = turningVector;
        this.fieldOriented = fieldOriented;
        this.turningLeft = turningLeft;
    }

    /**Returns value of the {@code imuHeadingInDegrees} variable, but in RADIANS form.*/
    public double degreesToRadians() {
        return Math.toRadians(imuHeadingInDegrees);
    }

    /**Calculates and returns the motor power to give the driving motor depending on the values of {@code forwardPower} and
     * {@code sidePower} and their relationship.*/
    public double calculateForwardVector() {
        return Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;
    }

    public double getForwardPower() {
        return forwardPower;
    }

    public double getSidePower() {
        return sidePower;
    }

    public double getTurningVector() {
        return turningVector;
    }

    public int getImuHeadingInDegrees() {
        return imuHeadingInDegrees;
    }

    public boolean getFieldOriented() {
        return fieldOriented;
    }

    public boolean getTurningLeft() {
        return turningLeft;
    }
}
