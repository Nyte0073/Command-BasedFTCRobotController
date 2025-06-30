package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import static org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.SwerveDriveFields.stopMotorsIsRunning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Swerve;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.SwerveDriveFields;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.SwerveVector;

import java.util.Arrays;

public class ThreadBasedSwerveDrive extends Swerve {

    Runnable[] runnables = new Runnable[] {
            () -> {},
            () -> {},
            () -> {},
            () -> {},
            () -> {}
    };

    final Object driveLock = new Object();

    final SwerveDriveFields swerveDriveFields = new SwerveDriveFields();

    volatile SwerveState swerveState = SwerveState.IDLE;
    volatile CompleteRotations rotations = CompleteRotations.ROTATION_WAS_NOT_DONE;

    Thread[] threads = {
            new Thread( //Driving thread.
                    () -> {
                        while(!Thread.currentThread().isInterrupted()) {
                            synchronized (driveLock) {
                                while(swerveState != SwerveState.DRIVE && swerveState != SwerveState.IDLE) {
                                    try {
                                        driveLock.wait();
                                    } catch(Exception e) {
                                        Thread.currentThread().interrupt();
                                        break;
                                    }
                                }
                                    driveLock.notifyAll();
                                    runnables[0].run(); //Running the applyFieldOrientedSwerve/applyRobotOrientedSwerve methods.
                                    runnables[1].run(); //Running the setPower methods.
                                    swerveState = SwerveState.IDLE;
                            }

                            try {
                                Thread.sleep(25);
                            } catch(Exception e) {
                                Thread.currentThread().interrupt();
                                break;
                            }
                        }
                    }
            ),

            new Thread( //Complete rotate thread.
                    () -> {
                        while(!Thread.currentThread().isInterrupted()) {
                            synchronized (driveLock) {
                                while(swerveState != SwerveState.COMPLETE_ROTATE && swerveState != SwerveState.IDLE) {
                                    try {
                                        driveLock.wait();
                                    } catch(Exception e) {
                                        Thread.currentThread().interrupt();
                                        break;
                                    }
                                }

                                    driveLock.notifyAll();
                                    runnables[2].run(); //Runs the completeRotate method.
                                    runnables[3].run(); //Runs the setPowerForCompleteRotate method.
                                    swerveState = SwerveState.IDLE;
                                    rotations = CompleteRotations.ROTATION_WAS_DONE;
                            }

                            try {
                                Thread.sleep(25);
                            } catch(Exception e) {
                                Thread.currentThread().interrupt();
                            }
                        }
                    }
            ),

            new Thread( //Reset wheel thread.
                    () -> {
                        while(!Thread.currentThread().isInterrupted()) {
                            synchronized (driveLock) {

                                while(swerveState != SwerveState.RESET_WHEEL && swerveState != SwerveState.IDLE) {
                                   try {
                                       driveLock.wait();
                                   } catch(Exception e) {
                                       Thread.currentThread().interrupt();
                                       break;
                                   }
                                }

                                    /*For tis specific runnable below, you need to make sure that the code for the resetWheelHeading()
                                     * method inside this runnable is updated WITHIN THE THREAD ITSELF. It is because these values can change
                                     * very quickly and you need them to be EXACTLY up to date for the code in this runnable to work
                                     * effectively.*/

                                        driveLock.notifyAll();
                                        runnables[4].run(); //This method needs to be constantly updated by the program.
                                swerveState = SwerveState.IDLE;
                            }

                            try {
                                Thread.sleep(25);
                            } catch(Exception e) {
                                Thread.currentThread().interrupt();
                                break;
                            }
                        }
                    }
            )
    };

    enum SwerveState {
        IDLE,
        DRIVE,
        COMPLETE_ROTATE,
        RESET_WHEEL;
    }

    enum CompleteRotations {
        ROTATION_WAS_DONE,
        ROTATION_WAS_NOT_DONE;
    }


    public ThreadBasedSwerveDrive(Telemetry telemetry, Motor[] turningMotors, Motor[] drivingMotors, IMU imu, GamepadEx gamepadEx, boolean fieldOriented) {
        swerveDriveFields.telemetry = telemetry;
        swerveDriveFields.turningMotors = turningMotors;
        swerveDriveFields.drivingMotors = drivingMotors;
        swerveDriveFields.imu = imu;
        swerveDriveFields.gamepadEx = gamepadEx;
        swerveDriveFields.fieldOriented = fieldOriented;

        for(Motor m : turningMotors) {
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            m.stopAndResetEncoder();
            m.setRunMode(Motor.RunMode.PositionControl);
        }

        for(Motor m : drivingMotors) {
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        for(Thread thread : threads) {
            thread.start();
        }
    }

    @Override
    public void setSwerveModuleState(boolean fieldOriented, double forwardPower, double sidePower, int headingInDegrees, double turningVector, boolean turningLeft) {
        //If the robot wants to drive field-oriented, drive field oriented, otherwise robot-oriented.

        if(stopMotorsIsRunning) { //If this boolean is true, stop motors completely.
            stop();
            return;
        }

        int heading = (Math.abs(forwardPower) <= 0.01 && Math.abs(sidePower) <= 0.01) ? 0 :
                (int) Math.toDegrees(Math.atan2(forwardPower, sidePower)) - 90;

        if(fieldOriented) { //If the robot wants to drive field-oriented, drive field-oriented. Otherwise, not.
            applyFieldOrientedSwerve(heading, forwardPower, sidePower, headingInDegrees, turningVector, turningLeft);
        } else {
            applyRobotOrientedSwerve(heading, forwardPower, sidePower, turningVector, turningLeft);
        }
    }

    public void stopThreads() {
        for(Thread thread : threads) {
            thread.interrupt();
        }
    }

    public void applyFieldOrientedSwerve(int heading, double forwardPower, double sidePower, int headingInDegrees, double turningVector, boolean turningLeft) {
        if(Math.abs(turningVector) >= 0.05 && swerveState == SwerveState.IDLE) {
            runnables[2] = () -> completeRotate(turningLeft, headingInDegrees, turningVector, true);
            runnables[3] = () ->
                    setPowerForCompleteRotate(turningLeft, swerveDriveFields.headingsReversed, turningVector, new int[] {
                        swerveDriveFields.headingsReversed[0] ? swerveDriveFields.reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0],
                        swerveDriveFields.headingsReversed[1] ? swerveDriveFields.reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1],
                        swerveDriveFields.headingsReversed[2] ? swerveDriveFields.reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2],
                        swerveDriveFields.headingsReversed[3] ? swerveDriveFields.reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3]
                }, true);
            swerveState = SwerveState.COMPLETE_ROTATE;
        } else if(rotations == CompleteRotations.ROTATION_WAS_DONE && swerveState == SwerveState.IDLE) {
           runnables[4] = this::resetWheelHeading;
           swerveState = SwerveState.RESET_WHEEL;
        }
        //Implement regular driving operations.

        runnables[0] = () -> {
            swerveDriveFields.forwardVector = Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;

            swerveDriveFields.normalizedHeading = normalizeHeading(headingInDegrees, heading);

            swerveDriveFields.totalHeadingFrontLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[0], swerveDriveFields.normalizedHeading, 0);
            swerveDriveFields.reversedHeadingFrontLeft = calculateReverseHeading(swerveDriveFields.totalHeadingFrontLeft, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[0]);

            swerveDriveFields.totalHeadingFrontRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[1], swerveDriveFields.normalizedHeading, 1);
            swerveDriveFields.reversedHeadingFrontRight =  calculateReverseHeading(swerveDriveFields.totalHeadingFrontRight, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[1]);

            swerveDriveFields.totalHeadingBackLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[2], swerveDriveFields.normalizedHeading, 2);
            swerveDriveFields.reversedHeadingBackLeft = calculateReverseHeading(swerveDriveFields.totalHeadingBackLeft, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[2]);

            swerveDriveFields.totalHeadingBackRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[3], swerveDriveFields.normalizedHeading, 3);
            swerveDriveFields.reversedHeadingBackRight = calculateReverseHeading(swerveDriveFields.totalHeadingBackRight, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[3]);

            /*If the heading of a certain wheel on the robot has been reversed to reach the desired target angle, then a boolean corresponding to the wheel in the boolean
             * array 'headingsReversed' wil be set to true. Else, that boolean will be set false, and so forth will occur again and again for the other wheels.*/
            swerveDriveFields.headingsReversed[0] = swerveDriveFields.reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
            swerveDriveFields.headingsReversed[1] = swerveDriveFields.reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
            swerveDriveFields.headingsReversed[2] = swerveDriveFields.reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
            swerveDriveFields.headingsReversed[3] = swerveDriveFields.reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

            swerveDriveFields.individualTargetPositions[0] = swerveDriveFields.headingsReversed[0] ? swerveDriveFields.reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0];
            swerveDriveFields.individualTargetPositions[1] = swerveDriveFields.headingsReversed[1] ? swerveDriveFields.reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1];
            swerveDriveFields.individualTargetPositions[2] = swerveDriveFields.headingsReversed[2] ? swerveDriveFields.reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2];
            swerveDriveFields.individualTargetPositions[3] = swerveDriveFields.headingsReversed[3] ? swerveDriveFields.reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3];

            swerveDriveFields.turningMotors[0].setTargetPosition(swerveDriveFields.individualTargetPositions[0]);
            swerveDriveFields.turningMotors[1].setTargetPosition(swerveDriveFields.individualTargetPositions[1]);
            swerveDriveFields.turningMotors[2].setTargetPosition(swerveDriveFields.individualTargetPositions[2]);
            swerveDriveFields.turningMotors[3].setTargetPosition(swerveDriveFields.individualTargetPositions[3]);

            swerveDriveFields.individualWheelHeadings[0] = swerveDriveFields.headingsReversed[0] ? swerveDriveFields.individualWheelHeadings[0] + swerveDriveFields.reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0];
            swerveDriveFields.individualWheelHeadings[1] = swerveDriveFields.headingsReversed[1] ? swerveDriveFields.individualWheelHeadings[1] + swerveDriveFields.reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1];
            swerveDriveFields.individualWheelHeadings[2] = swerveDriveFields.headingsReversed[2] ? swerveDriveFields.individualWheelHeadings[2] + swerveDriveFields.reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2];
            swerveDriveFields.individualWheelHeadings[3] = swerveDriveFields.headingsReversed[3] ? swerveDriveFields.individualWheelHeadings[3] + swerveDriveFields.reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3];

        /*If the physical sign of a certain wheel heading is negative, a boolean corresponding to that wheel in the boolean array 'headingsNegativeOrNot' will be set to true,
         otherwise false. This will occur for all four wheels.*/
            swerveDriveFields.headingsNegativeOrNot[0] = swerveDriveFields.individualTargetPositions[0] != Math.abs(swerveDriveFields.individualTargetPositions[0]);
            swerveDriveFields.headingsNegativeOrNot[1] = swerveDriveFields.individualTargetPositions[1] != Math.abs(swerveDriveFields.individualTargetPositions[1]);
            swerveDriveFields.headingsNegativeOrNot[2] = swerveDriveFields.individualTargetPositions[2] != Math.abs(swerveDriveFields.individualTargetPositions[2]);
            swerveDriveFields.headingsNegativeOrNot[3] = swerveDriveFields.individualTargetPositions[3] != Math.abs(swerveDriveFields.individualTargetPositions[3]);
        };

        runnables[1] = () -> setPower(swerveDriveFields.headingsNegativeOrNot, swerveDriveFields.forwardVector);
        swerveState = SwerveState.DRIVE;
    }


    public int calculateTotalHeading(int currentHeading, int targetHeading, int wheelNumber) {
        int normalizedHeadingForWheel = normalizeHeading(currentHeading, targetHeading);
        swerveDriveFields.individualTargetPositions[wheelNumber] = normalizedHeadingForWheel;
        return currentHeading + normalizedHeadingForWheel;
    }


    public int calculateReverseHeading(int totalHeading, int normalizedHeading, int wheelHeading) {
        return Math.abs(totalHeading) > 180 ? normalizeHeading(wheelHeading,
                normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180) : Constants.SwerveConstants.NO_REVERSAL;
    }

    public void applyRobotOrientedSwerve(int heading, double forwardPower, double sidePower, double turningVector, boolean turningLeft) {
        if(Math.abs(turningVector) >= 0.05 && swerveState == SwerveState.IDLE) {
            runnables[2] = () -> completeRotate(turningLeft, 0, turningVector, false);
            runnables[3] = () ->
                setPowerForCompleteRotate(turningLeft, swerveDriveFields.headingsReversed, turningVector, new int[] {
                        swerveDriveFields.headingsReversed[0] ? swerveDriveFields.reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0],
                        swerveDriveFields.headingsReversed[1] ? swerveDriveFields.reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1],
                        swerveDriveFields.headingsReversed[2] ? swerveDriveFields.reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2],
                        swerveDriveFields.headingsReversed[3] ? swerveDriveFields.reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3]
                }, true);
            swerveState = SwerveState.COMPLETE_ROTATE;
        } else if(rotations == CompleteRotations.ROTATION_WAS_DONE && swerveState == SwerveState.IDLE) {
            runnables[4] = this::resetWheelHeading;
            swerveState = SwerveState.RESET_WHEEL;
        }

        //Implement regular driving operations.

        runnables[0] = () -> {
            swerveDriveFields.forwardVector = Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;

            swerveDriveFields.normalizedHeading = normalizeHeading(0, heading);

            swerveDriveFields.totalHeadingFrontLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[0], swerveDriveFields.normalizedHeading, 0);
            swerveDriveFields.reversedHeadingFrontLeft = calculateReverseHeading(swerveDriveFields.totalHeadingFrontLeft, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[0]);

            swerveDriveFields.totalHeadingFrontRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[1], swerveDriveFields.normalizedHeading, 1);
            swerveDriveFields.reversedHeadingFrontRight =  calculateReverseHeading(swerveDriveFields.totalHeadingFrontRight, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[1]);

            swerveDriveFields.totalHeadingBackLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[2], swerveDriveFields.normalizedHeading, 2);
            swerveDriveFields.reversedHeadingBackLeft = calculateReverseHeading(swerveDriveFields.totalHeadingBackLeft, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[2]);

            swerveDriveFields.totalHeadingBackRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[3], swerveDriveFields.normalizedHeading, 3);
            swerveDriveFields.reversedHeadingBackRight = calculateReverseHeading(swerveDriveFields.totalHeadingBackRight, swerveDriveFields.normalizedHeading, swerveDriveFields.individualWheelHeadings[3]);

            /*If the heading of a certain wheel on the robot has been reversed to reach the desired target angle, then a boolean corresponding to the wheel in the boolean
             * array 'headingsReversed' wil be set to true. Else, that boolean will be set false, and so forth will occur again and again for the other wheels.*/
            swerveDriveFields.headingsReversed[0] = swerveDriveFields.reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
            swerveDriveFields.headingsReversed[1] = swerveDriveFields.reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
            swerveDriveFields.headingsReversed[2] = swerveDriveFields.reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
            swerveDriveFields.headingsReversed[3] = swerveDriveFields.reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

            swerveDriveFields.individualTargetPositions[0] = swerveDriveFields.headingsReversed[0] ? swerveDriveFields.reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0];
            swerveDriveFields.individualTargetPositions[1] = swerveDriveFields.headingsReversed[1] ? swerveDriveFields.reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1];
            swerveDriveFields.individualTargetPositions[2] = swerveDriveFields.headingsReversed[2] ? swerveDriveFields.reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2];
            swerveDriveFields.individualTargetPositions[3] = swerveDriveFields.headingsReversed[3] ? swerveDriveFields.reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3];

            swerveDriveFields.turningMotors[0].setTargetPosition(swerveDriveFields.individualTargetPositions[0]);
            swerveDriveFields.turningMotors[1].setTargetPosition(swerveDriveFields.individualTargetPositions[1]);
            swerveDriveFields.turningMotors[2].setTargetPosition(swerveDriveFields.individualTargetPositions[2]);
            swerveDriveFields.turningMotors[3].setTargetPosition(swerveDriveFields.individualTargetPositions[3]);

            swerveDriveFields.individualWheelHeadings[0] = swerveDriveFields.headingsReversed[0] ? swerveDriveFields.individualWheelHeadings[0] + swerveDriveFields.reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0];
            swerveDriveFields.individualWheelHeadings[1] = swerveDriveFields.headingsReversed[1] ? swerveDriveFields.individualWheelHeadings[1] + swerveDriveFields.reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1];
            swerveDriveFields.individualWheelHeadings[2] = swerveDriveFields.headingsReversed[2] ? swerveDriveFields.individualWheelHeadings[2] + swerveDriveFields.reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2];
            swerveDriveFields.individualWheelHeadings[3] = swerveDriveFields.headingsReversed[3] ? swerveDriveFields.individualWheelHeadings[3] + swerveDriveFields.reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3];

        /*If the physical sign of a certain wheel heading is negative, a boolean corresponding to that wheel in the boolean array 'headingsNegativeOrNot' will be set to true,
         otherwise false. This will occur for all four wheels.*/
            swerveDriveFields.headingsNegativeOrNot[0] = swerveDriveFields.individualTargetPositions[0] != Math.abs(swerveDriveFields.individualTargetPositions[0]);
            swerveDriveFields.headingsNegativeOrNot[1] = swerveDriveFields.individualTargetPositions[1] != Math.abs(swerveDriveFields.individualTargetPositions[1]);
            swerveDriveFields.headingsNegativeOrNot[2] = swerveDriveFields.individualTargetPositions[2] != Math.abs(swerveDriveFields.individualTargetPositions[2]);
            swerveDriveFields.headingsNegativeOrNot[3] = swerveDriveFields.individualTargetPositions[3] != Math.abs(swerveDriveFields.individualTargetPositions[3]);
        };

        runnables[1] = () -> setPower(swerveDriveFields.headingsNegativeOrNot, swerveDriveFields.forwardVector);
        swerveState = SwerveState.DRIVE;
    }

    @Override
    public void stopMotors() {
        for(Motor m : swerveDriveFields.turningMotors) {
            m.set(0);
        }

        for(Motor m : swerveDriveFields.drivingMotors) {
            m.set(0);
        }
    }
    @Override
    public void resetWheelHeading() {
        int botHeading = (int) Math.round(swerveDriveFields.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        int totalHeadingFrontLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[0], botHeading, 0);
        int reversedHeadingFrontLeft = calculateReverseHeading(totalHeadingFrontLeft, botHeading, swerveDriveFields.individualWheelHeadings[0]);

        int totalHeadingFrontRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[1], botHeading, 1);
        int reversedHeadingFrontRight = calculateReverseHeading(totalHeadingFrontRight, botHeading, swerveDriveFields.individualWheelHeadings[1]);

        int totalHeadingBackLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[2], botHeading, 2);
        int reversedHeadingBackLeft = calculateReverseHeading(totalHeadingBackLeft, botHeading, swerveDriveFields.individualWheelHeadings[2]);

        int totalHeadingBackRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[3], botHeading, 3);
        int reversedHeadingBackRight = calculateReverseHeading(totalHeadingBackRight, botHeading, swerveDriveFields.individualWheelHeadings[3]);

        swerveDriveFields.headingsReversed[0] = reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
        swerveDriveFields.headingsReversed[1] = reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
        swerveDriveFields.headingsReversed[2] = reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
        swerveDriveFields.headingsReversed[3] = reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

        swerveDriveFields.individualWheelHeadings[0] = swerveDriveFields.headingsReversed[0] ? swerveDriveFields.individualWheelHeadings[0] + reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0];
        swerveDriveFields.individualWheelHeadings[1] = swerveDriveFields.headingsReversed[1] ? swerveDriveFields.individualWheelHeadings[1] + reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1];
        swerveDriveFields.individualWheelHeadings[2] = swerveDriveFields.headingsReversed[2] ? swerveDriveFields.individualWheelHeadings[2] + reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2];
        swerveDriveFields.individualWheelHeadings[3] = swerveDriveFields.headingsReversed[3] ? swerveDriveFields.individualWheelHeadings[3] + reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3];

        swerveDriveFields.turningMotors[0].setTargetPosition(swerveDriveFields.headingsReversed[0] ? reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0]);
        swerveDriveFields.turningMotors[1].setTargetPosition(swerveDriveFields.headingsReversed[1] ? reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1]);
        swerveDriveFields.turningMotors[2].setTargetPosition(swerveDriveFields.headingsReversed[2] ? reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2]);
        swerveDriveFields.turningMotors[3].setTargetPosition(swerveDriveFields.headingsReversed[3] ? reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3]);

        for(int i = 0; i < swerveDriveFields.turningMotors.length; i++) {
            swerveDriveFields.turningMotors[i].set(swerveDriveFields.headingsNegativeOrNot[i] ? -1 : 1);
        }

        while(!allWheelsHaveRotatedToPosition()) {
            try {
                Thread.sleep(10);
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
        }

        for(Motor m : swerveDriveFields.turningMotors) {
            m.set(0);
        }

        Arrays.fill(swerveDriveFields.wheelsHaveRotated, false);
        swerveDriveFields.alreadyRotated = false;
    }

    @Override
    public void completeRotate(boolean turningLeft, int imuHeadingInDegrees, double turnVector, boolean fieldOriented) {
        if(swerveDriveFields.alreadyRotated) {
            setPowerForCompleteRotate(swerveDriveFields.previousTurningLeft, swerveDriveFields.headingsReversed, swerveDriveFields.previousTurningVector, new int[] {}, false);
            return;
        }

        swerveDriveFields.previousTurningLeft = turningLeft;

        int headingFLBR = -45, headingFRBL = 45;

        int totalHeadingFrontLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[0], headingFLBR, 0);
        int reversedHeadingFrontLeft = calculateReverseHeading(totalHeadingFrontLeft, headingFLBR, swerveDriveFields.individualWheelHeadings[0]);

        int totalHeadingFrontRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[1], headingFRBL, 1);
        int reversedHeadingFrontRight = calculateReverseHeading(totalHeadingFrontRight, headingFRBL, swerveDriveFields.individualWheelHeadings[1]);

        int totalHeadingBackLeft = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[2], headingFLBR, 2);
        int reversedHeadingBackLeft = calculateReverseHeading(totalHeadingBackLeft, headingFLBR, swerveDriveFields.individualWheelHeadings[2]);

        int totalHeadingBackRight = calculateTotalHeading(swerveDriveFields.individualWheelHeadings[3], headingFRBL, 3);
        int reversedHeadingBackRight = calculateReverseHeading(totalHeadingBackRight, headingFRBL, swerveDriveFields.individualWheelHeadings[3]);

        swerveDriveFields.headingsReversed[0] = reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
        swerveDriveFields.headingsReversed[1] = reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
        swerveDriveFields.headingsReversed[2] = reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
        swerveDriveFields.headingsReversed[3] = reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

        swerveDriveFields.individualWheelHeadings[0] = swerveDriveFields.headingsReversed[0] ? swerveDriveFields.individualWheelHeadings[0] + reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0];
        swerveDriveFields.individualWheelHeadings[1] = swerveDriveFields.headingsReversed[1] ? swerveDriveFields.individualWheelHeadings[1] + reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1];
        swerveDriveFields.individualWheelHeadings[2] = swerveDriveFields.headingsReversed[2] ? swerveDriveFields.individualWheelHeadings[2] + reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2];
        swerveDriveFields.individualWheelHeadings[3] = swerveDriveFields.headingsReversed[3] ? swerveDriveFields.individualWheelHeadings[3] + reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3];

        swerveDriveFields.previousTargetPositions[0] = swerveDriveFields.headingsReversed[0] ? reversedHeadingFrontLeft : swerveDriveFields.individualTargetPositions[0];
        swerveDriveFields.previousTargetPositions[1] = swerveDriveFields.headingsReversed[1] ? reversedHeadingFrontRight : swerveDriveFields.individualTargetPositions[1];
        swerveDriveFields.previousTargetPositions[2] = swerveDriveFields.headingsReversed[2] ? reversedHeadingBackLeft : swerveDriveFields.individualTargetPositions[2];
        swerveDriveFields.previousTargetPositions[3] = swerveDriveFields.headingsReversed[3] ? reversedHeadingBackRight : swerveDriveFields.individualTargetPositions[3];
    }

    @Override
    public int normalizeHeading(int currentPosition, int targetPosition) {
        return (targetPosition - currentPosition + 540) % 360 - 180;
    }

    @Override
    public void setPowerForCompleteRotate(boolean turningLeft, boolean[] headingsReversed, double turningVector, int[] targetPositions, boolean goToPosition) {
        if(goToPosition) {
            swerveDriveFields.turningMotors[0].setTargetPosition(targetPositions[0]);
            swerveDriveFields.turningMotors[1].setTargetPosition(targetPositions[1]);
            swerveDriveFields.turningMotors[2].setTargetPosition(targetPositions[2]);
            swerveDriveFields.turningMotors[3].setTargetPosition(targetPositions[3]);

            setPowerToIndividualWheel(0, headingsReversed[0]);
            setPowerToIndividualWheel(1, headingsReversed[1]);
            setPowerToIndividualWheel(2, headingsReversed[2]);
            setPowerToIndividualWheel(3, headingsReversed[3]);

            while(!allWheelsHaveRotatedToPosition()) {
                try {
                    Thread.sleep(10);
                } catch(Exception e) {
                    throw new RuntimeException(e);
                }
            }
        }

        swerveDriveFields.drivingMotors[0].set(headingsReversed[0] ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
        swerveDriveFields.drivingMotors[1].set(headingsReversed[1] ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));
        swerveDriveFields.drivingMotors[2].set(headingsReversed[2] ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
        swerveDriveFields.drivingMotors[3].set(headingsReversed[3] ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));

        Arrays.fill(swerveDriveFields.wheelsHaveRotated, false);
        swerveDriveFields.alreadyRotated = true;
    }

    @Override
    public void setPower(boolean[] headingIsNegative,  double forwardVector) {
        setPowerToIndividualWheel(0, swerveDriveFields.headingsReversed[0]);
        setPowerToIndividualWheel(1, swerveDriveFields.headingsReversed[1]);
        setPowerToIndividualWheel(2, swerveDriveFields.headingsReversed[2]);
        setPowerToIndividualWheel(3, swerveDriveFields.headingsReversed[3]);

        while(!allWheelsHaveRotatedToPosition()) {
            try {
                Thread.sleep(10);
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
        }

        for(int i = 0; i < swerveDriveFields.turningMotors.length; i++) {
            swerveDriveFields.drivingMotors[i].set(swerveDriveFields.headingsReversed[i] ? -forwardVector : forwardVector);
        }

        Arrays.fill(swerveDriveFields.wheelsHaveRotated, false);
    }

    public boolean allWheelsHaveRotatedToPosition() {
        return swerveDriveFields.wheelsHaveRotated[0]
                && swerveDriveFields.wheelsHaveRotated[1] &&
                swerveDriveFields.wheelsHaveRotated[2] &&
                swerveDriveFields.wheelsHaveRotated[3];
    }

    public void setPowerToIndividualWheel(int wheelNumber, boolean headingIsNegative) {
        swerveDriveFields.turningMotors[wheelNumber].set(headingIsNegative ? -1 : 1);
        while(!swerveDriveFields.turningMotors[wheelNumber].atTargetPosition()) {
            try {
                Thread.sleep(10);
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
        }
        swerveDriveFields.turningMotors[wheelNumber].set(0);
        swerveDriveFields.wheelsHaveRotated[wheelNumber] = true;
    }

    @Override
    public SwerveVector getRobotVector() {
        return new SwerveVector((int) Math.round(swerveDriveFields.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)),
                -swerveDriveFields.gamepadEx.getLeftY(), swerveDriveFields.gamepadEx.getLeftX(), swerveDriveFields.gamepadEx.getRightX(), swerveDriveFields.fieldOriented, swerveDriveFields.gamepadEx.getRightX() != Math.abs(swerveDriveFields.gamepadEx.getRightX()));
    }

    @Override
    public void periodic() {
        swerveDriveFields.telemetry.update();
    }
}
