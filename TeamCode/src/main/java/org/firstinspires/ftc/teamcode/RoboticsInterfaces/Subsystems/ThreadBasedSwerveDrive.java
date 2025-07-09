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
        RESET_WHEEL
    }

    enum CompleteRotations {
        ROTATION_WAS_DONE,
        ROTATION_WAS_NOT_DONE
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
                    setPowerForCompleteRotate(turningLeft, swerveDriveFields.headingsReversed, turningVector,
                            swerveDriveFields.individualTargetPositions, true, swerveDriveFields.headingsNegativeOrNot);
            swerveState = SwerveState.COMPLETE_ROTATE;
        } else if(rotations == CompleteRotations.ROTATION_WAS_DONE && swerveState == SwerveState.IDLE) {
           runnables[4] = this::resetWheelHeading;
           swerveState = SwerveState.RESET_WHEEL;
        }
        //Implement regular driving operations.

        runnables[0] = () -> {
           swerveDriveFields.forwardVector = Math.hypot(sidePower, forwardPower) / Constants.SwerveConstants.vectorScalar;
           swerveDriveFields.normalizedHeading = normalizeHeading(headingInDegrees, heading);

           for(int i = 0; i < swerveDriveFields.turningMotors.length; i++) {
               int normalizedHeadingForWheel = normalizeHeading(swerveDriveFields.individualWheelHeadings[i], swerveDriveFields.normalizedHeading);
               int totalHeadingForWheel = swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
               int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel,
                       normalizedHeadingForWheel, swerveDriveFields.individualWheelHeadings[i]);

               int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ?  swerveDriveFields.individualWheelHeadings[i] +
                       reversedHeadingForWheel : swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;

               swerveDriveFields.individualWheelHeadings[i] = normalizeHeading(0, swerveDriveFields.individualWheelHeadings[i]);
               targetPosition = normalizeHeading(0, targetPosition);
               swerveDriveFields.individualTargetPositions[i] = targetPosition;

               swerveDriveFields.headingsNegativeOrNot[i] = targetPosition != Math.abs(targetPosition);

               swerveDriveFields.turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
               swerveDriveFields.individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] +
                       reversedHeadingForWheel : swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
           }
        };

        runnables[1] = () -> setPower(swerveDriveFields.headingsNegativeOrNot, swerveDriveFields.forwardVector);
        swerveState = SwerveState.DRIVE;
    }


    public int calculateReverseHeading(int totalHeading, int normalizedHeading, int wheelHeading) {
        return Math.abs(totalHeading) > 180 ? normalizeHeading(wheelHeading,
                normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180) : Constants.SwerveConstants.NO_REVERSAL;
    }

    public void applyRobotOrientedSwerve(int heading, double forwardPower, double sidePower, double turningVector, boolean turningLeft) {
        if(Math.abs(turningVector) >= 0.05 && swerveState == SwerveState.IDLE) {
            runnables[2] = () -> completeRotate(turningLeft, 0, turningVector, false);
            runnables[3] = () ->
                setPowerForCompleteRotate(turningLeft, swerveDriveFields.headingsReversed, turningVector,
                        swerveDriveFields.individualTargetPositions, false, swerveDriveFields.headingsNegativeOrNot);
            swerveState = SwerveState.COMPLETE_ROTATE;
        } else if(rotations == CompleteRotations.ROTATION_WAS_DONE && swerveState == SwerveState.IDLE) {
            runnables[4] = this::resetWheelHeading;
            swerveState = SwerveState.RESET_WHEEL;
        }

        //Implement regular driving operations.

        runnables[0] = () -> {
            swerveDriveFields.forwardVector = Math.hypot(sidePower, forwardPower) / Constants.SwerveConstants.vectorScalar;
            swerveDriveFields.normalizedHeading = normalizeHeading(0, heading);

            for (int i = 0; i < swerveDriveFields.turningMotors.length; i++) {
                int normalizedHeadingForWheel = normalizeHeading(swerveDriveFields.individualWheelHeadings[i], swerveDriveFields.normalizedHeading);
                int totalHeadingForWheel = swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
                int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel,
                        normalizedHeadingForWheel, swerveDriveFields.individualWheelHeadings[i]);

                int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] +
                        reversedHeadingForWheel : swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;

                swerveDriveFields.individualWheelHeadings[i] = normalizeHeading(0, swerveDriveFields.individualWheelHeadings[i]);
                targetPosition = normalizeHeading(0, targetPosition);
                swerveDriveFields.individualTargetPositions[i] = targetPosition;

                swerveDriveFields.headingsNegativeOrNot[i] = targetPosition != Math.abs(targetPosition);

                swerveDriveFields.turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
                swerveDriveFields.individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] +
                        reversedHeadingForWheel : swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
            }
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

        for(int i = 0; i < swerveDriveFields.turningMotors.length; i++) {
            int normalizedHeadingForWheel = normalizeHeading(swerveDriveFields.individualWheelHeadings[i], botHeading);
            int totalHeading = swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
            int reversedHeading = calculateReverseHeading(totalHeading, normalizedHeadingForWheel, swerveDriveFields.individualWheelHeadings[i]);

            int targetPosition = reversedHeading != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] + reversedHeading :
                   swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;

            swerveDriveFields.individualWheelHeadings[i] = normalizeHeading(0, swerveDriveFields.individualWheelHeadings[i]);
            targetPosition = normalizeHeading(0, targetPosition);
            swerveDriveFields.individualTargetPositions[i] = targetPosition;
            swerveDriveFields.headingsReversed[i] = reversedHeading != Constants.SwerveConstants.NO_REVERSAL;
            swerveDriveFields.headingsNegativeOrNot[i] = targetPosition != Math.abs(targetPosition);

            swerveDriveFields.turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
            swerveDriveFields.individualWheelHeadings[i] = reversedHeading != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] +
                    reversedHeading : swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
        }

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
            setPowerForCompleteRotate(swerveDriveFields.previousTurningLeft, swerveDriveFields.headingsReversed,
                    swerveDriveFields.previousTurningVector, new int[] {},
                    false, swerveDriveFields.headingsNegativeOrNot);
            return;
        }

        swerveDriveFields.previousTurningLeft = turningLeft;

        int headingFLBR = -45, headingFRBL = 45;

        for(int i = 0; i < swerveDriveFields.turningMotors.length; i++) {
            if(i == 0 || i == 2) {
                int normalizedHeadingForWheel = normalizeHeading(swerveDriveFields.individualWheelHeadings[i], headingFLBR);
                int totalHeadingForWheel = swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
                int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel, normalizedHeadingForWheel,
                        swerveDriveFields.individualWheelHeadings[i]);

                int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] + reversedHeadingForWheel :
                        swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;

                swerveDriveFields.individualWheelHeadings[i] = normalizeHeading(0, swerveDriveFields.individualWheelHeadings[i]);
                targetPosition = normalizeHeading(0, targetPosition);
                swerveDriveFields.individualTargetPositions[i] = targetPosition;
                swerveDriveFields.headingsReversed[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL;

                swerveDriveFields.turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
                swerveDriveFields.individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] +
                        reversedHeadingForWheel : swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
            } else {
                int normalizedHeadingForWheel = normalizeHeading(swerveDriveFields.individualWheelHeadings[i], headingFRBL);
                int totalHeadingForWheel = swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
                int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel, normalizedHeadingForWheel,
                        swerveDriveFields.individualWheelHeadings[i]);

                int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] + reversedHeadingForWheel :
                        swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;

                swerveDriveFields.individualWheelHeadings[i] = normalizeHeading(0, swerveDriveFields.individualWheelHeadings[i]);
                targetPosition = normalizeHeading(0, targetPosition);
                swerveDriveFields.individualTargetPositions[i] = targetPosition;
                swerveDriveFields.headingsReversed[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL;

                swerveDriveFields.turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
                swerveDriveFields.individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? swerveDriveFields.individualWheelHeadings[i] +
                        reversedHeadingForWheel : swerveDriveFields.individualWheelHeadings[i] + normalizedHeadingForWheel;
            }
        }
    }

    @Override
    public int normalizeHeading(int currentPosition, int targetPosition) {
        return (targetPosition - currentPosition + 540) % 360 - 180;
    }

    @Override
    public void setPowerForCompleteRotate(boolean turningLeft, boolean[] headingsReversed, double turningVector, int[] targetPositions, boolean goToPosition, boolean[] headingDirectionsNegative) {
        if(goToPosition) {
            setPowerToIndividualWheel(0, headingDirectionsNegative[0]);
            setPowerToIndividualWheel(1, headingDirectionsNegative[1]);
            setPowerToIndividualWheel(2, headingDirectionsNegative[2]);
            setPowerToIndividualWheel(3, headingDirectionsNegative[3]);

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
        setPowerToIndividualWheel(0, headingIsNegative[0]);
        setPowerToIndividualWheel(1, headingIsNegative[1]);
        setPowerToIndividualWheel(2, headingIsNegative[2]);
        setPowerToIndividualWheel(3, headingIsNegative[3]);

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
