package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

public class SwerveDrivetrain extends SubsystemBase {
    /**The robot's communication system.*/
    Telemetry telemetry;

    /**Motors to point the robot in the right direction of the intended movement vector.*/
    public final Motor[] turningMotors;

    /**Motors to drive the robot in the right direction of the intended movement vector.*/
    public final Motor[] drivingMotors;

    /**The robot's orientation system, returning the robot's current heading relative to a certain angle that
     * is considered 0 degrees.*/
    IMU imu;

    int previousHeadingFieldOriented = 0, previousHeadingNotFieldOriented = 0;

    /**Keeps track of whether the heading of the motors have been applied to the turning motors
     * and that the driving motors have had their {@code forwardVector} power set to them and are driving.*/
    public AtomicBoolean asyncMethodHasFinished = new AtomicBoolean(false),
    asyncRotationMethodHasFinished = new AtomicBoolean(false),
    asyncResettingRotationHasFinished = new AtomicBoolean(false),
    rotationWasDone = new AtomicBoolean(false),
    alreadyRotated = new AtomicBoolean(false);
    int[] wheelRotationPreviousHeadings = new int[2];
    boolean[] previousTurningLeftAndReversed = new boolean[3];


    /**Constructs a new {@code SwerveDrivetrain} with initialized {@code Telemetry}, turning {@code Motor}'s,
     * driving {@code Motor}'s, {@code GamepadEx} and {@code IMU}.
     * This constructor method also stops and resets the encoders of both the turning and driving motors, sets their
     * {@code ZeroPowerBehavior} mode to {@code BRAKE}, and sets the {@code RunMode} of the turning motors to
     * {@code PositionControl} so that those motors will run based on a set distance for them to rotate across.*/
    public SwerveDrivetrain(Telemetry telemetry, Motor[] turningMotors, Motor[] drivingMotors, IMU imu) {
        this.telemetry = telemetry;
        this.turningMotors = turningMotors;
        this.drivingMotors = drivingMotors;
        this.imu = imu;

        for(Motor m : turningMotors) {
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            m.stopAndResetEncoder();
            m.setRunMode(Motor.RunMode.PositionControl);
        }

        for(Motor m : drivingMotors) {
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void resetGyro() {
        imu.resetYaw();
    }

    /**Sets the power of the driving and turning motors and sets the target angle for the turning motors to turn to.
     * This method uses {@code CompletableFuture.runAsync()} to give the motors in the program time to to get to their set
     * positions and to accelerate in a another thread, completely separate from the main {@code setSwerveModuleState()} method.
     * This way, even if the {@code setSwerveModuleState()} method runs another time before the {@code CompletableFuture.runAsync()} method is done finishing,
     * it won't update the heading of the turning motors or call the {@code runAsync()} method until the boolean in {@code CompletableFuture.runAsync()} has
     * declared that the method has finished. This way, only one program is updating the motors at a time.*/
    public void setPower(boolean headingReversed, double forwardVector, boolean targetHeadingIsNegative) {
        CompletableFuture.runAsync(() -> {
            for(Motor m : turningMotors) {
               m.set(targetHeadingIsNegative ? -1 : 1);
            }

            Motor motor = turningMotors[0];
            while(!motor.atTargetPosition()) {
                try {
                    Thread.sleep(10);
                } catch(Exception e) {
                    throw new RuntimeException(e);
                }
            }

            for(Motor m : turningMotors) {
                m.set(0);
            }
        }).thenRun(() -> {
            for(Motor m : drivingMotors) {
                m.set(headingReversed ? -forwardVector : forwardVector);
            }
            asyncMethodHasFinished.set(true);
        });
    }

    /**Rotates the the robot perfectly in a circle by rotating every wheel to angle of 45 degrees in
     * opposite directions and then powering the motors in opposite powers to produce rotation in either a clockwise
     * or counterclockwise direction.*/
    public void completeRotate(boolean turningLeft, int imuHeadingInDegrees, double turnVector, boolean fieldOriented) {
        if(!asyncRotationMethodHasFinished.get()) {
            return;
        } else {
            asyncRotationMethodHasFinished.set(false);
        }

        if(alreadyRotated.get()) {
            setPowerForCompleteRotate(previousTurningLeftAndReversed[0], previousTurningLeftAndReversed[1], previousTurningLeftAndReversed[2], Math.abs(turnVector),
                    new int[] {}, false);
        }

        if(!fieldOriented) {
            imuHeadingInDegrees = 0;
        }

        previousTurningLeftAndReversed[0] = turningLeft;

        int headingFLBR = -45, headingFRBL = 45;

            int normalizedHeadingFLBR = normalizeHeading(imuHeadingInDegrees, headingFLBR);
            int normalizedHeadingWithPreviousFLBR = normalizeHeading(previousHeadingFieldOriented, normalizedHeadingFLBR);
            int totalHeadingFLBR = previousHeadingFieldOriented + normalizedHeadingWithPreviousFLBR;
            int reversedHeadingFLBR = Math.abs(totalHeadingFLBR) > 180 ? normalizeHeading(previousHeadingFieldOriented,
                    normalizedHeadingFLBR != Math.abs(normalizedHeadingFLBR) ? normalizedHeadingFLBR + 180 : normalizedHeadingFLBR - 180) : 0;
            boolean headingReversedFLBR = Math.abs(totalHeadingFLBR) > 180;

            previousTurningLeftAndReversed[1] = headingReversedFLBR;

            int normalizedHeadingFRBL = normalizeHeading(imuHeadingInDegrees, headingFRBL);
            int normalizedHeadingWithPreviousFRBL = normalizeHeading(previousHeadingFieldOriented, normalizedHeadingFRBL);
            int totalHeadingFRBL = previousHeadingFieldOriented + normalizedHeadingWithPreviousFLBR;
            int reversedHeadingFRBL = Math.abs(totalHeadingFRBL) > 180 ? normalizeHeading(previousHeadingFieldOriented,
                     normalizedHeadingFRBL != Math.abs(normalizedHeadingFRBL) ? normalizedHeadingFRBL + 180 : normalizedHeadingFRBL - 180) : 0;
            boolean headingReversedFRBL = Math.abs(totalHeadingFRBL) > 180;

            previousTurningLeftAndReversed[2] = headingReversedFRBL;

            int frontLeftBackRightPosition = ((headingReversedFLBR ? reversedHeadingFLBR : normalizedHeadingWithPreviousFLBR) / 360) * 1440;
            int frontRightBackLeftPosition = ((headingReversedFRBL ? reversedHeadingFRBL : normalizedHeadingWithPreviousFRBL) / 360) * 1440;

            turningMotors[0].setTargetPosition(frontLeftBackRightPosition);
            turningMotors[1].setTargetPosition(frontRightBackLeftPosition);
            turningMotors[2].setTargetPosition(frontRightBackLeftPosition);
            turningMotors[3].setTargetPosition(frontLeftBackRightPosition);

            wheelRotationPreviousHeadings[0] = frontLeftBackRightPosition;
            wheelRotationPreviousHeadings[1] = frontRightBackLeftPosition;

            setPowerForCompleteRotate(turningLeft, headingReversedFLBR, headingReversedFRBL, Math.abs(turnVector),
                    new int[] {frontLeftBackRightPosition, frontRightBackLeftPosition}, true);

    }

    public void setPowerForCompleteRotate(boolean turningLeft, boolean headingReversedFLBR, boolean headingReversedFRBL, double turningVector, int[] targetPositions, boolean goToPosition) {
        CompletableFuture.runAsync(() -> {
            if(goToPosition) {
                turningMotors[0].set(targetPositions[0] < 0 ? -1 : 1);
                turningMotors[1].set(targetPositions[1] < 0 ? -1 : 1);
                turningMotors[2].set(targetPositions[0] < 0 ? -1 : 1);
                turningMotors[3].set(targetPositions[1] < 0 ? -1 : 1);

                Motor motor = turningMotors[0];
                while (!motor.atTargetPosition()) {
                    try {
                        Thread.sleep(10);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            }

            for(Motor m : turningMotors) {
                m.set(0);
            }
        }).thenRun(() -> {
            drivingMotors[0].set(headingReversedFLBR ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
            drivingMotors[1].set(headingReversedFRBL ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));
            drivingMotors[2].set(headingReversedFRBL ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
            drivingMotors[3].set(headingReversedFLBR ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));

            asyncRotationMethodHasFinished.set(true);
            rotationWasDone.set(true);
            alreadyRotated.set(true);
        });
    }

    public void resetWheelHeading(int[] wheelRotationPreviousHeadings) {
        turningMotors[0].setTargetPosition(-wheelRotationPreviousHeadings[0]);
        turningMotors[1].setTargetPosition(-wheelRotationPreviousHeadings[1]);
        turningMotors[2].setTargetPosition(-wheelRotationPreviousHeadings[0]);
        turningMotors[3].setTargetPosition(-wheelRotationPreviousHeadings[1]);

        CompletableFuture.runAsync(() -> {
           turningMotors[0].set(-wheelRotationPreviousHeadings[0] < 0 ? -1 : 1);
           turningMotors[1].set(-wheelRotationPreviousHeadings[1] < 0 ? -1 : 1);
           turningMotors[2].set(-wheelRotationPreviousHeadings[0] < 0 ? -1 : 1);
           turningMotors[3].set(-wheelRotationPreviousHeadings[1] < 0 ? -1 : 1);

            Motor motor = turningMotors[0];
            while(!motor.atTargetPosition()) {
                try {
                    Thread.sleep(10);
                } catch(Exception e) {
                    throw new RuntimeException(e);
                }
            }

            for(Motor m : turningMotors) {
                m.set(0);
            }
        }).thenRun(() -> {
          rotationWasDone.set(false);
          asyncResettingRotationHasFinished.set(true);
          alreadyRotated.set(false);
        });
    }

    /**Calculates and sets the states of all the turning and driving motors in all 4 swerve modules.
     * This method considers whether you want to drive field oriented or robot oriented, and will calculate motor powers
     * for the driving motors and the set target heading for the turning motors relative to the angle returned by
     * the {@code IMU} system. */
    public void setSwerveModuleState(boolean fieldOriented, double forwardPower, double sidePower, int headingDegrees, double turningVector, boolean turningLeft) {

        int heading = (Math.abs(forwardPower) <= 0.01 && Math.abs(sidePower) <= 0.01) ? 0 :
                (int) Math.toDegrees(Math.atan2(forwardPower, sidePower)) - 90;

        if(fieldOriented) {

                if(!asyncMethodHasFinished.get()) {
                    return;
                } else if(Math.abs(turningVector) < 0.05) {
                    asyncMethodHasFinished.set(false);
                }

                if(Math.abs(turningVector) >= 0.05) {
                    if(asyncRotationMethodHasFinished.get()) {
                        asyncRotationMethodHasFinished.set(false);
                        completeRotate(turningLeft, headingDegrees, turningVector, true);
                    } else {
                        return;
                    }
                } else if(rotationWasDone.get()) {
                    if(asyncResettingRotationHasFinished.get()) {
                        asyncResettingRotationHasFinished.set(false);
                        resetWheelHeading(wheelRotationPreviousHeadings);
                    }
                    return;
                }

                telemetry.addData("Robot Heading in Degrees", headingDegrees);

                double forwardVector = Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;
                telemetry.addData("Forward Vector Motor Power", forwardVector);

                int normalizedHeading = normalizeHeading(headingDegrees, heading);
                int normalizedHeadingWithPreviousHeading = normalizeHeading(previousHeadingFieldOriented, normalizedHeading);
                int totalHeading = previousHeadingFieldOriented + normalizedHeadingWithPreviousHeading;

                telemetry.addData("Normalized Heading Without Previous Heading", normalizedHeading);
                telemetry.addData("Normalized Heading with Previous Heading", normalizedHeadingWithPreviousHeading);
                telemetry.addData("Total Heading", totalHeading);

                int reversedHeading = Math.abs(totalHeading) > 180 ? normalizeHeading(previousHeadingFieldOriented,
                        (normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180)) : 0;
                telemetry.addData("Reversed Heading", reversedHeading);

                boolean headingReversed = Math.abs(totalHeading) > 180;
                int targetPosition = ((headingReversed ? reversedHeading : normalizedHeadingWithPreviousHeading) / 360) * 1440;
                boolean targetHeadingIsNegative = targetPosition < 0;

                for(Motor m : turningMotors) {
                    m.setTargetPosition(targetPosition);
                }

                setPower(headingReversed, forwardVector, targetHeadingIsNegative);

                previousHeadingFieldOriented = headingReversed ? previousHeadingFieldOriented + reversedHeading : normalizedHeading;

            } else {
                if(asyncMethodHasFinished.get()) {
                    return;
                } else if(Math.abs(turningVector) < 0.05) {
                    asyncMethodHasFinished.set(false);
                }

                if(Math.abs(turningVector) >= 0.05) {
                    if(asyncRotationMethodHasFinished.get()) {
                       asyncRotationMethodHasFinished.set(false);
                       completeRotate(turningLeft, 0, turningVector, false);
                    } else {
                        return;
                    }
                } else if(rotationWasDone.get()) {
                    if (asyncResettingRotationHasFinished.get()) {
                        resetWheelHeading(wheelRotationPreviousHeadings);
                    }
                    return;
                }

                double forwardVector = Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;
                telemetry.addData("Forward Vector Motor Power", forwardVector);

                int normalizedHeading = normalizeHeading(0, heading);
                int normalizedHeadingWithPreviousHeading = normalizeHeading(previousHeadingNotFieldOriented, normalizedHeading);
                int totalHeading = previousHeadingNotFieldOriented + normalizedHeadingWithPreviousHeading;

                telemetry.addData("Normalized Heading Without Previous Heading", normalizedHeading);
                telemetry.addData("Normalized Heading with Previous Heading", normalizedHeadingWithPreviousHeading);
                telemetry.addData("Total Heading", totalHeading);

                int reversedHeading = Math.abs(totalHeading) > 180 ? normalizeHeading(previousHeadingNotFieldOriented,
                        (normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180)) : 0;

                boolean headingReversed = Math.abs(totalHeading) > 180;
                int targetPosition = ((headingReversed ? reversedHeading : normalizedHeadingWithPreviousHeading) / 360) * 1440;
                boolean targetHeadingIsNegative = targetPosition < 0;

                for(Motor m : turningMotors) {
                    m.setTargetPosition(targetPosition);
                }

                setPower(headingReversed, forwardVector, targetHeadingIsNegative);

                previousHeadingNotFieldOriented = headingReversed ? previousHeadingNotFieldOriented + reversedHeading : normalizedHeading;

            }
    }

    /**Returns the normalized heading (shortest angular distance) of the target angle for the turning motors,
     * reducing the amount of distance the turning motors need to turn to be at a certain angle, and also limit how
     * much the turning can turn in sitting, preventing issues like tangled wires from the motors rotating 360 degrees.*/
    public int normalizeHeading(int currentHeading, int targetHeading) {
        return (targetHeading - currentHeading + 540) % 360 - 180;
    }


    @Override
    public void periodic() {
        telemetry.update();
    }
}
