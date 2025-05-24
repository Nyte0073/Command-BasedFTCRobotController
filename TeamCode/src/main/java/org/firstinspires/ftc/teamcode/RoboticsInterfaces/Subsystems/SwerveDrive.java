package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Swerve;

import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

public class SwerveDrive extends Swerve {

    Telemetry telemetry;

    private final Motor[] turningMotors, drivingMotors;

    private final IMU imu;

    int previousHeadingFieldOriented = 0, previousHeadingNotFieldOriented = 0;

    private final AtomicBoolean asyncMethodHasFinished = new AtomicBoolean(false),
            asyncRotationMethodHasFinished = new AtomicBoolean(false),
            asyncResettingRotationHasFinished = new AtomicBoolean(false),
            rotationWasDone = new AtomicBoolean(false),
            alreadyRotated = new AtomicBoolean(false);

    private final int[] wheelRotationPreviousHeadings = new int[2];

    private final boolean[] previousTurningLeftAndReversed = new boolean[3];

    private final GamepadEx gamepadEx;

    public SwerveDrive(Telemetry telemetry, Motor[] turningMotors, Motor[] drivingMotors, IMU imu, GamepadEx gamepadEx) {
        this.telemetry = telemetry;
        this.turningMotors = turningMotors;
        this.drivingMotors = drivingMotors;
        this.imu = imu;
        this.gamepadEx = gamepadEx;

        for(Motor m : turningMotors) {
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            m.stopAndResetEncoder();
            m.setRunMode(Motor.RunMode.PositionControl);
        }

        for(Motor m : drivingMotors) {
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void setSwerveModuleState(boolean fieldOriented, double forwardPower, double sidePower, int headingInDegrees, double turningVector, boolean turningLeft) {
        int heading = (Math.abs(forwardPower) <= 0.01 && Math.abs(sidePower) <= 0.01) ? 0 :
                (int) Math.toDegrees(Math.atan2(forwardPower, sidePower)) - 90;

        if(fieldOriented) {
            applyFieldOrientedSwerve(heading, forwardPower, sidePower, headingInDegrees, turningVector, turningLeft);
        } else {
            applyRobotOrientedSwerve(heading, forwardPower, sidePower, headingInDegrees, turningVector, turningLeft);
        }
    }

    public void applyFieldOrientedSwerve(int heading, double forwardPower, double sidePower, int headingInDegrees, double turningVector, boolean turningLeft) {
        if(!asyncMethodHasFinished.get()) {
            return;
        } else if(Math.abs(turningVector) < 0.05) {
            asyncMethodHasFinished.set(false);
        }

        if(Math.abs(turningVector) >= 0.05) {
            if(asyncRotationMethodHasFinished.get()) {
                asyncRotationMethodHasFinished.set(false);
                completeRotate(turningLeft, headingInDegrees, turningVector, true);
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

        double forwardVector = Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;

        int normalizedHeading = normalizeHeading(headingInDegrees, heading);
        int normalizedHeadingWithPreviousHeading = normalizeHeading(previousHeadingFieldOriented, normalizedHeading);
        int totalHeading = previousHeadingFieldOriented + normalizedHeadingWithPreviousHeading;

        int reversedHeading = Math.abs(totalHeading) > 180 ? normalizeHeading(previousHeadingFieldOriented,
                (normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180)) : 0;

        boolean headingReversed = Math.abs(totalHeading) > 180;
        int targetPosition = ((headingReversed ? reversedHeading : normalizedHeadingWithPreviousHeading) / 360) * 1440;
        boolean targetHeadingIsNegative = targetPosition < 0;

        updateTelemetry(telemetry, new HashMap<String, Object>() {{
            put("Robot Heading In Degrees", headingInDegrees);
            put("Forward Vector Motor Power", forwardVector);
            put("Normalized Heading Without Previous Heading", normalizedHeading);
            put("Normalized Heading With Previous Heading", normalizedHeadingWithPreviousHeading);
            put("Total Heading", totalHeading);
            put("Reversed Heading", reversedHeading);
        }});

        for(Motor m : turningMotors) {
            m.setTargetPosition(targetPosition);
        }

        setPower(headingReversed, forwardVector, targetHeadingIsNegative);

        previousHeadingFieldOriented = headingReversed ? previousHeadingFieldOriented + reversedHeading : normalizedHeading;

    }

    public void applyRobotOrientedSwerve(int heading, double forwardPower, double sidePower, int headingInDegrees, double turningVector, boolean turningLeft) {
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

        int normalizedHeading = normalizeHeading(0, heading);
        int normalizedHeadingWithPreviousHeading = normalizeHeading(previousHeadingNotFieldOriented, normalizedHeading);
        int totalHeading = previousHeadingNotFieldOriented + normalizedHeadingWithPreviousHeading;

        int reversedHeading = Math.abs(totalHeading) > 180 ? normalizeHeading(previousHeadingNotFieldOriented,
                (normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180)) : 0;

        boolean headingReversed = Math.abs(totalHeading) > 180;
        int targetPosition = ((headingReversed ? reversedHeading : normalizedHeadingWithPreviousHeading) / 360) * 1440;
        boolean targetHeadingIsNegative = targetPosition < 0;

        updateTelemetry(telemetry, new HashMap<String, Object>() {{
            put("Robot Heading In Degrees", headingInDegrees);
            put("Forward Vector Motor Power", forwardVector);
            put("Normalized Heading Without Previous Heading", normalizedHeading);
            put("Normalized Heading With Previous Heading", normalizedHeadingWithPreviousHeading);
            put("Total Heading", totalHeading);
            put("Reversed Heading", reversedHeading);
        }});

        for(Motor m : turningMotors) {
            m.setTargetPosition(targetPosition);
        }

        setPower(headingReversed, forwardVector, targetHeadingIsNegative);

        previousHeadingNotFieldOriented = headingReversed ? previousHeadingNotFieldOriented + reversedHeading : normalizedHeading;
    }

    @Override
    public void stopMotors() {
        for(Motor m : turningMotors) {
            m.set(0);
        }

        for(Motor m : drivingMotors) {
            m.set(0);
        }
    }

    @Override
    public void resetWheelHeading(int[] previousWheelHeadings) {

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

    @Override
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

    @Override
    public int normalizeHeading(int currentPosition, int targetPosition) {
        return (targetPosition - currentPosition + 540) % 360 - 180;
    }

    @Override
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

    @Override
    public boolean[] getRobotBooleans() {
        return previousTurningLeftAndReversed;
    }

    @Override
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

    @Override
    public double[] getRobotDoubles() {
        return new double[] {-gamepadEx.getLeftY(), gamepadEx.getLeftX()};
    }

    @Override
    public int getHeadingInDegrees() {
        return (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    @Override
    public void periodic() {
        telemetry.update();
    }
}
