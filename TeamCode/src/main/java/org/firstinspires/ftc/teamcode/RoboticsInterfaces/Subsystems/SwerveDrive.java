package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Swerve;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

public class SwerveDrive extends Swerve {

    Telemetry telemetry;

    private final Motor[] turningMotors, drivingMotors;

    private final IMU imu;

    private final AtomicBoolean asyncMethodHasFinished = new AtomicBoolean(false),
            asyncRotationMethodHasFinished = new AtomicBoolean(false),
            asyncResettingRotationHasFinished = new AtomicBoolean(false),
            rotationWasDone = new AtomicBoolean(false),
            alreadyRotated = new AtomicBoolean(false);

    private final AtomicBoolean[] wheelsHaveRotated = new AtomicBoolean[] {
            new AtomicBoolean(false),
            new AtomicBoolean(false),
            new AtomicBoolean(false),
            new AtomicBoolean(false)
    };

    private final int[] individualWheelHeadings = {0, 0, 0, 0};

    private final int[] individualTargetPositions = new int[4];

    private final boolean[] headingsReversed = new boolean[4];

    private int previousTurningVector = 0;

    private boolean previousTurningLeft = false, fieldOriented;

    private int[] previousTargetPositions = {0, 0, 0, 0};

    private final GamepadEx gamepadEx;

    public SwerveDrive(Telemetry telemetry, Motor[] turningMotors, Motor[] drivingMotors, IMU imu, GamepadEx gamepadEx, boolean fieldOriented) {
        this.telemetry = telemetry;
        this.turningMotors = turningMotors;
        this.drivingMotors = drivingMotors;
        this.imu = imu;
        this.gamepadEx = gamepadEx;
        this.fieldOriented = fieldOriented;

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
            applyRobotOrientedSwerve(heading, forwardPower, sidePower, turningVector, turningLeft);
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
                resetWheelHeading();
            }
            return;
        }

        double forwardVector = Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;

        int normalizedHeading = normalizeHeading(headingInDegrees, heading);

        int totalHeadingFrontLeft = calculateTotalHeading(individualWheelHeadings[0], normalizedHeading, 0);
        int reversedHeadingFrontLeft = calculateReverseHeading(totalHeadingFrontLeft, normalizedHeading, individualWheelHeadings[0]);

        int totalHeadingFrontRight = calculateTotalHeading(individualWheelHeadings[1], normalizedHeading, 1);
        int reversedHeadingFrontRight =  calculateReverseHeading(totalHeadingFrontRight, normalizedHeading, individualWheelHeadings[1]);

        int totalHeadingBackLeft = calculateTotalHeading(individualWheelHeadings[2], normalizedHeading, 2);
        int reversedHeadingBackLeft = calculateReverseHeading(totalHeadingBackLeft, normalizedHeading, individualWheelHeadings[2]);

        int totalHeadingBackRight = calculateTotalHeading(individualWheelHeadings[3], normalizedHeading, 3);
        int reversedHeadingBackRight = calculateReverseHeading(totalHeadingBackRight, normalizedHeading, individualWheelHeadings[3]);

        headingsReversed[0] = reversedHeadingFrontLeft != 2000;
        headingsReversed[1] = reversedHeadingFrontRight != 2000;
        headingsReversed[2] = reversedHeadingBackLeft != 2000;
        headingsReversed[3] = reversedHeadingBackRight != 2000;

        individualTargetPositions[0] = headingsReversed[0] ? reversedHeadingFrontLeft : individualTargetPositions[0];
        individualTargetPositions[1] = headingsReversed[1] ? reversedHeadingFrontRight : individualTargetPositions[1];
        individualTargetPositions[2] = headingsReversed[2] ? reversedHeadingBackLeft : individualTargetPositions[2];
        individualTargetPositions[3] = headingsReversed[3] ? reversedHeadingBackRight : individualTargetPositions[3];

        turningMotors[0].setTargetPosition(individualTargetPositions[0]);
        turningMotors[1].setTargetPosition(individualTargetPositions[1]);
        turningMotors[2].setTargetPosition(individualTargetPositions[2]);
        turningMotors[3].setTargetPosition(individualTargetPositions[3]);

       setPower(headingsReversed, forwardVector);
    }

    public int calculateTotalHeading(int currentHeading, int targetHeading, int wheelNumber) {
        int normalizedHeadingForWheel = normalizeHeading(currentHeading, targetHeading);
        individualTargetPositions[wheelNumber] = normalizedHeadingForWheel;
        return currentHeading + normalizedHeadingForWheel;
    }

    public int calculateReverseHeading(int totalHeading, int normalizedHeading, int wheelHeading) {
        return Math.abs(totalHeading) > 180 ? normalizeHeading(wheelHeading,
                normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180) : 2000;
    }

    public void applyRobotOrientedSwerve(int heading, double forwardPower, double sidePower, double turningVector, boolean turningLeft) {
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
                resetWheelHeading();
            }
            return;
        }

        double forwardVector = Math.hypot(forwardPower, sidePower) / Constants.SwerveConstants.vectorScalar;

        int normalizedHeading = normalizeHeading(0, heading);

        int totalHeadingFrontLeft = calculateTotalHeading(individualWheelHeadings[0], normalizedHeading, 0);
        int reversedHeadingFrontLeft = calculateReverseHeading(totalHeadingFrontLeft, normalizedHeading, individualWheelHeadings[0]);

        int totalHeadingFrontRight = calculateTotalHeading(individualWheelHeadings[1], normalizedHeading, 1);
        int reversedHeadingFrontRight =  calculateReverseHeading(totalHeadingFrontRight, normalizedHeading, individualWheelHeadings[1]);

        int totalHeadingBackLeft = calculateTotalHeading(individualWheelHeadings[2], normalizedHeading, 2);
        int reversedHeadingBackLeft = calculateReverseHeading(totalHeadingBackLeft, normalizedHeading, individualWheelHeadings[2]);

        int totalHeadingBackRight = calculateTotalHeading(individualWheelHeadings[3], normalizedHeading, 3);
        int reversedHeadingBackRight = calculateReverseHeading(totalHeadingBackRight, normalizedHeading, individualWheelHeadings[3]);

        headingsReversed[0] = reversedHeadingFrontLeft != 2000;
        headingsReversed[1] = reversedHeadingFrontRight != 2000;
        headingsReversed[2] = reversedHeadingBackLeft != 2000;
        headingsReversed[3] = reversedHeadingBackRight != 2000;

        individualTargetPositions[0] = headingsReversed[0] ? reversedHeadingFrontLeft : individualTargetPositions[0];
        individualTargetPositions[1] = headingsReversed[1] ? reversedHeadingFrontRight : individualTargetPositions[1];
        individualTargetPositions[2] = headingsReversed[2] ? reversedHeadingBackLeft : individualTargetPositions[2];
        individualTargetPositions[3] = headingsReversed[3] ? reversedHeadingBackRight : individualTargetPositions[3];

        turningMotors[0].setTargetPosition(individualTargetPositions[0]);
        turningMotors[1].setTargetPosition(individualTargetPositions[1]);
        turningMotors[2].setTargetPosition(individualTargetPositions[2]);
        turningMotors[3].setTargetPosition(individualTargetPositions[3]);

        setPower(headingsReversed, forwardVector);
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
    public void resetWheelHeading() {
        int botHeading = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        int totalHeadingFrontLeft = calculateTotalHeading(individualWheelHeadings[0], botHeading, 0);
        int reversedHeadingFrontLeft = calculateReverseHeading(totalHeadingFrontLeft, botHeading, individualWheelHeadings[0]);

        int totalHeadingFrontRight = calculateTotalHeading(individualWheelHeadings[1], botHeading, 1);
        int reversedHeadingFrontRight = calculateReverseHeading(totalHeadingFrontRight, botHeading, individualWheelHeadings[1]);

        int totalHeadingBackLeft = calculateTotalHeading(individualWheelHeadings[2], botHeading, 2);
        int reversedHeadingBackLeft = calculateReverseHeading(totalHeadingBackLeft, botHeading, individualWheelHeadings[2]);

        int totalHeadingBackRight = calculateTotalHeading(individualWheelHeadings[3], botHeading, 3);
        int reversedHeadingBackRight = calculateReverseHeading(totalHeadingBackRight, botHeading, individualWheelHeadings[3]);

        headingsReversed[0] = reversedHeadingFrontLeft != 2000;
        headingsReversed[1] = reversedHeadingFrontRight != 2000;
        headingsReversed[2] = reversedHeadingBackLeft != 2000;
        headingsReversed[3] = reversedHeadingBackRight != 2000;

        CompletableFuture.runAsync(() -> {

            turningMotors[0].setTargetPosition(headingsReversed[0] ? reversedHeadingFrontLeft : individualTargetPositions[0]);
            turningMotors[1].setTargetPosition(headingsReversed[1] ? reversedHeadingFrontRight : individualTargetPositions[1]);
            turningMotors[2].setTargetPosition(headingsReversed[2] ? reversedHeadingBackLeft : individualTargetPositions[2]);
            turningMotors[3].setTargetPosition(headingsReversed[3] ? reversedHeadingBackRight : individualTargetPositions[3]);

            while(!allWheelsHaveRotatedToPosition()) {
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
            for(AtomicBoolean b : wheelsHaveRotated) {
                b.set(false);
            }

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
            setPowerForCompleteRotate(previousTurningLeft, headingsReversed, previousTurningVector, previousTargetPositions, false);
        }

        previousTurningLeft = turningLeft;

        int headingFLBR = -45, headingFRBL = 45;

        int totalHeadingFrontLeft = calculateTotalHeading(individualWheelHeadings[0], headingFLBR, 0);
        int reversedHeadingFrontLeft = calculateReverseHeading(totalHeadingFrontLeft, headingFLBR, individualWheelHeadings[0]);

        int totalHeadingFrontRight = calculateTotalHeading(individualWheelHeadings[1], headingFRBL, 1);
        int reversedHeadingFrontRight = calculateReverseHeading(totalHeadingFrontRight, headingFRBL, individualWheelHeadings[1]);

        int totalHeadingBackLeft = calculateTotalHeading(individualWheelHeadings[2], headingFLBR, 2);
        int reversedHeadingBackLeft = calculateReverseHeading(totalHeadingBackLeft, headingFLBR, individualWheelHeadings[2]);

        int totalHeadingBackRight = calculateTotalHeading(individualWheelHeadings[3], headingFRBL, 3);
        int reversedHeadingBackRight = calculateReverseHeading(totalHeadingBackRight, headingFRBL, individualWheelHeadings[3]);

        headingsReversed[0] = reversedHeadingFrontLeft != 2000;
        headingsReversed[1] = reversedHeadingFrontRight != 2000;
        headingsReversed[2] = reversedHeadingBackLeft != 2000;
        headingsReversed[3] = reversedHeadingBackRight != 2000;

        previousTargetPositions[0] = headingsReversed[0] ? reversedHeadingFrontLeft : individualTargetPositions[0];
        previousTargetPositions[1] = headingsReversed[1] ? reversedHeadingFrontRight : individualTargetPositions[1];
        previousTargetPositions[2] = headingsReversed[2] ? reversedHeadingBackLeft : individualTargetPositions[2];
        previousTargetPositions[3] = headingsReversed[3] ? reversedHeadingBackRight : individualTargetPositions[3];

        setPowerForCompleteRotate(turningLeft, headingsReversed, turnVector, new int[] {
                headingsReversed[0] ? reversedHeadingFrontLeft : individualTargetPositions[0],
                headingsReversed[1] ? reversedHeadingFrontRight : individualTargetPositions[1],
                headingsReversed[2] ? reversedHeadingBackLeft : individualTargetPositions[2],
                headingsReversed[3] ? reversedHeadingBackRight : individualTargetPositions[3]
        }, true);
    }

    @Override
    public int normalizeHeading(int currentPosition, int targetPosition) {
        return (targetPosition - currentPosition + 540) % 360 - 180;
    }

    @Override
    public void setPowerForCompleteRotate(boolean turningLeft, boolean[] headingsReversed, double turningVector, int[] targetPositions, boolean goToPosition) {
        CompletableFuture.runAsync(() -> {
            if(goToPosition) {
                turningMotors[0].setTargetPosition(targetPositions[0]);
                turningMotors[1].setTargetPosition(targetPositions[1]);
                turningMotors[2].setTargetPosition(targetPositions[2]);
                turningMotors[3].setTargetPosition(targetPositions[3]);

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
        }).thenRun(() -> {
            drivingMotors[0].set(headingsReversed[0] ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
            drivingMotors[1].set(headingsReversed[1] ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));
            drivingMotors[2].set(headingsReversed[2] ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
            drivingMotors[3].set(headingsReversed[3] ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));

            for(AtomicBoolean b : wheelsHaveRotated) {
                b.set(false);
            }

            asyncRotationMethodHasFinished.set(true);
            rotationWasDone.set(true);
            alreadyRotated.set(true);
        });
    }

    @Override
    public boolean getTurningLeft() {
        return previousTurningLeft;
    }

    @Override
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    @Override
    public void setPower(boolean[] headingsReversed,  double forwardVector) {
        CompletableFuture.runAsync(() -> {

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
        }).thenRun(() -> {
            for(int i = 0; i < turningMotors.length; i++) {
                drivingMotors[i].set(headingsReversed[i] ? -forwardVector : forwardVector);
            }

            for(AtomicBoolean b : wheelsHaveRotated) {
                b.set(false);
            }
            asyncMethodHasFinished.set(true);
        });
    }

    public boolean allWheelsHaveRotatedToPosition() {
        return wheelsHaveRotated[0].get() &&
                wheelsHaveRotated[1].get() &&
                wheelsHaveRotated[2].get() &&
                wheelsHaveRotated[3].get();
    }

    public void setPowerToIndividualWheel(int wheelNumber, boolean headingIsNegative) {
        CompletableFuture.runAsync(() -> {
            turningMotors[wheelNumber].set(headingIsNegative ? -1 : 1);
            while(!turningMotors[0].atTargetPosition()) {
                try {
                    Thread.sleep(10);
                } catch(Exception e) {
                    throw new RuntimeException(e);
                }
            }
            turningMotors[wheelNumber].set(0);
        }).thenRun(() -> wheelsHaveRotated[wheelNumber].set(true));
    }

    @Override
    public double[] getRobotDoubles() {
        return new double[] {-gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX()};
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
