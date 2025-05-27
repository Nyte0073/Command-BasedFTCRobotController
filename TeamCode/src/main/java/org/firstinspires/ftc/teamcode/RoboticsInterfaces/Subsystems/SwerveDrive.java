package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.RobotVector;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Swerve;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

/**Class containing code for developing a functional field-oriented/robot-oriented swerve drivetrain. This class will calculate all the math
 * used for making the swerve drive either drive or rotate, */
public class SwerveDrive extends Swerve {

    /**Robot's information system for updating the driver with information
     *  about the swerve drivetrain and its current functionality.*/
    Telemetry telemetry;

    /**Array containing references to the swerve drive's turning motors.*/
    private final Motor[] turningMotors,
    /**Array containing references to the swerve drive's driving motors.*/
            drivingMotors;

    /**Robot's orientation system for returning the angle that the robot is currently facing.*/
    private final IMU imu;

    /**Whether the robot's setPower() method has completed yet.*/
    private final AtomicBoolean asyncMethodHasFinished = new AtomicBoolean(false),
    /**Whether the robot's {@code completeRotate()} and {@code setPowerForCompleteRotate()} methods have
     * finished yet.*/
            asyncRotationMethodHasFinished = new AtomicBoolean(false),
    /**Whether the robot's {@code resetWheelHeading()} method has completed yet.*/
            asyncResettingRotationHasFinished = new AtomicBoolean(false),
    /**Whether the robot underwent an on-the-spot rotation or not without getting its wheel headings reset yet.*/
            rotationWasDone = new AtomicBoolean(false),
    /**Whether the robot has done a rotation on the spot and does not to have new calculations done to undergo rotation
     * again.*/
            alreadyRotated = new AtomicBoolean(false);

    /**Booleans states for keeping track of if all the wheels have rotated to their respective given headings.*/
    private final AtomicBoolean[] wheelsHaveRotated = new AtomicBoolean[] {
            new AtomicBoolean(false),
            new AtomicBoolean(false),
            new AtomicBoolean(false),
            new AtomicBoolean(false)
    };

    /**Array containing the individual headings of every single turning motor.*/
    private final int[] individualWheelHeadings = {0, 0, 0, 0};

    /**Array containing the individual target angles of every single turning motor.*/
    private final int[] individualTargetPositions = new int[4];

    /**Array containing the boolean states of if any wheel has their individual heading reversed due to their total heading
     * being greater than 180 degrees.*/
    private final boolean[] headingsReversed = new boolean[4];

    /**The previous turning vector from when the robot rotated last.*/
    private int previousTurningVector = 0;

    /**Boolean state of whether the robot was turning left the last time it rotated.*/
    private boolean previousTurningLeft = false;

    /**Boolean state of whether the robot will drive field-oriented or not.*/
    private final boolean fieldOriented;

    /**Array containing the previous target positions of every single turning motor from when the robot rotated last.*/
    private final int[] previousTargetPositions = {0, 0, 0, 0};

    /**Reference to the human driver's Xbox Controller for controlling the robot.*/
    private final GamepadEx gamepadEx;

    /**Constructs a new {@code SwerveDrive()} with initialized {@code Telemetry}, turning {@code Motor}'s, driving {@code Motor}'s,
     * {@code IMU}, {@code GamepadEx} and boolean state for whether the robot will drive field-oriented or robot-oriented.
     * This method also sets the {@code ZeroPowerBehavior} of the turning and driving motors to {@code BRAKE} when the motors have their power
     * set to 0, and also resets the encoders of the turning {@code Motor}'s.*/
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

    /**Drives the robot's swerve drive in a field-oriented way, that way when the human driver makes the robot drive forward, no matter the
     * robot's orientation, forward will be always be straight ahead.*/
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

        headingsReversed[0] = reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[1] = reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[2] = reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[3] = reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

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

    /**Calculates and returns the total heading of a specific turning motor, which is the sum of the turning motor's current heading
     * and calculated normalized heading based on its desired target heading.*/
    public int calculateTotalHeading(int currentHeading, int targetHeading, int wheelNumber) {
        int normalizedHeadingForWheel = normalizeHeading(currentHeading, targetHeading);
        individualTargetPositions[wheelNumber] = normalizedHeadingForWheel;
        return currentHeading + normalizedHeadingForWheel;
    }

    /**Calculates and returns a specific reversed heading for a specified turning motor. If the heading of the moto doesn't
     * need to be reversed, then this method will return the value of the {@code NO_REVERSAL} variable from the {@code Constants.Swerve}*/
    public int calculateReverseHeading(int totalHeading, int normalizedHeading, int wheelHeading) {
        return Math.abs(totalHeading) > 180 ? normalizeHeading(wheelHeading,
                normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180) : Constants.SwerveConstants.NO_REVERSAL;
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

        headingsReversed[0] = reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[1] = reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[2] = reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[3] = reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

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

        headingsReversed[0] = reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[1] = reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[2] = reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[3] = reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

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

        headingsReversed[0] = reversedHeadingFrontLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[1] = reversedHeadingFrontRight != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[2] = reversedHeadingBackLeft != Constants.SwerveConstants.NO_REVERSAL;
        headingsReversed[3] = reversedHeadingBackRight != Constants.SwerveConstants.NO_REVERSAL;

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

    /**Returns whether all of the turning motors have rotated to their desired target position.*/
    public boolean allWheelsHaveRotatedToPosition() {
        return wheelsHaveRotated[0].get() &&
                wheelsHaveRotated[1].get() &&
                wheelsHaveRotated[2].get() &&
                wheelsHaveRotated[3].get();
    }

    /**Sets the motor power to a specific individual turning wheel, specified by the {@code wheelNumber} parameter for array indexing.
     * This method also finds the index in the {@code wheelsHaveRotated} array that contains the boolean state for whether the wheel has rotated
     * to its desired position or not and sets it to true when the wheel has completed that task. This method runs asynchronously so that it won't
     * interrupt any other swerve drivetrain-related programs from running in the background.*/
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
    public RobotVector getRobotVector() {
        return new RobotVector((int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)),
                -gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX(), fieldOriented, gamepadEx.getRightX() != Math.abs(gamepadEx.getRightX()));
    }

    @Override
    public void periodic() {
        telemetry.update();
    }
}
