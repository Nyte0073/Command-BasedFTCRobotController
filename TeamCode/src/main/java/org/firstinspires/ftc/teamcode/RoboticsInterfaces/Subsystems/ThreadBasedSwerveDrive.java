package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.BackEndServer;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Swerve;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.SwerveVector;

import java.util.Arrays;
import java.util.List;

public class ThreadBasedSwerveDrive extends Swerve {

    /**Reference to the robot's IMU system, returning the heading that the robot is currently facing.*/
    private final IMU imu;

    /**The factor by which the driving motors' motor powers will be tuned to for driving.*/
    private volatile double forwardVector = 0;

    /**The normalized, calculated heading for the robot to move in the direction of, relative ot the current heading of the ROBOT,
     * NOT the wheels.*/
    private volatile int normalizedHeading = 0;

    /**Boolean arrays for keeping track of if the headings of the turning wheels are reversed, if those headings are negative or not,
     * and if the wheels have rotated.*/
    private final boolean[] headingsReversed = new boolean[4],
            headingsNegativeOrNot = new boolean[4], wheelsHaveRotated = new boolean[4];

    /**Integer arrays for keeping track of the individual headings of each turning wheel and their individual target positions.*/
    private final int[] individualWheelHeadings = new int[4], individualTargetPositions = new int[4],
            previousTargetPositions = new int[4];

    /**Boolean state for if the robot will be rotating on the spot to the left or to the right.*/
    private volatile boolean previousTurningLeft = false;

    /**Boolean state for if the robot will be driving field-oriented or robot oriented.*/
    private final boolean fieldOriented;

    /**Boolean state for if the robot has already underwent a complete rotation without having its turing wheels reset yet.*/
    private volatile boolean alreadyRotated = false;

    /**Reference to the human driver's Xbox controller.*/
    private final GamepadEx gamepadEx;

    /**Boolean state for if the robot is requesting to the completely stop its turning and driving motors.*/
    public static boolean stopMotorsIsRunning = false;

    /**{@code Motor} arrays for keeping references to the robot's turning and driving motors.*/
    private final Motor[] turningMotors, drivingMotors;

    /**The robot's information sending system. Use {@code telemetry} to send information regarding the states of all the hardware on
     * the robot pertaining to this subsystem to the Driver Station screen for you to view while you're driving the robot.*/
    private final Telemetry telemetry;

    /**{@code Runnable} array to keep references to the tasks that the different threads within this subsystem will be carrying out.*/
    Runnable[] runnables = new Runnable[] {
            () -> {},
            () -> {},
            () -> {},
            () -> {},
            () -> {}
    };

    /**Object to use with the threads' {@code synchronized() {}} blocks to act as lock that only thread can hold at a time. This means
     * that having this object will ensure that only the thread with this object can execute synchronized code at will.*/
    private final Object driveLock = new Object();

    /**Reference to the current driving state of the robot.*/
    private volatile SwerveState swerveState = SwerveState.IDLE;

    /**Reference to current state of the robot's rotational abilities.*/
    private volatile CompleteRotations rotations = CompleteRotations.ROTATION_WAS_NOT_DONE;

    /**{@code Thread} array for keeping references to all the different threads this subsystem will be using to execute different driving
     * actions asynchronously and separately.*/
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
                                if(swerveState == SwerveState.DRIVE) {
                                    driveLock.notifyAll();
                                    runnables[0].run();
                                    runnables[1].run();
                                    try {
                                        Thread.sleep(50);
                                    } catch(Exception e) {
                                        Thread.currentThread().interrupt();
                                    }
                                    swerveState = SwerveState.IDLE;
                                    driveLock.notifyAll();
                                }
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
                                if(swerveState == SwerveState.COMPLETE_ROTATE) {
                                    driveLock.notifyAll();
                                    runnables[2].run();
                                    runnables[3].run();
                                    try {
                                        Thread.sleep(50);
                                    } catch(Exception e) {
                                        Thread.currentThread().interrupt();
                                    }
                                    swerveState = SwerveState.IDLE;
                                    rotations = CompleteRotations.ROTATION_WAS_DONE;
                                    driveLock.notifyAll();
                                }
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
                                if(swerveState == SwerveState.RESET_WHEEL) {
                                    driveLock.notifyAll();
                                    runnables[4].run();
                                    try {
                                        Thread.sleep(50);
                                    } catch(Exception e) {
                                        Thread.currentThread().interrupt();
                                    }
                                    swerveState = SwerveState.IDLE;
                                    rotations = CompleteRotations.ROTATION_WAS_NOT_DONE;
                                    System.out.println("Wheels reset.");
                                    driveLock.notifyAll();
                                }
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

    /**Constant class for keeping references to all the robot's possible driving states.*/
    enum SwerveState {
        IDLE,
        DRIVE,
        COMPLETE_ROTATE,
        RESET_WHEEL
    }

    /**Constant class for keeping references to all the robot's possible rotation states.*/
    enum CompleteRotations {
        ROTATION_WAS_DONE,
        ROTATION_WAS_NOT_DONE
    }

    /**Constructs an empty {@code ThreadBasedSwerveDrive()} that is typically used for referencing void methods, NOT for initializing
     *the subsystem in a Teleop class.*/
    public ThreadBasedSwerveDrive() {
        this(null, new Motor[]{}, new Motor[]{}, null, null, false);
    }

    public ThreadBasedSwerveDrive(Telemetry telemetry, Motor[] turningMotors, Motor[] drivingMotors, IMU imu, GamepadEx gamepadEx, boolean fieldOriented) {
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

        for(Thread thread : threads) {
            thread.start();
        }

        BackEndServer backEndServer = new BackEndServer(4000);
        backEndServer.start();
    }

    public List <boolean[]> getRobotBooleanArrays() {
        return List.of(this.headingsReversed.clone(), this.headingsNegativeOrNot.clone(),
                this.wheelsHaveRotated.clone());
    }

    public List <Double> getForwardVectorAndNormalizedHeading() {
        double forwardVector = this.forwardVector;
        double normalizedHeading = this.normalizedHeading;
        return List.of(forwardVector, normalizedHeading);
    }

    public List <Boolean> getRobotBooleans() {
        boolean previousTurningLeft = this.previousTurningLeft;
        boolean fieldOriented = this.fieldOriented;
        boolean alreadyRotated = this.alreadyRotated;
        return List.of(previousTurningLeft, fieldOriented, alreadyRotated);
    }

    public int[] getTargetPositions() {
        return individualTargetPositions.clone();
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
            runnables[2] = () -> completeRotate(turningLeft, headingInDegrees, Math.abs(turningVector), true);
            runnables[3] = () ->
                    setPowerForCompleteRotate(turningLeft, headingsReversed, Math.abs(turningVector),
                            individualTargetPositions, true, headingsNegativeOrNot);
            swerveState = SwerveState.COMPLETE_ROTATE;
        } else if(rotations == CompleteRotations.ROTATION_WAS_DONE && swerveState == SwerveState.IDLE) {
           runnables[4] = this::resetWheelHeading;
           swerveState = SwerveState.RESET_WHEEL;
        }
        //Implement regular driving operations.

        runnables[0] = () -> {
           forwardVector = Math.hypot(sidePower, forwardPower) / Constants.SwerveConstants.vectorScalar;
           normalizedHeading = normalizeHeading(headingInDegrees, heading);

           for(int i = 0; i < turningMotors.length; i++) {
               int normalizedHeadingForWheel = normalizeHeading(individualWheelHeadings[i], normalizedHeading);
               int totalHeadingForWheel = individualWheelHeadings[i] + normalizedHeadingForWheel;
               int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel,
                       individualWheelHeadings[i]);

               int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ?  individualWheelHeadings[i] +
                       reversedHeadingForWheel : individualWheelHeadings[i] + normalizedHeadingForWheel;

               individualWheelHeadings[i] = normalizeHeading(0, individualWheelHeadings[i]);
               targetPosition = normalizeHeading(0, targetPosition);
               individualTargetPositions[i] = targetPosition;

               headingsNegativeOrNot[i] = targetPosition != Math.abs(targetPosition);

               turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
               individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] +
                       reversedHeadingForWheel : individualWheelHeadings[i] + normalizedHeadingForWheel;
           }
        };

        runnables[1] = () -> setPower(headingsNegativeOrNot, forwardVector);
        swerveState = SwerveState.DRIVE;
    }


    public int calculateReverseHeading(int totalHeading, int wheelHeading) {
        return Math.abs(totalHeading) > 180 ? normalizeHeading(wheelHeading,
                totalHeading < 0 ? totalHeading + 180 : totalHeading - 180) : Constants.SwerveConstants.NO_REVERSAL;
    }

    public void applyRobotOrientedSwerve(int heading, double forwardPower, double sidePower, double turningVector, boolean turningLeft) {
        if(Math.abs(turningVector) >= 0.05 && swerveState == SwerveState.IDLE) {
            runnables[2] = () -> completeRotate(turningLeft, 0, Math.abs(turningVector), false);
            runnables[3] = () ->
                setPowerForCompleteRotate(turningLeft, headingsReversed, Math.abs(turningVector),
                        individualTargetPositions, false, headingsNegativeOrNot);
            swerveState = SwerveState.COMPLETE_ROTATE;
        } else if(rotations == CompleteRotations.ROTATION_WAS_DONE && swerveState == SwerveState.IDLE) {
            runnables[4] = this::resetWheelHeading;
            swerveState = SwerveState.RESET_WHEEL;
        }

        //Implement regular driving operations.

        runnables[0] = () -> {
            forwardVector = Math.hypot(sidePower, forwardPower) / Constants.SwerveConstants.vectorScalar;
            normalizedHeading = normalizeHeading(0, heading);

            for (int i = 0; i < turningMotors.length; i++) {
                int normalizedHeadingForWheel = normalizeHeading(individualWheelHeadings[i], normalizedHeading);
                int totalHeadingForWheel = individualWheelHeadings[i] + normalizedHeadingForWheel;
                int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel,
                        individualWheelHeadings[i]);

                int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] +
                        reversedHeadingForWheel : individualWheelHeadings[i] + normalizedHeadingForWheel;

                individualWheelHeadings[i] = normalizeHeading(0, individualWheelHeadings[i]);
                targetPosition = normalizeHeading(0, targetPosition);
                individualTargetPositions[i] = targetPosition;

                headingsNegativeOrNot[i] = targetPosition != Math.abs(targetPosition);

                turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
                individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] +
                        reversedHeadingForWheel : individualWheelHeadings[i] + normalizedHeadingForWheel;
            }
        };

        runnables[1] = () -> setPower(headingsNegativeOrNot, forwardVector);
        swerveState = SwerveState.DRIVE;
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
        stopMotors();
        int botHeading = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        for(int i = 0; i < turningMotors.length; i++) {
            int normalizedHeadingForWheel = normalizeHeading(individualWheelHeadings[i], botHeading);
            int totalHeading = individualWheelHeadings[i] + normalizedHeadingForWheel;
            int reversedHeading = calculateReverseHeading(totalHeading, individualWheelHeadings[i]);

            int targetPosition = reversedHeading != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] + reversedHeading :
                   individualWheelHeadings[i] + normalizedHeadingForWheel;

            individualWheelHeadings[i] = normalizeHeading(0, individualWheelHeadings[i]);
            targetPosition = normalizeHeading(0, targetPosition);
            individualTargetPositions[i] = targetPosition;
            headingsReversed[i] = reversedHeading != Constants.SwerveConstants.NO_REVERSAL;
            headingsNegativeOrNot[i] = targetPosition != Math.abs(targetPosition);

            turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
            individualWheelHeadings[i] = reversedHeading != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] +
                    reversedHeading : individualWheelHeadings[i] + normalizedHeadingForWheel;
        }

        for(int i = 0; i < turningMotors.length; i++) {
            setPowerToIndividualWheel(i, headingsNegativeOrNot[i]);
        }

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

        Arrays.fill(wheelsHaveRotated, false);
        alreadyRotated = false;
    }

    @Override
    public void completeRotate(boolean turningLeft, int imuHeadingInDegrees, double turnVector, boolean fieldOriented) {
        if(alreadyRotated) {
            setPowerForCompleteRotate(previousTurningLeft, headingsReversed,
                    Math.abs(turnVector), new int[] {},
                    false, headingsNegativeOrNot);
            return;
        }

        previousTurningLeft = turningLeft;

        int headingFLBR = -45, headingFRBL = 45;

        for(int i = 0; i < turningMotors.length; i++) {
            if(i == 0 || i == 3) {
                int normalizedHeadingForWheel = normalizeHeading(individualWheelHeadings[i], headingFLBR);
                int totalHeadingForWheel = individualWheelHeadings[i] + normalizedHeadingForWheel;
                int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel, individualWheelHeadings[i]);

                int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] + reversedHeadingForWheel :
                        individualWheelHeadings[i] + normalizedHeadingForWheel;

                individualWheelHeadings[i] = normalizeHeading(0, individualWheelHeadings[i]);
                targetPosition = normalizeHeading(0, targetPosition);
                individualTargetPositions[i] = targetPosition;
                headingsReversed[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL;

                turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
                individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] +
                        reversedHeadingForWheel : individualWheelHeadings[i] + normalizedHeadingForWheel;
            } else {
                int normalizedHeadingForWheel = normalizeHeading(individualWheelHeadings[i], headingFRBL);
                int totalHeadingForWheel = individualWheelHeadings[i] + normalizedHeadingForWheel;
                int reversedHeadingForWheel = calculateReverseHeading(totalHeadingForWheel, individualWheelHeadings[i]);

                int targetPosition = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] + reversedHeadingForWheel :
                        individualWheelHeadings[i] + normalizedHeadingForWheel;

                individualWheelHeadings[i] = normalizeHeading(0, individualWheelHeadings[i]);
                targetPosition = normalizeHeading(0, targetPosition);
                individualTargetPositions[i] = targetPosition;
                headingsReversed[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL;

                turningMotors[i].setTargetPosition((targetPosition / 360) * 1440);
                individualWheelHeadings[i] = reversedHeadingForWheel != Constants.SwerveConstants.NO_REVERSAL ? individualWheelHeadings[i] +
                        reversedHeadingForWheel : individualWheelHeadings[i] + normalizedHeadingForWheel;
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

        drivingMotors[0].set(headingsReversed[0] ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
        drivingMotors[1].set(headingsReversed[1] ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));
        drivingMotors[2].set(headingsReversed[2] ? (turningLeft ? turningVector : -turningVector) : (turningLeft ? -turningVector : turningVector));
        drivingMotors[3].set(headingsReversed[3] ? (turningLeft ? -turningVector : turningVector) : (turningLeft ? turningVector : -turningVector));

        Arrays.fill(wheelsHaveRotated, false);
        alreadyRotated = true;
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

        for(int i = 0; i < turningMotors.length; i++) {
            drivingMotors[i].set(headingsReversed[i] ? -forwardVector : forwardVector);
        }

        Arrays.fill(wheelsHaveRotated, false);
    }

    public boolean allWheelsHaveRotatedToPosition() {
        return wheelsHaveRotated[0]
                && wheelsHaveRotated[1] &&
                wheelsHaveRotated[2] &&
                wheelsHaveRotated[3];
    }

    public void setPowerToIndividualWheel(int wheelNumber, boolean headingIsNegative) {
        turningMotors[wheelNumber].set(headingIsNegative ? -1 : 1);
        while(!turningMotors[wheelNumber].atTargetPosition()) {
            try {
                Thread.sleep(10);
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
        }
        turningMotors[wheelNumber].set(0);
        wheelsHaveRotated[wheelNumber] = true;
    }

    @Override
    public SwerveVector getRobotVector() {
        return new SwerveVector((int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)),
                -gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX(), fieldOriented, gamepadEx.getRightX() != Math.abs(gamepadEx.getRightX()));
    }

    @Override
    public void periodic() {
        telemetry.update();
    }
}
