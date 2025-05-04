package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    /**Reference to the driver's gamepad controller.*/
    GamepadEx gamepadEx;

    /**The robot's orientation system, returning the robot's current heading relative to a certain angle that
     * is considered 0 degrees.*/
    IMU imu;

    double previousHeadingFieldOriented = 0, previousHeadingNotFieldOriented = 0;

    /**Keeps track of whether the heading of the motors have been applied to the turning motors
     * and that the driving motors have had their {@code forwardVector} power set to them and are driving.*/
    public AtomicBoolean asyncMethodHasFinished = new AtomicBoolean(false);

    /**Constructs a new {@code SwerveDrivetrain} with initialized {@code Telemetry}, turning {@code Motor}'s,
     * driving {@code Motor}'s, {@code GamepadEx} and {@code IMU}.
     * This constructor method also stops and resets the encoders of both the turning and driving motors, sets their
     * {@code ZeroPowerBehavior} mode to {@code BRAKE}, and sets the {@code RunMode} of the turning motors to
     * {@code PositionControl} so that those motors will run based on a set distance for them to rotate across.*/
    public SwerveDrivetrain(Telemetry telemetry, Motor[] turningMotors, Motor[] drivingMotors, GamepadEx gamepadEx, IMU imu) {
        this.telemetry = telemetry;
        this.gamepadEx = gamepadEx;
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

    /**Sets the power of the driving and turning motors and sets the target angle for the turning motors to turn to.
     * This method uses {@code CompletableFuture.runAsync()} to give the motors in the program time to to get to their set
     * positions and to accelerate in a another thread, completely separate from the main {@code setSwerveModuleState()} method.
     * This way, even if the {@code setSwerveModuleState()} method runs another time before the {@code CompletableFuture.runAsync()} method is done finishing,
     * it won't update the heading of the turning motors or call the {@code runAsync()} method until the boolean in {@code CompletableFuture.runAsync()} has
     * declared that the method has finished. This way, only one program is updating the motors at a time.*/
    public void setPower(boolean headingReversed, double forwardVector) {
        CompletableFuture.runAsync(() -> {
            for(Motor m : turningMotors) {
               m.set(1);
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

    /**Calculates and sets the states of all the turning and driving motors in all 4 swerve modules.
     * This method considers whether you want to drive field oriented or robot oriented, and will calculate motor powers
     * for the driving motors and the set target heading for the turning motors relative to the angle returned by
     * the {@code IMU} system. */
    public void setSwerveModuleState(boolean fieldOriented) {
            if(fieldOriented) {

                if(!asyncMethodHasFinished.get()) {
                    return;
                } else {
                    asyncMethodHasFinished.set(false);
                }

                double headingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double forward = -gamepadEx.getLeftY();
                double side = gamepadEx.getLeftX();

                double forwardVector = Math.hypot(forward, side) / Constants.SwerveConstants.vectorScalar;
                double heading = Math.toDegrees(Math.atan2(forward, side)) - 90;

                double normalizedHeading = normalizeHeading(headingDegrees, heading);
                double normalizedHeadingWithPreviousHeading = normalizeHeading(previousHeadingFieldOriented, normalizedHeading);
                double totalHeading = previousHeadingFieldOriented + normalizedHeadingWithPreviousHeading;

                double reversedHeading = Math.abs(totalHeading) > 180 ? normalizeHeading(previousHeadingFieldOriented,
                        (normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180)) : 0;

                for(Motor m : turningMotors) {
                    m.setTargetPosition((int) Math.round(((reversedHeading != 0 ? reversedHeading : normalizedHeadingWithPreviousHeading) / 360) * 1440));
                }

                setPower(reversedHeading != 0, forwardVector);

                previousHeadingFieldOriented = reversedHeading != 0 ? previousHeadingFieldOriented + reversedHeading : normalizedHeading;

            } else {
                if(!asyncMethodHasFinished.get()) {
                    return;
                } else {
                    asyncMethodHasFinished.set(false);
                }

                double forward = -gamepadEx.getLeftY();
                double side = gamepadEx.getLeftX();
                double heading = Math.toDegrees(Math.atan2(forward, side)) - 90;
                double forwardVector = Math.hypot(forward, side) / Constants.SwerveConstants.vectorScalar;

                double normalizedHeading = normalizeHeading(0, heading);
                double normalizedHeadingWithPreviousHeading = normalizeHeading(previousHeadingNotFieldOriented, normalizedHeading);
                double totalHeading = previousHeadingNotFieldOriented + normalizedHeadingWithPreviousHeading;

                double reversedHeading = Math.abs(totalHeading) > 180 ? normalizeHeading(previousHeadingNotFieldOriented,
                        (normalizedHeading != Math.abs(normalizedHeading) ? normalizedHeading + 180 : normalizedHeading - 180)) : 0;

                for(Motor m : turningMotors) {
                    m.setTargetPosition((int) Math.round(((reversedHeading != 0 ? reversedHeading : normalizedHeadingWithPreviousHeading) / 360) * 1440));
                }

                setPower(reversedHeading != 0, forwardVector);

                previousHeadingNotFieldOriented = reversedHeading != 0 ? previousHeadingNotFieldOriented + reversedHeading : normalizedHeading;

            }
    }

    /**Returns the normalized heading (shortest angular distance) of the target angle for the turning motors,
     * reducing the amount of distance the turning motors need to turn to be at a certain angle, and also limit how
     * much the turning can turn in sitting, preventing issues like tangled wires from the motors rotating 360 degrees.*/
    public double normalizeHeading(double currentHeading, double targetHeading) {
        return (targetHeading - currentHeading + 540) % 360 - 180;
    }


    @Override
    public void periodic() {
        telemetry.addLine("Swerve Module States");
        telemetry.update();
    }
}
