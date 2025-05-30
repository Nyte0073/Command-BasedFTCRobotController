package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Holonomic;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.RobotVector;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;


public class HolonomicDrive extends Holonomic {

    GamepadEx gamepadEx;

    Motor frontMiddle, backLeft, backRight;

    IMU imu;

    Telemetry telemetry;

    public AtomicBoolean asyncSettingPowerMethodFinished = new AtomicBoolean(false);

    private boolean fieldOriented;

    public HolonomicDrive(GamepadEx gamepadEx, Motor[] motors, IMU imu, Telemetry telemetry, boolean fieldOriented) {
        this.gamepadEx = gamepadEx;
        this.imu = imu;
        this.telemetry = telemetry;
        this.fieldOriented = fieldOriented;

        frontMiddle = motors[0];
        backLeft = motors[1];
        backRight = motors[2];

        for(Motor m : motors) {
            m.stopAndResetEncoder();
            m.setRunMode(Motor.RunMode.PositionControl);
        }

        imu.resetYaw();
    }

    @Override
    public void setPower(int headingInDegrees, boolean fieldOriented, double forwardPower, double sidePower, double turningVector, boolean turningLeft) {
        int heading = (Math.abs(forwardPower) <= 0.01 && Math.abs(sidePower) <= 0.01) ? 0 :
                (int) Math.toDegrees(Math.atan2(forwardPower, sidePower)) - 90;

        if(fieldOriented) {
            driveFieldOriented(heading, headingInDegrees, forwardPower, sidePower, turningVector, turningLeft);
        } else {
            driveRobotOriented(heading, forwardPower, sidePower, turningVector, turningLeft);
        }
    }

    public void driveFieldOriented(int heading, int headingInDegrees, double forwardPower, double sidePower, double turningVector, boolean turningLeft) {
        if(!asyncSettingPowerMethodFinished.get()) {
            return;
        } else if(turningVector < 0.05) {
            asyncSettingPowerMethodFinished.set(false);
        }


    }

    public void driveRobotOriented(int headingInDegrees, double forwardPower, double sidePower, double turningVector, boolean turningLeft) {
        if(!asyncSettingPowerMethodFinished.get()) {
            return;
        } else if(turningVector < 0.05) {
            asyncSettingPowerMethodFinished.set(false);
        }
    }

    public void setPowerForDrive(double frontMiddlePower, double backLeftPower, double backRightPower) {
        CompletableFuture.runAsync(() -> {
            frontMiddle.set(frontMiddlePower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);
        }).thenRun(() -> {
            asyncSettingPowerMethodFinished.set(true);
        });
    }

    @Override
    public void stopMotors() {
        frontMiddle.set(0);
        backLeft.set(0);
        backRight.set(0);
    }

    @Override
    public RobotVector getRobotVector() {
        return new RobotVector((int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), -gamepadEx.getLeftY(),
                gamepadEx.getLeftX(), gamepadEx.getRightX(), fieldOriented,
                gamepadEx.getRightX() != Math.abs(gamepadEx.getRightX()));
    }

    @Override
    public void periodic() {
        telemetry.update();
    }
}
