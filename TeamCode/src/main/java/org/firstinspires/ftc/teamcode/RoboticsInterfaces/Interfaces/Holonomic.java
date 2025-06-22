package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.Map;


public abstract class Holonomic extends SubsystemBase implements Driveable {
    @Override
    public void stop() {
        stopMotors();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    @Override
    public void drive() {
        String driveType = getDriveType();
        List <Motor[]> hMotors = getHolonomicMotors();
        double[] motorAngles = getHolonomicMotorAngles();

        HolonomicVector holonomicVector = new HolonomicVector(hMotors.get(0), hMotors.get(0), motorAngles[0],
                motorAngles[1], motorAngles[2], driveType);

        HDrive drive = holonomicVector.drivetrain;
        boolean fieldOriented = getFieldOriented();
        GamepadEx gamepadEx = getGamepadEx();
        IMU imu = getIMU();

        if(fieldOriented) {
           drive.driveFieldCentric(gamepadEx.getLeftX(), -gamepadEx.getLeftY(), gamepadEx.getRightX(),
                   imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        } else {
            drive.driveRobotCentric(gamepadEx.getLeftX(), -gamepadEx.getLeftY(), gamepadEx.getRightX());
        }
    }

    public abstract String getDriveType();

    public abstract List<Motor[]> getHolonomicMotors();

    public abstract double[] getHolonomicMotorAngles();

    public abstract boolean getFieldOriented();

    public abstract GamepadEx getGamepadEx();

    public abstract IMU getIMU();

    public abstract void stopMotors();
}
