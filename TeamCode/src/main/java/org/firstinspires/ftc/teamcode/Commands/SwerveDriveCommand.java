package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrivetrain;


public class SwerveDriveCommand extends CommandBase {
    volatile SwerveDrivetrain swerveDrivetrain;
    GamepadEx gamepadEx;
    IMU imu;
    boolean fieldOriented;
    public SwerveDriveCommand(SwerveDrivetrain swerveDrivetrain, GamepadEx gamepadEx, IMU imu, boolean fieldOriented) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
        this.fieldOriented = fieldOriented;
        addRequirements(swerveDrivetrain);
    }

    @Override
    public void execute() {
            swerveDrivetrain.setSwerveModuleState(fieldOriented);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
