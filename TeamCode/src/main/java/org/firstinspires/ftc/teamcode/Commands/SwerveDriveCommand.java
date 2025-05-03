package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrivetrain;


public class SwerveDriveCommand extends CommandBase {

    /**Reference to the drivetrain subsystem of the robot.*/
    SwerveDrivetrain swerveDrivetrain;

    /**Reference to the driver's gamepad controller.*/
    GamepadEx gamepadEx;

    /**The robot's angle orientation system. Returns the current orientation of the robot
     * (in degrees or radians depending on the setting applied).*/
    IMU imu;

    /**State of whether the robot is driving field-centric or robot-centric.*/
    boolean fieldOriented;

    /**Constructs a new {@code SwerveDriveCommand()} with initialized {@code SwerveDrivetrain} class, {@code GamepadEx},
     * {@code IMU}, boolean state stored in the {@code fieldOriented} variable, and registers the {@code SwerveDrivetrain} variable
     * for getting its {@code periodic()} method run every cycle to update the robot's telemetry with current information about the different
     * states of the drive subsystem.*/
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
