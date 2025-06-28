package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.DifferentialDrive;

public class DifferentialDriveCommand extends CommandBase {

    /**The {@code DifferentialDrive} subsystem for this command to control.*/
    DifferentialDrive differentialDrive;

    /**Reference to the human driver's Xbox controller.*/
    GamepadEx gamepadEx;

    /**The IMU orientation system of the robot. It returns what the heading the robot is currently facing,
     *  in degrees or radians.*/
    IMU imu;

    /**The boolean state to toggle between if the robot should set its maximum speed to half of its capable speed or to
     * the full amount.*/
    boolean speedToggler = false;

    /**Constructs a new {@code DifferentialDriveCommand()} with an initialized {@code DifferentialDrive} subsystem,
     * {@code GamepadEx} and {@code IMU}.*/
    public DifferentialDriveCommand(DifferentialDrive differentialDrive, IMU imu, GamepadEx gamepadEx) {
        this.differentialDrive = differentialDrive;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
    }

    /**When this method runs, if the driver presses the "A" button, then the robot will reset its gyro, making it think it's
     * at 0 degrees again, but if the user presses the "B" button instead, the robot will then activate a speed toggler which will toggle between
     * whether the robot should sets its maximum speed to half of its capable maximum speed or the full amount.*/
    @Override
    public void execute() {
        com.arcrobotics.ftclib.drivebase.DifferentialDrive differentialDrive1 = differentialDrive.getDifferentialDrive(
                differentialDrive.getDifferentialVector()
        );

        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            differentialDrive.resetGyro(imu);
        } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)){
            speedToggler = !speedToggler;
            differentialDrive1.setMaxSpeed(speedToggler ? 0.5 : 1);
        } else {
            differentialDrive.drive();
        }
    }
}
