package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Commands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrivetrain;

@TeleOp(name = "SwerveTeleop", group = "teamcode")
public class SwerveTeleop extends CommandOpMode {

    /**Reference to the driver's gamepad controller.*/
    public GamepadEx gamepadEx;

    /**{@code Motor}'s to either drive the robot forward/backward or point the robot in the direction of a certain
     * movement vector.*/
    public Motor[] turningMotors, drivingMotors;

    public SwerveDrivetrain swerveDrivetrain;

    /**Initializes the {@code GamepadEx} and the {@code Motor} objects in both {@code Motor}
     * arrays. Also initializes the {@code IMU}, a {@code SwerveDrivetrain} subsystem and a
     * {@code SwerveDriveCommand} command to make the robot's hardware components run. This method also
     * registers the {@code SwerveDrivetrain} subsystem's {@code periodic()} method to be run every cycle, thus
     * all the information regarding the subsystem's functionality will be updated and can be viewed using the subsystem's
     * {@code Telemetry}.*/
    @Override
    public void initialize() {
         gamepadEx = new GamepadEx(gamepad1);
         turningMotors = new Motor[] {
                new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)
        };
         drivingMotors = new Motor[] {
                new Motor(hardwareMap, Constants.SwerveConstants.frontLeftDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.frontRightDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.backLeftDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.backRightDriving)
        };
        IMU imu = hardwareMap.get(IMU.class, "imu");
        swerveDrivetrain = new SwerveDrivetrain(telemetry, turningMotors, drivingMotors, imu);
        SwerveDriveCommand swerveDriveCommand = new SwerveDriveCommand(swerveDrivetrain, gamepadEx, imu, false);
        swerveDrivetrain.setDefaultCommand(swerveDriveCommand);
        register(swerveDrivetrain);
    }

    @Override
    public void run() {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            for(Motor m : turningMotors) {
                m.stopAndResetEncoder();
                m.setRunMode(Motor.RunMode.PositionControl);
            }
        }

        if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            swerveDrivetrain.resetGyro();
        }
    }
}
