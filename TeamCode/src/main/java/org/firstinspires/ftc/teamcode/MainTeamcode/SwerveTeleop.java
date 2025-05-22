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

    /**Implementation of the robot's swerve drivetrain. This object represents all the four wheels of the robot, their headings, heading states, and
     * booleans for when certain methods running the Swerve Drivetrain haven't/have completed their tasks.*/
    public SwerveDrivetrain swerveDrivetrain;

    /**Initializes the {@code GamepadEx} and the {@code Motor} objects in both {@code Motor}
     * arrays. Also initializes the {@code IMU}, a {@code SwerveDrivetrain} subsystem and a
     * {@code SwerveDriveCommand} command to make the robot's hardware components run. This method also
     * registers the {@code SwerveDrivetrain} subsystem's {@code periodic()} method to be run every cycle, thus
     * all the information regarding the subsystem's functionality will be updated and can be viewed using the subsystem's
     * {@code Telemetry}. This method also sets the 'default command' of {@code SwerveDrivetrain} to the {@code execute()} method
     * in the {@code swerveDriveCommand} object, that way, the {@code setSwerveModuleState()} method is always called no matter what, which
     * is what you want to be able to drive the robot around anytime to any place, over any distance.*/
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

    /**Runs the {@code SwerveTeleop} class. When {@code SwerveTeleop} calls this method, it will check if the button 'A' or 'B' was pressed on the
     * {@code gamepadEx} controller, and if 'A' was pressed, then it will reset the encoders of the turning motors and reset their mode to PositionControl, so that
     * the turning motors only go to a certain distance. If the human driver presses 'B' though, the 'yaw' angle of the IMU system built into the robot will eb reset,
     * so the robot will think that its orientation that it's currently in is 0 degrees, instead of what it was before.*/
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
