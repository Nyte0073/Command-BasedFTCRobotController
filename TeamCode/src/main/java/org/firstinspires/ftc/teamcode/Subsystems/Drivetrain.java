package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain extends SubsystemBase {
    /**Drivetrain motor.*/
    private final Motor frontLeft, frontRight, backLeft, backRight;

    /**The robot's gyroscope system.*/
    public final IMU imu; 

    /**The robot's telemetry system.*/
    private final Telemetry telemetry; 

    /**A reference to the robot's mecanum drive ability.*/
    public final MecanumDrive mecanumDrive;

    /**The driver's controller.*/
    private final GamepadEx gamepadEx; 

    /**The robot's kinematics system.*/
    public MecanumDriveKinematics kinematics = new MecanumDriveKinematics(new Translation2d(7.5625, -7.375), new Translation2d(7.5625, 7.375),
            new Translation2d(-7.5625, -7.375), new Translation2d(-7.5625, 7.375));

    /**The robot's odometry system.*/
    public MecanumDriveOdometry odometry = new MecanumDriveOdometry( // Mecanum Drive odometry. Don't worry about this for now.
            kinematics, new Rotation2d(0)
    );

   /**The wheel speeds of the robot.*/
    public final MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(); //Wheel speeds of all motors. Don't worry about this for now.

    /**Constructs a new {@code Drivetrain()} with built-in {@code Telemetry}, and initialized {@code Motor}'s, {@code IMU}
     * and {@code GamepadEx}.*/
    public Drivetrain(Telemetry telemetry, Motor[] motors, IMU imu, GamepadEx gamepadEx) { //Initializing motors within drivetrain class itself.
        frontLeft = motors[0];
        frontRight = motors[1];
        backLeft = motors[2];
        backRight = motors[3];

        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); //Setting the default mode for the motors with zero power to brake mode.
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.imu = imu;
        this.telemetry = telemetry;
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.gamepadEx = gamepadEx;

        frontLeft.setInverted(true); //Reversing the two left motors, so all the motors travel the same direction.
        backLeft.setInverted(true);
        mecanumDrive.setRightSideInverted(false); //Making the sure the MecanumDrive class doesn't reverse two right motors by default.

        mecanumDrive.setRange(-1, 1); //Setting the range for the motor powers.
    }

    /**Makes the robot drive either field oriented or robot oriented.*/
    public void drive(boolean fieldOriented) { //Will drive either normally or field oriented depending on the state of fieldOriented.
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if(fieldOriented) {
            mecanumDrive.driveFieldCentric(gamepadEx.getLeftX(), Math.abs(gamepadEx.getLeftY()) <= 0.08 ? 0 : gamepadEx.getLeftY(), -gamepadEx.getRightX(), heading);
        } else {
            mecanumDrive.driveRobotCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), -gamepadEx.getRightX());
        }
    }

    /**Drives the robot manually using inputted wheel speeds.*/
    public void set(MecanumDriveWheelSpeeds wheelSpeeds) {
        frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond);
        backLeft.set(wheelSpeeds.rearLeftMetersPerSecond);
        frontRight.set(wheelSpeeds.frontRightMetersPerSecond);
        backRight.set(wheelSpeeds.rearRightMetersPerSecond);
    }

    /**Returns the drivetrain's {@code gamepadEx} variable.
     * @return {@code gamepadEx} - The robot's Xbox controller.*/
    public GamepadEx getGamepadEx() {
        return gamepadEx;
    }

    /**Updates the robot's wheel speed system with the current speeds of the robot's wheels.*/
    public void updateWheelsSpeeds() {
        wheelSpeeds.frontLeftMetersPerSecond = (frontLeft.getCorrectedVelocity() * 0.3192) / 1440;
        wheelSpeeds.frontRightMetersPerSecond = (frontRight.getCorrectedVelocity() * 0.3192) / 1440;
        wheelSpeeds.rearLeftMetersPerSecond = (backLeft.getCorrectedVelocity() * 0.3192) / 1440;
        wheelSpeeds.rearRightMetersPerSecond = (backRight.getCorrectedVelocity() * 0.3192) / 1440;
    }

    /**Updates the robot's odometry system with the current X and Y coordinate of the robot on the field,
     * and also the orientation (in degrees) the robot is currently facing.*/
    public void updateOdometry() {
       odometry.updateWithTime((double) System.currentTimeMillis() / 1000,
               Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), wheelSpeeds);
    }

    /**Returns the X and Y position of the robot.
     * @return {@code odometry.getPoseMeters()} - The X and Y position of the robot on the field (in meters).*/
    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    /**Returns the robot's kinematics system.
     * @return {@code kinematics} - The robot's kinematics system.*/
    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    /**Updates the {@code telemetry} with the robot's current motor powers, IMU orientation, and
     * the max RPM of all the motors.*/
    @Override
    public void periodic() { //Returns motor powers and the robot's rotation calculated from the IMU.
        updateWheelsSpeeds(); //This method has to be called first before the odometry is updated. 
        updateOdometry();

        telemetry.addLine("MOTOR POWERS");
        telemetry.addData("FrontLeft Power", frontLeft.get());
        telemetry.addData("FrontRight Power Level", frontRight.get());
        telemetry.addData("BackLeft Power", backLeft.get());
        telemetry.addData("BackRight Power", backRight.get());

        telemetry.addLine("IMU");
        telemetry.addData("IMU orientation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Odometer Position", getRobotPose().toString());

        telemetry.addLine("MOTOR RPM");
        telemetry.addData("Front Left Max RPM", frontLeft.getMaxRPM());
        telemetry.addData("Front Right Max RPM", frontRight.getMaxRPM());
        telemetry.addData("Back Left Max RPM", backLeft.getMaxRPM());
        telemetry.addData("Back Right Max RPM", backRight.getMaxRPM());
    }
}
