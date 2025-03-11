package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Commands.ButtonCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.TriggerCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

import java.util.List;

@TeleOp(name = "Teleop", group = "teamcode")
public class Teleop extends CommandOpMode { //Main class for making the robot function using controller outputs from the human driver.

    /**The drivetrain's motors.*/
    Motor[] motors;

    /**The robot's gyroscope system.*/
    static IMU imu;

    /**The driver's controller.*/
    static GamepadEx gamepadEx;

    /**Reader for one of the driver's controller's triggers.*/
   public static TriggerReader leftReader, rightReader;

   /**Robot's vision system.*/
   HuskyLens huskyLens;

   /**Vision subsystem class for updating the state of the robot based on the output from the HuskyLens.*/
    static Vision vision;
    static VisionCommand visionCommand;

   List<GamepadKeys.Button> buttons = List.of( //List of game pad controls, here for referencing in the code.
           GamepadKeys.Button.A,
           GamepadKeys.Button.B,
           GamepadKeys.Button.X,
           GamepadKeys.Button.Y,
           GamepadKeys.Button.DPAD_DOWN,
           GamepadKeys.Button.DPAD_UP,
           GamepadKeys.Button.DPAD_LEFT,
           GamepadKeys.Button.DPAD_RIGHT
   );

    @Override
    public void initialize() { //This is where you will initialize any variables and set default commands and register subsystems.
        gamepadEx = new GamepadEx(gamepad1);

         imu = hardwareMap.get(IMU.class, "imu"); //Use hardwareMap.get() method to initialize IMU.

        final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)); //This is the direction the IMU is facing on the robot. DO NOT change.

        imu.initialize(parameters);

        sleep(500);

        motors = new Motor[]{new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor), //Initializing motors.
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)};

        //Drivetrain class to control the four motors;
        Drivetrain drivetrain = new Drivetrain(telemetry, motors, imu, gamepadEx); //Initializing drivetrain.
        DriveCommand driveCommand = new DriveCommand(motors, imu, gamepadEx, true, drivetrain); //Setting up drive command.

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens"); //Initializing huskyLens.
        vision = new Vision(huskyLens, telemetry); //Initializing vision.

        visionCommand = new VisionCommand(vision, VisionCommand.States.NOT_IN_RANGE);
        vision.setDefaultCommand(visionCommand);

        leftReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.LEFT_TRIGGER); //Setting up readers for both left and right trigger inputs.
        rightReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.RIGHT_TRIGGER);

        register(drivetrain, vision); //Registering drivetrain subsystem for having its periodic() method called automatically.

        drivetrain.setDefaultCommand(driveCommand); //Setting the default command for drivetrain.

        imu.resetYaw(); //Resetting IMU upon every initialization.
    }

    @Override
    public void run() {
        gamepadEx.readButtons(); //Reading button input values every loop.

        if(leftReader.isDown()) {//If left or right trigger was pressed, schedule a new TriggerCommand().
            TriggerCommand.leftTriggerPressed = true;
            schedule(new TriggerCommand(telemetry));
        } else if(rightReader.isDown()) {
            TriggerCommand.rightTriggerPressed = true;
            schedule(new TriggerCommand(telemetry));
        }

        for(GamepadKeys.Button button : buttons) { //If any button pressed, schedule a new ButtonCommand().
            if(gamepadEx.wasJustPressed(button)) {
                schedule(new ButtonCommand(gamepadEx, button, telemetry, imu));
            }
        }

        CommandScheduler.getInstance().run(); //Run every scheduled command.
        telemetry.update();

        sleep(20);
    }
}
