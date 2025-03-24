package org.firstinspires.ftc.teamcode.TestingFolder;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainTeamcode.Constants;

/*This file is where we will test new ideas for programming once we come up with them.
*
* We will be using the CommandOpMode class as our standard system for running our robot using a
* command-based system and programming software.*/

@TeleOp(name = "PracticeFile", group = "teamcode")
public class PracticeFile extends CommandOpMode {
    PracticeDrivetrain drivetrain;
    Motor[] motors = new Motor[]{
            new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
            new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
            new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
            new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)};
    @Override
    public void initialize() {
        drivetrain = new PracticeDrivetrain(telemetry, motors);
        register(drivetrain);
    }

    @Override
    public void run() {
        telemetry.update();
    }
}
