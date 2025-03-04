package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Otos_Mecanum.MecanumDrive;

@Autonomous(name = "Auto", group = "teamcode")
public class Auto extends CommandOpMode { //Class for making the robot function autonomously (on its own).
    PIDController pidController; //PID controller for controlling the drivetrain's speed over certain distances.
    MecanumDrive mecanumDrive; //Drivetrain class used for keeping track of the drivetrain's position on the field (in x and y coordinates).

    @Override
    public void initialize() { //This is where you will put any variable that will be initialized during the opMode's initialization period.
        pidController = new PIDController(0.2, 0.05, 0.1); //Initializing PID controller with modest drive constants.
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); //Initializing MecanumDrive with an initial position of 0.
    }

    @Override
    public void run() { //This is where you will put any code you want to run during the "auto" phase.
        mecanumDrive.updatePoseEstimate(); //Update the robot with its current position.

        telemetry.addLine("ROBOT X AND Y"); //Using telemetry to output the robot's position to the Driver Hub.
        telemetry.addData("Robot Position X", mecanumDrive.pose.position.x);
        telemetry.addData("Robot Position Y", mecanumDrive.pose.position.y);
        telemetry.update();
    }
}
