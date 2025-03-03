package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Otos_Mecanum.MecanumDrive;

@Autonomous(name = "Auto", group = "teamcode")
public class Auto extends CommandOpMode {
    PIDController pidController;
    MecanumDrive mecanumDrive;

    @Override
    public void initialize() {
        pidController = new PIDController(0.2, 0.05, 0.1);
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void run() {
        mecanumDrive.updatePoseEstimate();

        telemetry.addLine("ROBOT X AND Y");
        telemetry.addData("Robot Position X", mecanumDrive.pose.position.x);
        telemetry.addData("Robot Position Y", mecanumDrive.pose.position.y);
        telemetry.update();
    }
}
