package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "teamcode")
public class Auto extends CommandOpMode {
    PIDController pidController;

    @Override
    public void initialize() {
        pidController = new PIDController(0.2, 0.05, 0.1);
    }

    @Override
    public void run() {

    }
}
