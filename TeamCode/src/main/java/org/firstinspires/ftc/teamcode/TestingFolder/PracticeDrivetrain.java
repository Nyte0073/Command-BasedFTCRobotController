package org.firstinspires.ftc.teamcode.TestingFolder;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PracticeDrivetrain extends SubsystemBase {
    private Telemetry telemetry;
    private Motor frontLeft, backLeft, frontRight, backRight;
    public MecanumDrive mecanumDrive;

    public PracticeDrivetrain(Telemetry telemetry, @NonNull Motor[] motors) {
        this.telemetry = telemetry;
        frontLeft = motors[0];
        frontRight = motors[1];
        backLeft = motors[2];
        backRight = motors[3];

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void periodic() {
       telemetry.addLine("Drivetrain Telemetry");
       //Add your telemetry output here for drivetrain stuff.
    }
}
