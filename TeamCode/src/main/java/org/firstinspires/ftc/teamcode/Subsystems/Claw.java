package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends SubsystemBase {
    /**The robot's claw.*/
    ServoEx claw;

    /**The robot's telemetry system.*/
    Telemetry telemetry;

    /**Constructs a new {@code Claw()} with an initialized claw {@code Servo}
     * and {@code Telemetry}.*/
    public Claw(ServoEx claw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.claw = claw;
        claw.setRange(0, 360);
        claw.setPosition(0);
    }

    /**Rotates the claw by a certain amount of degrees.*/
    public void rotateClawByDegrees(double angle) {
        claw.rotateByAngle(angle);
    }

    /**Updates the robot {@code telemetry} with the position of the claw.*/
    @Override
    public void periodic() {
        telemetry.addLine("CLAW POSITIONS");
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.update();
    }
}
