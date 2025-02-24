package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends SubsystemBase {
    ServoEx claw;
    Telemetry telemetry;
    public Claw(ServoEx claw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.claw = claw;
        claw.setRange(0, 360);
    }

    public void rotateClawByDegrees(double angle) {
        claw.rotateByAngle(angle);
    }

    public void reset() {
        claw.setPosition(0);
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }

    @Override
    public void periodic() {
        telemetry.addLine("CLAW POSITIONS");
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.update();
    }
}
