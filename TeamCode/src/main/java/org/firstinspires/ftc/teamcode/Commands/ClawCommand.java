package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class ClawCommand extends CommandBase {
    private final Claw claw;
    private final GamepadEx gamepadEx;

    public ClawCommand(Claw claw, GamepadEx gamepadEx) {
        this.claw = claw;
        this.gamepadEx = gamepadEx;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.rotateClawByDegrees(360 / gamepadEx.getLeftX());
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
