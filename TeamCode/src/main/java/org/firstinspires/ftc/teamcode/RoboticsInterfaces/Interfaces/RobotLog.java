package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.ThreadBasedSwerveDrive;

import java.util.List;

public class RobotLog {

    private final ThreadBasedSwerveDrive threadBasedSwerveDrive = new ThreadBasedSwerveDrive();
    public static class SwerveLog {

    }

    public static class ThreadBasedSwerveLog {
        boolean[] headingsReversed = new boolean[4], headingsNegativeOrNot = new boolean[4],
        wheelsHaveRotated = new boolean[4];

        double forwardVector = 0, normalizedHeading = 0;

        boolean previousTurningLeft = false, fieldOriented = false, alreadyRotated = false;

        public Object[] setThreadBasedSwerveLog(List <boolean[]> booleanArrays, List <Double> forwardNormalized, List <Boolean> booleans) {
            headingsReversed = booleanArrays.get(0);
            headingsNegativeOrNot = booleanArrays.get(1);
            wheelsHaveRotated = booleanArrays.get(2);

            forwardVector = forwardNormalized.get(0);
            normalizedHeading = forwardNormalized.get(1);

            previousTurningLeft = booleans.get(0);
            fieldOriented = booleans.get(1);
            alreadyRotated = booleans.get(2);

            return new Object[] {headingsReversed, headingsNegativeOrNot, wheelsHaveRotated, forwardVector, normalizedHeading,
            previousTurningLeft, fieldOriented, alreadyRotated};
        }
    }

    public static class HolonomicLog {

    }

    public static class MecanumLog {

    }

    public static class DifferentialLog {

    }
}
