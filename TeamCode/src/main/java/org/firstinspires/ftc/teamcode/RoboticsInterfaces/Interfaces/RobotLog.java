package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import java.util.List;

/**Class containing all the inner classes that pertain to each drivetrain type, and they are responsible for gathering and cloning all
 * the information gathered during robot operation and storing it in one place, ready to be transferred over to another class for display
 * in a GUI.*/
public final class RobotLog {

    /**Responsible for cloning and gathering robot information for the {@code SwerveDrive} class.*/
    public static class SwerveLog {

    }

    /**Responsible for cloning and gathering robot information for the {@code ThreadBasedSwerveDrive} class.*/
    public static class ThreadBasedSwerveLog {
        boolean[] headingsReversed = new boolean[4], headingsNegativeOrNot = new boolean[4],
        wheelsHaveRotated = new boolean[4];

        double forwardVector = 0, normalizedHeading = 0;

        boolean previousTurningLeft = false, fieldOriented = false, alreadyRotated = false;
        int[] targetPositions = new int[4];

        public void setThreadBasedSwerveLog(List <boolean[]> booleanArrays, List <Double> forwardNormalized, List <Boolean> booleans, int[] targetPositions) {
            headingsReversed = booleanArrays.get(0);
            headingsNegativeOrNot = booleanArrays.get(1);
            wheelsHaveRotated = booleanArrays.get(2);

            forwardVector = forwardNormalized.get(0);
            normalizedHeading = forwardNormalized.get(1);

            previousTurningLeft = booleans.get(0);
            fieldOriented = booleans.get(1);
            alreadyRotated = booleans.get(2);
            this.targetPositions = targetPositions;
        }
    }

    /**Responsible for cloning and gathering robot information for the {@code HolonomicDrive} class.*/
    public static class HolonomicLog {

    }

    /**Responsible for cloning and gathering robot information for the {@code MecanumDrive} class.*/
    public static class MecanumLog {

    }

    /**Responsible for cloning and gathering robot information for the {@code DifferentialDrive} class.*/
    public static class DifferentialLog {

    }
}
