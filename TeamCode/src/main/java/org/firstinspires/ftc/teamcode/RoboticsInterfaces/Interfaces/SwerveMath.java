package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import java.util.function.BiFunction;
import java.util.function.Function;

public final class SwerveMath {

    public static final BiFunction<Double, Double, Double> calculateForwardVector = SwerveMath::calculateForwardVector;
    public static final BiFunction<Integer, Integer, Integer> calculateNormalizedHeadingForRobot = SwerveMath::calculateNormalizedHeadingForRobot;
    public static final BiFunction<Integer, Integer, int[]> calculateNormalizedHeadingForWheel = (i1, i2) -> {
        int normalizedHeading = normalizedHeading(i1, i2);
        return new int[] {normalizedHeading, i1}; //normalized heading = i[0], current heading = i[1]
    };
    public static final Function<int[], int[]> calculateTotalHeadingForWheel = i -> {
        int totalHeading = calculateTotalHeading(i[1], i[0]);
        return new int[] {totalHeading, i[1], i[0]}; //total heading = i[0], current heading = i[1], normalizedHeading = i[2]
    };
    public static final Function<int[], int[]> calculateReversedHeadingForWheel = i -> {
        int reversedHeading = calculateReversedHeading(i[0], i[1]);
        return new int[] {reversedHeading, i[1], i[2]}; //reversed heading = i[0], current heading = i[1], normalized heading = i[2]
    };
    public static final Function<int[], int[]> calculateTargetPositionAndFinalizeValues = i -> {
        int heading = i[0] != 2000 ? i[0] : i[2];
        int targetPosition = i[1] + heading;
        return new int[] {i[0], i[1], i[2], targetPosition, heading}; //reversed heading = i[0], current heading = i[1], normalized heading = i[2], target position = i[3], heading = i[4]
    };
    private static int normalizedHeading(int currentPosition, int targetPosition) {
        return (targetPosition - currentPosition + 540) % 360 - 180;
    }
    private static double calculateForwardVector(double forwardPower, double sidePower) {
        return Math.hypot(forwardPower, sidePower) / (Math.sqrt(2));
    }
    private static int calculateNormalizedHeadingForRobot(int robotHeadingInDegrees, int targetHeading) {
        return normalizedHeading(robotHeadingInDegrees, targetHeading);
    }
    private static int calculateTotalHeading(int currentHeading, int normalizedHeading) {
        return currentHeading + normalizedHeading;
    }
    private static int calculateReversedHeading(int totalHeading, int currentHeading) {
        return Math.abs(totalHeading) > 180 ? normalizedHeading(
                currentHeading, totalHeading < 0 ? totalHeading + 180 : totalHeading - 180
        ) : 2000;
    }
}
