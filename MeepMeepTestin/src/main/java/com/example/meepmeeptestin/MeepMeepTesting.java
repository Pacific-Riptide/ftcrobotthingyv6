package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Define your custom origin (-10, 67)
        Vector2d customOrigin = new Vector2d(-10, 67);

        // Initialize MeepMeep
        MeepMeep meepMeep = new MeepMeep(800);
        meepMeep.setMouseCoordinateDisplayPosition(10, 20);

        // Add trajectory simulation
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true) // Optional dark theme
                .setBackgroundAlpha(0.95f)
                .addEntity(
                        new DefaultBotBuilder(meepMeep)
                                .setDimensions(18, 18) // Set robot dimensions
                                .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(
                                                        new Pose2d(-10, 67, Math.toRadians(90)) // Start at (-10, 67)
                                                )
                                                .lineTo(translateToCustomOrigin(new Vector2d(34, 0), customOrigin)) // Example path (downward)
                                              //  .splineTo(new Vector2d(0, 20), Math.toRadians(0)) // Example path (to the right)
                                                .build()
                                )
                )
                .start();
    }

    // Helper to translate a Pose2d for custom origin and real-life coordinate system
    private static Pose2d translateToCustomOrigin(Pose2d pose, Vector2d origin) {
        return new Pose2d(
                origin.getY() - pose.getX(),         // Transform real-life X = MeepMeep Y
                origin.getX() - pose.getY(),         // Transform real-life Y = -MeepMeep X
                pose.getHeading()
        );
    }

    // Helper to translate a Vector2d for custom origin and real-life coordinate system
    private static Vector2d translateToCustomOrigin(Vector2d point, Vector2d origin) {
        return new Vector2d(
                origin.getY() - point.getX(),
                origin.getX() - point.getY()
        );
    }
}
