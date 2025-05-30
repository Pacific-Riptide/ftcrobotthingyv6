/*package org.firstinspires.ftc.teamcode.auto.unusable;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.actions.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.auto.actions.RobotWakeAction;
import org.firstinspires.ftc.teamcode.auto.actions.SpecimenPickupAction;
@Disabled
@Autonomous
public class RightShortcutParallelN extends AutonomousRobot {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        super.initializeRobot();
        super.setRobotBehavior();
        super.startRobot();

        // Start a background thread for telemetry updates
        Thread telemetryThread = new Thread(() -> {
            while (opModeIsActive()) {
                super.displayTelemetry(); // Update robot telemetry data
                try {
                    Thread.sleep(100); // Update every 100ms
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });

        // Wait for the game to start
        waitForStart();
        telemetryThread.start();

        Actions.runBlocking(new ParallelAction(
                drive
                        .actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, -750))
                        .waitSeconds(0.5)
                        .lineToX(30)
                        .build()));

        Actions.runBlocking(new SequentialAction(
                drive
                        .actionBuilder(new Pose2d(30, 0, 0))
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))
                        .build()));

        Actions.runBlocking(new ParallelAction(
                drive
                        .actionBuilder(new Pose2d(30, 0, 0))
                        .turn(Math.toRadians(179))
                        .strafeTo(new Vector2d(14, -45))
                        .build()
        ));
        Actions.runBlocking(new SequentialAction(
                drive
                        .actionBuilder(new Pose2d(14, -45, 0))

                        // Hang Specimen 1
                        .waitSeconds(0.5)
                        .stopAndAdd(new SpecimenPickupAction(armMotor, rotaMotor, clawServo, clawRota))
                        .build()));

        Actions.runBlocking(new ParallelAction(
                drive
                        .actionBuilder(new Pose2d(14, -45, 180))
                        .turn(Math.toRadians(179))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, -750))
                        .strafeTo(new Vector2d(33.5, 10))
                        .build()));

        Actions.runBlocking(new SequentialAction(
                drive
                        .actionBuilder(new Pose2d(33.5, 10, 0))
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))

                        //Attempt to sweep first blue sample
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(20, -20))
                        .build()));



        Actions.runBlocking(new ParallelAction(
                drive
                        .actionBuilder(new Pose2d(20, -20, 0))
                        .strafeTo(new Vector2d(50, -45))
                        .turn(Math.toRadians(-90))
                        .build()
        ));

        //Attempt to sweep one sample into observation zone
        Actions.runBlocking(new SequentialAction(
                drive
                        .actionBuilder(new Pose2d(50, -45, 0))
                        .strafeTo(new Vector2d(50, -50))
                        .strafeTo(new Vector2d(6, -50))
                        .build()
        ));
    }
}*/
