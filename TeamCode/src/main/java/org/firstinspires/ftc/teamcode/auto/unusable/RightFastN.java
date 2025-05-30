/*package org.firstinspires.ftc.teamcode.auto.unusable;

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
public class RightFastN extends AutonomousRobot {

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

        Actions.runBlocking(new SequentialAction(
                drive
                        .actionBuilder(new Pose2d(0, 0, 0))


                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, -750))
                        .waitSeconds(0.5)
                        .lineToX(32)
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))

                        .turn(Math.toRadians(179))


                        // Hang Specimen 1
                        .strafeTo(new Vector2d(14, -45))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SpecimenPickupAction(armMotor, rotaMotor, clawServo, clawRota))
                        .turn(Math.toRadians(-179))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, -750))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(35, 5))
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))

                        //Park in observation zone
                        .strafeTo(new Vector2d(6, -45))
                        .build()));
    }
}*/
