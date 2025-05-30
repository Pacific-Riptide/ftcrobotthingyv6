package org.firstinspires.ftc.teamcode.auto.inline;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.actions.ClawLowBarTouchAction;
import org.firstinspires.ftc.teamcode.auto.actions.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.auto.actions.RobotWakeAction;
import org.firstinspires.ftc.teamcode.auto.actions.SamplePickupAction;
import org.firstinspires.ftc.teamcode.auto.actions.ScoreSampleBackAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

@Autonomous(name = "Left - 2 Samples, 1 Spec")
public class LeftSamplesAlt extends AutonomousRobot {

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

        // Start telemetry updates
        telemetryThread.start();
        Actions.runBlocking(
                drive
                        .actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, claw360, -750))
                        .waitSeconds(0.5)
                        .lineToX(32)
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota, claw360))
                        .strafeTo(new Vector2d(9, 46))
                        .waitSeconds((0.0))
                        .turn(Math.toRadians(24))
                        .stopAndAdd(new SamplePickupAction(armMotor, rotaMotor, clawServo, clawRota, claw360, RobotConstants.ARM_STRAIGHT_SAMPLE_PICK_UP_HEIGHT - 300))
                        .waitSeconds((0.0))
                        .turn(Math.toRadians(-26.5))
                        .strafeTo(new Vector2d(8, 59))
                        .waitSeconds((0.0))
                        .stopAndAdd(new ScoreSampleBackAction(armMotor, rotaMotor, clawServo, clawRota, claw360))
                        .turn(Math.toRadians(25))
                        .strafeTo(new Vector2d(9, 58))
                        .waitSeconds((0.0))
                        .stopAndAdd(new SamplePickupAction(armMotor, rotaMotor, clawServo, clawRota, claw360,RobotConstants.ARM_STRAIGHT_SAMPLE_PICK_UP_HEIGHT - 200))
                        .waitSeconds((0.0))
                        .strafeTo(new Vector2d(8, 59))
                        .turn(Math.toRadians(-25))
                        .stopAndAdd(new ScoreSampleBackAction(armMotor, rotaMotor, clawServo, clawRota, claw360))
                        .build());
    }
}