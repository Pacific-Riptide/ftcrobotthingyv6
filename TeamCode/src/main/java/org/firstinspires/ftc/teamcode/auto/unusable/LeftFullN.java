/*package org.firstinspires.ftc.teamcode.auto.unusable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.actions.ClawLowBarTouchAction;
import org.firstinspires.ftc.teamcode.auto.actions.SamplePickupAction;
import org.firstinspires.ftc.teamcode.auto.actions.ScoreSampleBackAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
@Disabled
@Autonomous
public class LeftFullN extends AutonomousRobot {

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
                        /*.strafeTo(new Vector2d(0, -5))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota))
                        .waitSeconds(1)
                        .lineToX(32)
                        .waitSeconds(1)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(14, 47.5))
                        .waitSeconds(1)
                        .stopAndAdd(new SamplePickupAction(armMotor, rotaMotor, clawServo, clawRota, RobotConstants.ARM_STRAIGHT_SAMPLE_PICK_UP_HEIGHT))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(14, 58))
                        .turn(Math.toRadians(-25))
                        .waitSeconds(1)
                        .stopAndAdd(new ScoreSampleBackAction(armMotor, rotaMotor, clawServo, clawRota))
                        .turn(Math.toRadians(25))
                        .waitSeconds(1)
                        .stopAndAdd(new SamplePickupAction(armMotor, rotaMotor, clawServo, clawRota, RobotConstants.ARM_STRAIGHT_SAMPLE_PICK_UP_HEIGHT))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-20))
                        .stopAndAdd(new ScoreSampleBackAction(armMotor, rotaMotor, clawServo, clawRota))
                        //.stopAndAdd(new SamplePickupAction(armMotor, rotaMotor, clawServo, clawRota, RobotConstants.ARM_DIAGONAL_SAMPLE_PICK_UP_HEIGHT))
                        // .waitSeconds(1)
                        //.turn(Math.toRadians(-45))
                        // .waitSeconds(1)
                        // .stopAndAdd(new ScoreSampleBackAction(armMotor, rotaMotor, clawServo, clawRota))
                        // .waitSeconds(1)
                        //Park in observation zone
                        .strafeTo(new Vector2d(50, -59))
                        .turn(Math.toRadians(-70))
                        .lineToY(13)
                        .stopAndAdd(new ClawLowBarTouchAction(armMotor, rotaMotor, clawServo, clawRota))
                        .build());
        }
}*/





