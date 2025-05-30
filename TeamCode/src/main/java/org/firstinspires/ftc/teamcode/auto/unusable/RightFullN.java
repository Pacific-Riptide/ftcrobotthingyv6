/*package org.firstinspires.ftc.teamcode.auto.unusable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.actions.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.auto.actions.RobotWakeAction;
import org.firstinspires.ftc.teamcode.auto.actions.SampleDropAction;
import org.firstinspires.ftc.teamcode.auto.actions.SamplePickupAction;
import org.firstinspires.ftc.teamcode.auto.actions.SpecimenPickupAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
@Disabled
@Autonomous
public class RightFullN extends AutonomousRobot {

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

        Actions.runBlocking(
                drive
                        .actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, -750))
                        .waitSeconds(0.5)
                        .lineToX(32)
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))
                        .waitSeconds(0.5)
                        .lineToX(14)
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(14, -47.5))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SamplePickupAction(armMotor, rotaMotor, clawServo, clawRota, RobotConstants.ARM_STRAIGHT_SAMPLE_PICK_UP_HEIGHT))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(14, -59))
                        .turn(Math.toRadians(180+1e-6))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SampleDropAction(armMotor, rotaMotor, clawServo, clawRota))
                        .waitSeconds(0.5)
                        .turn(Math.toRadians(180 + 1e-6))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SamplePickupAction(armMotor, rotaMotor, clawServo, clawRota, RobotConstants.ARM_STRAIGHT_SAMPLE_PICK_UP_HEIGHT))
                        .waitSeconds(0.5)
                        .turn(Math.toRadians(-180 + 1e-6))
                        .stopAndAdd(new SampleDropAction(armMotor, rotaMotor, clawServo, clawRota))
                        .waitSeconds(0.5)

                        // Hang Specimen 1
                        .strafeTo(new Vector2d(14, -45))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SpecimenPickupAction(armMotor, rotaMotor, clawServo, clawRota))
                        .turn(Math.toRadians(180+1e-6))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, -750))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(35, 5))
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))

                        // Hang Specimen 2
                        .strafeTo(new Vector2d(14, -45))
                        .waitSeconds(0.5)
                        .turn(Math.toRadians(180+1e-6))
                        .stopAndAdd(new SpecimenPickupAction(armMotor, rotaMotor, clawServo, clawRota))
                        .waitSeconds(0.5)
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, -750))
                        .waitSeconds(0.5)
                        .turn(Math.toRadians(180+1e-6))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(38, 10))
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota))

                        //Park in observation zone
                        .strafeTo(new Vector2d(6, -45))
                        .build());
    }
}*/
