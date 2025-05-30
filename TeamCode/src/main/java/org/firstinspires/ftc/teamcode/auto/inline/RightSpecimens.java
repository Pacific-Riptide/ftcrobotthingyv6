package org.firstinspires.ftc.teamcode.auto.inline;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.actions.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.auto.actions.RobotWakeAction;
import org.firstinspires.ftc.teamcode.auto.actions.SpecimenPickupAction;

@Autonomous(name =  "Right - 2 Specs, Pickup, Observe Zone")
public class RightSpecimens extends AutonomousRobot {

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
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, claw360, -1000))
                        .waitSeconds(0.5)
                        .lineToX(34)
                        .waitSeconds(0.5)
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota, claw360))
                        .lineToX(20)
                        .turn(Math.toRadians(180+1e-6))
                        .strafeTo(new Vector2d(16, -42.5))

                        // Hang Specimen 1
                        .waitSeconds(0.5)
                        .stopAndAdd(new SpecimenPickupAction(armMotor, rotaMotor, clawServo, clawRota, claw360))

                        .turn(Math.toRadians(179))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, claw360,-1000))
                        .strafeTo(new Vector2d(32, 12.5))

                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota, claw360))

                        //Attempt to sweep first blue sample
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(20, -17.5))

                        .strafeTo(new Vector2d(50, -42.5))
                        .turn(Math.toRadians(-90))

                        //Attempt to sweep one sample into observation zone
                        .strafeTo(new Vector2d(50, -48))
                        .strafeTo(new Vector2d(-1, -48))
                        .build()
        ));
    }
}
