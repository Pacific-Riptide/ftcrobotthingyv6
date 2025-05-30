package org.firstinspires.ftc.teamcode.auto.inline;

import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.auto.actions.armAction;
import org.firstinspires.ftc.teamcode.auto.actions.clawAction;
import org.firstinspires.ftc.teamcode.auto.actions.clawRotaAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

@Autonomous(name =  "Right - 3 Specs, Observe Zone")
public class RightSpecimensUlt extends AutonomousRobot {

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

        Actions.runBlocking(new SequentialAction( //make parallel
                drive
                        .actionBuilder(new Pose2d(0, 0, 0))

                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, claw360,-1100))
                        .lineToX(34)
                        .stopAndAdd( new armAction( armMotor, -250))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLOSED_CLAW))
                        .waitSeconds(0.3)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        .splineToLinearHeading(new Pose2d(16, -26, 0), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(58, -37, 0), 2*Math.PI)
                        .lineToX(6)
                        .splineToLinearHeading(new Pose2d(58, -47, 0), 2*Math.PI)
                        .lineToX(6)
                        // .strafeTo(new Vector2d(54, -42.5))
                        // .strafeTo(new Vector2d(54, -54))
                        // .strafeToConstantHeading(new Vector2d(4, -42.5))


                        // Hang Specimen 2
                        .strafeTo(new Vector2d(12, -42.5))
                        .turn(Math.toRadians(179))
                        .stopAndAdd(new SpecimenPickupAction(armMotor, rotaMotor, clawServo, clawRota, claw360))
                        .turn(Math.toRadians(179))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, claw360,-1220))
                        .strafeTo(new Vector2d(32, 14))
                        .stopAndAdd( new armAction( armMotor, -250))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLOSED_CLAW))
                        .waitSeconds(0.3)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        // Hang Specimen 3
                        .strafeTo(new Vector2d(12, -42.5))
                        .turn(Math.toRadians(179))
                        .stopAndAdd(new SpecimenPickupAction(armMotor, rotaMotor, clawServo, clawRota, claw360))
                        .turn(Math.toRadians(179))
                        .stopAndAdd(new RobotWakeAction(armMotor, rotaMotor, clawServo, clawRota, claw360, -820))
                        .strafeTo(new Vector2d(32, 17))
                        .stopAndAdd(new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota, claw360))

                        //Go to Observe zone
                        .strafeTo(new Vector2d(2, -48))
                        .build()
        ));

    }
}
