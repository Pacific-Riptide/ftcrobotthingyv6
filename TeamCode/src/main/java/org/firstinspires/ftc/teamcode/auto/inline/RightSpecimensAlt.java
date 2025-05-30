package org.firstinspires.ftc.teamcode.auto.inline;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.actions.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.auto.actions.RobotWakeAction;
import org.firstinspires.ftc.teamcode.auto.actions.SpecimenPickupAction;
import org.firstinspires.ftc.teamcode.auto.actions.armAction;
import org.firstinspires.ftc.teamcode.auto.actions.claw360Action;
import org.firstinspires.ftc.teamcode.auto.actions.clawAction;
import org.firstinspires.ftc.teamcode.auto.actions.clawRotaAction;
import org.firstinspires.ftc.teamcode.auto.actions.rotaAction;
import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

@Autonomous(name =  "Right - 2 Specs, Observe Zone")
public class RightSpecimensAlt extends AutonomousRobot {

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

                        // Specimen Hang Routine 1
                        //Wake Robot
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, 500))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))

                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(34, 10))
                        .waitSeconds(0.3)

                        //Hang Specimen
                        .stopAndAdd( new armAction(armMotor, -250))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLOSED_CLAW))
                        .waitSeconds(0.3)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))

                        // Specimen Hang Routine 2
                        //Move to Pick up zone
                        .lineToX(20)
                        .turn(Math.toRadians(180+1e-6))
                        .strafeTo(new Vector2d(18, -42.5))
                        .waitSeconds(0.3)

                        //Pick Specimen
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_SPECIMEN_PICK))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_SPECIMEN_PICK + 400))
                        .stopAndAdd(new armAction(armMotor, RobotConstants.ARM_SPECIMEN_REACH_HEIGHT - 300))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .stopAndAdd(new armAction(armMotor, -250))

                        .turn(Math.toRadians(179))
                        .strafeToConstantHeading(new Vector2d(36, 12), new TranslationalVelConstraint(60))
                        .strafeTo(new Vector2d(38, 8))

                        //Wake Robot
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, 550))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))

                        //Hang Specimen
                        .stopAndAdd( new armAction(armMotor, -250))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLOSED_CLAW))
                        .waitSeconds(0.7)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))

                        .strafeToConstantHeading(new Vector2d(0, -48))
                        .build()
        ));
    }
}
