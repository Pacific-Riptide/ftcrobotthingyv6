package org.firstinspires.ftc.teamcode.auto.official;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.actions.ClawLowBarTouchAction;
import org.firstinspires.ftc.teamcode.auto.actions.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.auto.actions.RobotWakeAction;
import org.firstinspires.ftc.teamcode.auto.actions.SamplePickupAction;
import org.firstinspires.ftc.teamcode.auto.actions.ScoreSampleBackAction;
import org.firstinspires.ftc.teamcode.auto.actions.armAction;
import org.firstinspires.ftc.teamcode.auto.actions.clawAction;
import org.firstinspires.ftc.teamcode.auto.actions.clawRotaAction;
import org.firstinspires.ftc.teamcode.auto.actions.rotaAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

@Autonomous(name = "Left - 3 Samples (1 from Start) - Official", group = "Official")
public class sampleBuckets extends AutonomousRobot {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        super.initializeRobot();
        super.setRobotBehavior();
        super.startRobot();
        clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);

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


                        .strafeToLinearHeading(new Vector2d(5, 57), Math.toRadians(-45), new TranslationalVelConstraint(150))        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .stopAndAdd( new armAction(armMotor, RobotConstants.ARM_TO_SAMPLE_BASKET_HEIGHT))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_BACK+25))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))
                        .waitSeconds(1)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        .waitSeconds(0.1)

                        //Sample 2
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_PERPENDICULAR))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT + 150))
                        .stopAndAdd(new armAction(armMotor, RobotConstants.ARM_DOWN))
                        .strafeToLinearHeading(new Vector2d(28.5, 47), Math.toRadians(0), new TranslationalVelConstraint(120))
                        .waitSeconds((0.0))
                        .stopAndAdd(new armAction(armMotor, -150))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_DOWN))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .waitSeconds((0.0))
                        .strafeToLinearHeading(new Vector2d(5, 58), Math.toRadians(-45), new TranslationalVelConstraint(120))
                        .waitSeconds((0.0))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .stopAndAdd( new armAction(armMotor, RobotConstants.ARM_TO_SAMPLE_BASKET_HEIGHT))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_BACK+100))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))
                        .waitSeconds(0.5)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        .waitSeconds(0.1)

                        //Sample 3
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_PERPENDICULAR))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT + 150))
                        .stopAndAdd(new armAction(armMotor, RobotConstants.ARM_DOWN))
                        .strafeToLinearHeading(new Vector2d(27, 59), Math.toRadians(0), new TranslationalVelConstraint(80))
                        .waitSeconds((0.0))
                        .stopAndAdd(new armAction(armMotor, -150))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_DOWN))
                        .waitSeconds(0.1)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .waitSeconds((0.0))
                        .strafeToLinearHeading(new Vector2d(5, 59), Math.toRadians(-45), new TranslationalVelConstraint(80))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .stopAndAdd( new armAction(armMotor, RobotConstants.ARM_TO_SAMPLE_BASKET_HEIGHT))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_BACK+150))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))
                        .waitSeconds(0.8)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        .waitSeconds(0.1)

                        //Attempt to go to zone
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_PERPENDICULAR+0.26))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT + 800))
                        .stopAndAdd(new armAction(armMotor, RobotConstants.ARM_DOWN - 900))
                        .setTangent(2*Math.PI)


                        .splineToLinearHeading(new Pose2d(60, 12, 105),Math.toRadians(200), new TranslationalVelConstraint(110))
                        .build());
    }
}