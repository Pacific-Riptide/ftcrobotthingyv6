package org.firstinspires.ftc.teamcode.auto.official;

import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

@Autonomous(name =  "Right - 3 Specs, Observe Zone - Official", group = "Official")
public class specimenChambers extends AutonomousRobot {

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
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, 500))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))
                        .setTangent(Math.PI/2)
                        .strafeToConstantHeading(new Vector2d(35, 25), new TranslationalVelConstraint(150))
                        .stopAndAdd( new armAction(armMotor, -200))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLOSED_CLAW))
                        .waitSeconds(0.3)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        //strafes to right
                        .setReversed(true)
                        .strafeToConstantHeading(new Vector2d(18, -20), new TranslationalVelConstraint(180.0))
                        //moves forward
                        .strafeToConstantHeading(new Vector2d(53, -17), new TranslationalVelConstraint(140))
                        .splineToConstantHeading(new Vector2d(53, -31), 2 * Math.PI, new TranslationalVelConstraint(180))
                        //comes back
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(20, -27), 0, new TranslationalVelConstraint(150))
                        //moves forward
                       /* .setReversed(false)
                        .splineToConstantHeading(new Vector2d(60, -45), -Math.PI/2)
                        //moves backwards
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(20, -37), 0)*/

//
//

//
                        //FIRST PICKUP
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(19, -28, 0), 0, new TranslationalVelConstraint(1600))
                        .turn(Math.PI)
                        .waitSeconds(0.05)
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_SPECIMEN_PICK))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_SPECIMEN_PICK + 400))
                        .stopAndAdd(new armAction(armMotor, RobotConstants.ARM_SPECIMEN_REACH_HEIGHT - 500))
                        .waitSeconds(0.1)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .stopAndAdd(new armAction(armMotor, -250))

//                        //2nd PLACER
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, 500))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))
                        .setTangent(2*Math.PI)
                        .splineToLinearHeading(new Pose2d(39, 31, Math.toRadians(0)),Math.PI/3, new TranslationalVelConstraint(100))
                        .strafeTo(new Vector2d(44,20))

                        .stopAndAdd( new armAction(armMotor, -200))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLOSED_CLAW))
                        .waitSeconds(0.3)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))

//                        //Second PICKER
                        .strafeToLinearHeading(new Vector2d(28, -25), Math.PI, new TranslationalVelConstraint(100))
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_SPECIMEN_PICK))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_SPECIMEN_PICK + 400))
                        .stopAndAdd(new armAction(armMotor, RobotConstants.ARM_SPECIMEN_REACH_HEIGHT - 200))
                        .waitSeconds(0.1)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT))
                        .stopAndAdd(new armAction(armMotor, -250))
//                        .waitSeconds(0.05)
//
//                        //3rd placer
                        .setTangent(2*Math.PI)
                        .splineToLinearHeading(new Pose2d(40, 30, Math.toRadians(0)),Math.PI/3, new TranslationalVelConstraint(100))
                        .strafeTo(new Vector2d(50, 42))
                        .stopAndAdd(new claw360Action(claw360, RobotConstants.CLAW_STRAIGHT))
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.CLOSED_CLAW))
                        .stopAndAdd(new rotaAction(rotaMotor, 500))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLAW_ROTA_UP))
                        .stopAndAdd( new armAction( armMotor, -250))
                        .stopAndAdd(new clawRotaAction(clawRota, RobotConstants.CLOSED_CLAW))
                        .waitSeconds(0.3)
                        .stopAndAdd(new clawAction(clawServo, RobotConstants.OPEN_CLAW))

                      // This might not happen during auto but whatever
                        .setTangent(3*Math.PI/2.5)//
                        .splineToConstantHeading(new Vector2d(5,-35),3*Math.PI/2.5, new TranslationalVelConstraint(110.0))

                        .build()
        );

    }
}
