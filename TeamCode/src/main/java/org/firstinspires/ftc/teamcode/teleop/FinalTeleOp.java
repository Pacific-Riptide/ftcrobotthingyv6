package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.actions.TeleOpScoreSampleBackAction;
import org.firstinspires.ftc.teamcode.auto.actions.TeleOpSpecimenPickupAction;
import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

@TeleOp
public final class FinalTeleOp extends Robot {

    private static final double HOLD_POWER = 0.0;   // Power to hold position
    private static final double UP_POWER = -1.8;     // Full power for upward movement
    private static final double DOWN_POWER = 1.8;  // Limited power for controlled descent

    private static final double DEROTA_POWER = 0.0;   // Power to hold position
    private static final double AROTA_POWER = -1;// Full power for ascension
    private static final double OROTA_POWER = 1;// Full power for declension
    //private static final double TICK_LIMIT_UP = -50;
    //private static final double TICK_LIMIT_DOWN = 950;
    private static final double LEFT_HOOK = 1;
    private static final double RIGHT_HOOK = 1;


    private static final double clawMinPosition = 0.0;
    private static final double clawMaxPosition = 1.0;

    private static final double clawRotatorMinPosition = 0;
    private static final double clawRotatorMaxPosition = 1;
    //Start
    @Override
    public void runOpMode() throws InterruptedException {
        super.initializeRobot();
        super.setRobotBehavior();
        waitForStart();

        while (opModeIsActive()) {
            // Operate drive train
            operateDriveTrain();

            //Operate Viper Slide Up and Down
            operateViperSlide();

            operateHangerMechanism();


            operateClaw(clawMinPosition, clawMaxPosition);

            operateClawRotator(clawRotatorMinPosition, clawRotatorMaxPosition);

            checkForPresetButtons();

            super.displayTelemetry();
        }
    }

    private void operateClawRotator(Double minPosition, Double maxPosition) {
        double clawRotaPosition = clawRota.getPosition();
        if (gamepad1.dpad_up||gamepad2.dpad_up) {
            clawRotaPosition += 0.1;
        } else if (gamepad1.dpad_down||gamepad2.dpad_down) {
            clawRotaPosition -= 0.1;
        }

        clawRotaPosition = Math.max(minPosition, Math.min(clawRotaPosition, maxPosition));
        clawRota.setPosition(clawRotaPosition);
    }

    private void operateClaw(Double minPosition, Double maxPosition) {
        double clawPosition = clawServo.getPosition();
        double claw360Position = claw360.getPosition();

        if (gamepad1.left_trigger > 0.01||gamepad2.left_trigger > 0.01) {
            clawPosition += 0.5;
        } else if (gamepad1.right_trigger > 0.01||gamepad2.right_trigger > 0.01) {
            clawPosition -= 0.5;
        }
        clawPosition = Math.max(minPosition, Math.min(clawPosition, maxPosition));
        clawServo.setPosition(clawPosition);

        if (gamepad1.dpad_left||gamepad2.dpad_left) {
            claw360Position += 0.1;
        } else if (gamepad1.dpad_right||gamepad2.dpad_right) {
            claw360Position -= 0.1;
        }
        claw360.setPosition(claw360Position);
    }

    private void operateViperSlide() throws InterruptedException {
        // Check for button inputs
        boolean downPressed = gamepad1.left_bumper||gamepad2.left_bumper;
        boolean upPressed = gamepad1.right_bumper||gamepad2.right_bumper;

       // double safeDistance = magneticSensor.getDistance(DistanceUnit.MM);
        if (upPressed) {
            // Move arm up
            moveDcMotor(armMotor, UP_POWER);
            telemetry.addData("Arm Up Pressed", 1);
        } else if (downPressed) {
            // Move arm down
            moveDcMotor(armMotor, DOWN_POWER);
            telemetry.addData("Arm Down Pressed", 2);
        } else {
            // Hold position
            moveDcMotor(armMotor, HOLD_POWER);
            telemetry.addData("Arm Pressed", 0);
        }

        // Check for button inputs
        boolean derotaPressed = gamepad1.b||gamepad2.left_stick_button;  // Use left bumper for down
        boolean arotaPressed = gamepad1.y||gamepad2.right_stick_button ; // Use right bumper for up


        if (arotaPressed && rotaMotor.getCurrentPosition() > RobotConstants.ARM_ROTATION_BACK - 500) {
            // Rotate up
            moveDcMotor(rotaMotor, AROTA_POWER);
        } else if (derotaPressed && !touchSensor.isPressed() && rotaMotor.getCurrentPosition() < RobotConstants.ARM_ROTATION_DOWN + 100) {
            // Rotate down
            moveDcMotor(rotaMotor, OROTA_POWER);
        } else {
            // Hang position
            moveDcMotor(rotaMotor, DEROTA_POWER);
        }

    }

    private void operateHangerMechanism()
    {
        if (gamepad2.back)
        {
            if (leftHook.getPosition() == 0)
            {
                leftHook.setPosition(1);
                rightHook.setPosition(1);
            }
            else if (leftHook.getPosition() == 1)
            {
                leftHook.setPosition(0);
                rightHook.setPosition(0);
            }
        }
    }

    private void moveDcMotor(DcMotor motor, double power) {
        motor.setPower(power);
    }

    private void operateDriveTrain() {
            // Robot-centric driving
        double drive = -gamepad1.left_stick_y; // Forward/Backward
        double strafe = gamepad1.left_stick_x;  // Left/Right
        double turn = gamepad1.right_stick_x; // Turning

            // Calculate power for each motor
            double frontLeftPower = (drive + strafe + turn);
            double backLeftPower = (drive - strafe + turn);
            double frontRightPower = (drive - strafe - turn);
            double backRightPower = (drive + strafe - turn);

            // Normalize motor powers if any power is above 1.0
            double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;

            // Set power to motors
            moveDcMotor(frontLeftMotor, frontLeftPower);
            moveDcMotor(backLeftMotor, backLeftPower);
            moveDcMotor(frontRightMotor, frontRightPower);
            moveDcMotor(backRightMotor, backRightPower);

    }

    // Method to check for preset button presses

    private void checkForPresetButtons() throws InterruptedException {
        // To go into collection mode, press A
        if (gamepad1.a) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_SPECIMEN_PICK - 0.09);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_DOWN - 50, false);
            clawServo.setPosition(RobotConstants.OPEN_CLAW);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_STRAIGHT_SAMPLE_PICK_UP_HEIGHT, true);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_DOWN - 250, false);
        }
        // To go back to the starting position, press B, RESET
        else if (gamepad1.x) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            DCMotorUtils.controlMotorWithPID(armMotor, -50,  true);
            new TeleOpSpecimenPickupAction(armMotor,rotaMotor, clawServo, clawRota, claw360).run(new TelemetryPacket());
        }
        // To score a specimen in the high chamber, press Y
        else if (gamepad2.a) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            DCMotorUtils.controlMotorWithPID(armMotor, -50, true);
            clawRota.setPosition(1);
            DCMotorUtils.controlMotorWithPID(rotaMotor, 200, false);
            // new HangSpecimenAction(armMotor, rotaMotor, clawServo, clawRota, claw360).run(new TelemetryPacket());
        }

        // To go back to the starting position, press B, RESET
        else if (gamepad2.b) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_DOWN);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_INCOMING, true);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_GAME_START_POSITION, false);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_UP);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_DOWN, true);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT, false);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
            clawServo.setPosition(0);

        }
        //Arm motor go up and down
        else if (gamepad2.x)
        {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_TO_MAX_HEIGHT, true);
            DCMotorUtils.controlMotorWithPID(rotaMotor, 500, false);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_DOWN-100, true);

        }


        // To score a sample into high basket, press Y
        else if (gamepad2.y) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            new TeleOpScoreSampleBackAction(armMotor, rotaMotor, clawServo, clawRota, claw360).run(new TelemetryPacket());
        }


    }
}

