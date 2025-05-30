package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public final class MecanumTeleOp extends LinearOpMode {

    // Declare motors and servos
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private DcMotor hangMotor;
    private Servo actuator;
    private Servo wrist;
    private Servo gripper;

    private static final double HOLD_POWER = 0.0;   // Power to hold position
    private static final double UP_POWER = -1;     // Full power for upward movement
    private static final double DOWN_POWER = 0.3;  // Limited power for controlled descent

    private static final double HANG_POWER = 0.0;   // Power to hold position
    private static final double ASCENT_POWER = -1;     // Full power for upward movement
    private static final double DESCENT_POWER = 0.9;  // Limited power for controlled descent

    @Override

    public void runOpMode() {
        // Initialize hardware
        frontLeftMotor = hardwareMap.dcMotor.get("TopLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("BottomLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("TopRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("BottomRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        hangMotor = hardwareMap.dcMotor.get("hangMotor");
        actuator = hardwareMap.servo.get("actuatorServo");
        wrist = hardwareMap.servo.get("wristServo");
        gripper = hardwareMap.servo.get("gripServo");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initial positions for each servo
        double actuatorPosition = 0.32;
        double wristPosition = 0.5;

        double gripperPosition = 0.5;

        // Wait for the start button to be pressed
        waitForStart();
        actuator.setDirection(Servo.Direction.REVERSE);
        actuator.setPosition(actuatorPosition);
        wrist.setPosition(wristPosition);
        gripper.setPosition(gripperPosition);

        while (opModeIsActive()) {
            // Get joystick values
            double drive = -gamepad1.left_stick_y; // Forward/Backward
            double strafe = gamepad1.left_stick_x; // Left/Right
            double turn = gamepad1.right_stick_x;  // Turning

            // Calculate power for each motor
            double frontLeftPower = drive + strafe + turn;
            double backLeftPower = drive - strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backRightPower = drive + strafe - turn;

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
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Check for button inputs
            boolean downPressed = gamepad1.left_bumper && !gamepad1.b;  // Use left bumper for down
            boolean upPressed = gamepad1.right_bumper && !gamepad1.b; // Use right bumper for up

            if (upPressed) {
                // Move arm up
                armMotor.setPower(UP_POWER);
            } else if (downPressed) {
                // Move arm down
                armMotor.setPower(DOWN_POWER);
            } else {
                // Hold position
                armMotor.setPower(HOLD_POWER);
            }

            // Send telemetry to driver station
            telemetry.addData("Arm Power", armMotor.getPower());
            telemetry.update();


            // Check for button inputs
            boolean descPressed = gamepad1.b && gamepad1.left_bumper;  // Use left bumper for down
            boolean ascPressed = gamepad1.b && gamepad1.right_bumper;// Use right bumper for up

            if (ascPressed) {
                // Move hanger up
                hangMotor.setPower(ASCENT_POWER);
            } else if (descPressed) {
                // Move hanger down
                hangMotor.setPower(DESCENT_POWER);
            } else {
                // Hang position
                hangMotor.setPower(HANG_POWER);
            }

            // Send telemetry to driver station
            telemetry.addData("Hanging Power", hangMotor.getPower());
            telemetry.update();

            double min_actuator_position = 0.32;
            double max_actuator_position = 0.7;


            // Control the actuator with D-pad up/down
            if (gamepad1.dpad_up) {
                actuatorPosition += 0.003;
            } else if (gamepad1.dpad_down) {
                actuatorPosition -= 0.001;
            }

            double min_wrist_position = 0.01;
            double max_wrist_position = 0.95;
            // Control the wrist with bumpers
            if (gamepad1.dpad_left) {
                wristPosition += 0.003; // Move left
            } else if (gamepad1.dpad_right) {
                wristPosition -= 0.003; // Move right
            }
                // Control the gripper with triggers
                if (gamepad1.left_trigger > 0.5) {
                    gripperPosition = 0.0; // Close gripper
                } else if (gamepad1.right_trigger > 0.5) {
                    gripperPosition = 1.0; // Open gripper
                } else {
                    gripperPosition = 0.5; //Stop gripper
                }

                // Clamp the servo positions to stay within range [0.0, 1.0]
                actuatorPosition = Math.max(min_actuator_position, Math.min(actuatorPosition, max_actuator_position));
                wristPosition = Math.max(min_wrist_position, Math.min(wristPosition, max_wrist_position));
                gripperPosition = Math.max(0.0, Math.min(gripperPosition, 1.0));

                // Update servos with new positions
                actuator.setPosition(actuatorPosition);
                wrist.setPosition(wristPosition);
                gripper.setPosition(gripperPosition);

                // Telemetry for debugging
                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);
                telemetry.addData("Actuator Position", actuatorPosition);
                telemetry.addData("Wrist Position", wristPosition);
                telemetry.addData("Gripper Position", gripperPosition);

                telemetry.addData("Drive", drive);
                telemetry.addData("Strafe", strafe);
                telemetry.addData("Turn", turn);
                telemetry.update();
            }
        }
    }
