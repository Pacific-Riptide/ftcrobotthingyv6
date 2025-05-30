package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

@TeleOp
public final class MecanumDirectionDebugger extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private DcMotor rotaMotor;
    private Servo clawServo;
    private Servo clawRota;
    private TouchSensor touchSensor;

    private static final double HOLD_POWER = 0.0;   // Power to hold position
    private static final double UP_POWER = -1;     // Full power for upward movement
    private static final double DOWN_POWER = 1;  // Limited power for controlled descent

    private static final double DEROTA_POWER = 0.0;   // Power to hold position
    private static final double AROTA_POWER = -1;// Full power for ascension
    private static final double OROTA_POWER = 0.2;// Full power for declension
    private static final double TICK_LIMIT_UP = -50;
    private static final double TICK_LIMIT_DOWN = 950;

    private static final double clawMinPosition = 0.0;
    private static final double clawMaxPosition = 1.0;

    private static final double clawRotatorMinPosition = 0;
    private static final double clawRotatorMaxPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardwareMap();
        setRobotBehavior();
        waitForStart();

        while (opModeIsActive()) {
            // Operate drive train
            if (gamepad1.x)
            {
                frontLeftMotor.setPower(1);
            }
            if (gamepad1.y)
            {
                frontRightMotor.setPower(1);
            }
            if (gamepad1.a)
            {
                backLeftMotor.setPower(1);
            }
            if (gamepad1.b)
            {
                backRightMotor.setPower(1);
            }
        }
    }

    private void operateClawRotator(Double minPosition, Double maxPosition) {
        double clawRotaPosition = clawRota.getPosition();
        if(gamepad1.dpad_left){
            clawRotaPosition += 0.003;
        } else if (gamepad1.dpad_right) {
            clawRotaPosition -= 0.003;
        }

        clawRotaPosition = Math.max(minPosition, Math.min(clawRotaPosition, maxPosition));
        clawRota.setPosition(clawRotaPosition);
    }

    private void operateClaw(Double minPosition, Double maxPosition) {
        double clawPosition = clawServo.getPosition();

        if(gamepad1.left_trigger > 0.01){
            clawPosition += 0.009;
        } else if (gamepad1.right_trigger > 0.01) {
            clawPosition -= 0.009;
        }
        clawPosition = Math.max(minPosition, Math.min(clawPosition, maxPosition));
        clawServo.setPosition(clawPosition);
    }

    private void operateViperSlide() {
        // Check for button inputs
        boolean downPressed = gamepad1.left_bumper;  // Use left bumper for down
        boolean upPressed = gamepad1.right_bumper; // Use right bumper for up

        if (upPressed) {
            // Move arm up
            moveDcMotor(armMotor, UP_POWER);
        } else if (downPressed) {
            // Move arm down
            moveDcMotor(armMotor, DOWN_POWER);
        } else {
            // Hold position
            moveDcMotor(armMotor, HOLD_POWER);
        }

        // Check for button inputs
        boolean derotaPressed = gamepad1.dpad_down;  // Use left bumper for down
        boolean arotaPressed = gamepad1.dpad_up;// Use right bumper for up



        if (arotaPressed && rotaMotor.getCurrentPosition() > TICK_LIMIT_UP) {
            // Rotate up
            moveDcMotor(rotaMotor, AROTA_POWER);
        } else if (derotaPressed && !touchSensor.isPressed() && rotaMotor.getCurrentPosition() < TICK_LIMIT_DOWN) {
            // Rotate down
            moveDcMotor(rotaMotor, OROTA_POWER);
        } else {
            // Hang position
            moveDcMotor(rotaMotor, DEROTA_POWER);
        }
    }

    private void moveDcMotor(DcMotor motor, double power)
    {
        motor.setPower(power);
    }

    private void operateDriveTrain() {
        double drive = -gamepad1.left_stick_y; // Forward/Backward
        double strafe = gamepad1.left_stick_x; // Left/Right
        double turn = gamepad1.right_stick_x;  // Turning

        // Calculate power for each motor
        double frontLeftPower = drive + strafe + turn;
        double backLeftPower = drive - strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backRightPower = drive + strafe - turn;

        // Normalize motor powers if any power is above 1.0
        double maxPower = Math.max(0.25, Math.abs(frontLeftPower));
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
            clawServo.setPosition(1);
            controlMotorWithPID(rotaMotor, 700);
            Thread.sleep(1000);
            controlMotorWithPID(armMotor, -1400);
            controlMotorWithPID(rotaMotor, 650);
            clawRota.setPosition(0.77);
        }
        // To go back to the starting position, press B, RESET
        else if (gamepad1.b && !gamepad1.y) {
            controlMotorWithPID(armMotor, -700);
            controlMotorWithPID(rotaMotor, 650);
            clawRota.setPosition(0.52);
            controlMotorWithPID(armMotor, 0);
            controlMotorWithPID(rotaMotor, 0);
            Thread.sleep(1000);
            clawRota.setPosition(0.18);
            clawServo.setPosition(0);
        }
        // To ready the robot to hang, press X
        else if (gamepad1.x) {
            setPresetPosition(0, -750, 0, 0.18);
        }
        // To score a sample into high basket, press Y
        else if (gamepad1.y && !gamepad1.b) {
            controlMotorWithPID(armMotor, -2100);
            controlMotorWithPID(rotaMotor, 70);
            Thread.sleep(500);

            clawRota.setPosition(0.18);
            clawServo.setPosition(1);
            Thread.sleep(500);
            controlMotorWithPID(armMotor, -2100);

            controlMotorWithPID(rotaMotor, 0);
        }
        // To get into the position to score a specimen onto the high rung, press B and Y at the same time
        else if (gamepad1.b && gamepad1.y) {
            setPresetPosition(0 ,0, 0.0, 0.52);
        }
    }

    // Method to set motors and servos to preset positions
    private void setPresetPosition(int armPosition,  int rotaPosition, double clawPosition, double clawRotaPosition) throws InterruptedException {

        controlMotorWithPID(armMotor, armPosition);
        controlMotorWithPID(rotaMotor, rotaPosition);
        Thread.sleep(1000);


        // Set servo positions
        clawServo.setPosition(clawPosition);
        clawRota.setPosition(clawRotaPosition);

    }

    private void setRobotBehavior() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotaMotor.setDirection(DcMotor.Direction.REVERSE);
        rotaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawRota.setDirection(Servo.Direction.REVERSE);
    }

    private void initializeHardwareMap() {
        // Declare motors and servos
        frontLeftMotor = hardwareMap.dcMotor.get("TopLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("BottomLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("TopRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("BottomRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        rotaMotor = hardwareMap.dcMotor.get("rotaMotor");
        clawServo = hardwareMap.servo.get("ClawServo");
        clawRota = hardwareMap.servo.get("ClawRota");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
    }

    public void controlMotorWithPID(DcMotor motor, int targetPosition) {
        int currentPosition = motor.getCurrentPosition();  // Get current position
        double error = 0;
        double integral = 0;
        double derivative = 0;
        double previousError = 0;

        int threshold = 10;

        // kP: Determines how aggressively the motor responds to errors.
        // kI: Helps reduce steady-state error by accumulating past errors.
        // kD: Smooths the motor’s response by dampening rapid changes in error.

        double kP = 1.0;
        double kI = 0.1;
        double kD = 0.01;

        while (Math.abs(currentPosition - targetPosition) > threshold) {
            error = targetPosition - currentPosition;
            integral += error;
            derivative = error - previousError;
            double output = (kP * error) + (kI * integral) + (kD * derivative);
            previousError = error;

            double adjustedOutput = Math.max(-1.0, Math.min(output, 1.0));
            motor.setPower( adjustedOutput);  // Move forward

            //Motor is Rota Motor and Direction is going down
            if(motor.getDeviceName().equals("rotaMotor") && adjustedOutput > 0){
                motor.setPower(motor.getPower()/2);
            }
            currentPosition = motor.getCurrentPosition();  // Update position*/


            // Debugging output
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Motor Run Mode", motor.getMode());
            telemetry.addData("Motor Type", motor.getMotorType());
            telemetry.addData("Target Position ", motor.getTargetPosition());
            telemetry.addData("Motor Power ", motor.getPower());
            telemetry.addData("Kp ", kP);
            telemetry.addData("Ki ", kI);
            telemetry.addData("Kd ", kD);
            //telemetry.addData("Output ", /*output*/);
            telemetry.update();
        }
        motor.setPower(0);
    }
}
