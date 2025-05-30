package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoFunctions {

    public static void moveForward(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power to all motors
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void moveBackward(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power to all motors
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void turnInPlaceClockwise(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set opposite power to left and right motors for rotation
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void turnInPlaceCounterClockwise(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set opposite power to left and right motors for rotation
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void moveDiagonallyFrontLeft(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power to diagonal motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(0);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void moveDiagonallyFrontRight(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power to diagonal motors
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void moveDiagonallyBackLeft(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power to diagonal motors
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(-power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void moveDiagonallyBackRight(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power to diagonal motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(0);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void strafeLeft(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power for strafing left
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(-power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void strafeRight(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, double power, long duration) throws InterruptedException {
        // Set power for strafing right
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(power);
        Thread.sleep(duration);
        stopMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    public static void stopMotors(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor) {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
