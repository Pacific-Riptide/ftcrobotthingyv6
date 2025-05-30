package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp(name = "Axon Servo Control via I2C (Using I2cDeviceSynch)", group = "Examples")
public class AxonServoControlWithSynch extends LinearOpMode {

    private I2cDeviceSynch axonServoI2C;  // I2C device for Axon Servo using Synch interface
    private byte[] positionCommand = new byte[1]; // Byte array to store the position command

    @Override
    public void runOpMode() {
        // Initialize the I2C device
        axonServoI2C = hardwareMap.get(I2cDeviceSynch.class, "axonServo");

        // Set the I2C address for your Axon servo
        axonServoI2C.setI2cAddress(I2cAddr.create7bit(0x40));  // Replace with your servo's actual I2C address

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop for the OpMode
        while (opModeIsActive()) {

            // Variable to store the target position for the servo
            int targetPosition = 0; // Default position (0) for the servo

            // Check for button presses
            if (gamepad1.y) {
                // When Y is pressed, set the position to 0 (start position or 0 degrees)
                targetPosition = 0;
            } else if (gamepad1.a) {
                // When A is pressed, set the position to 180 (full movement or 180 degrees)
                targetPosition = 180;
            }

            // Convert the target position to a value that the servo can use
            int servoPosition = (int) (targetPosition * (255.0 / 180.0));  // Map [0, 180] to [0, 255]

            // Create the command to send to the servo (this format is a guess; refer to the Axon servo manual)
            positionCommand[0] = (byte) servoPosition;  // Send the position command byte

            // Write the position data to the device
            axonServoI2C.write(positionCommand);  // Write the position data to the I2C device

            // Optional: Read the current position feedback (if the servo supports feedback)
            byte[] positionResponse = axonServoI2C.read(2);  // Read 2 bytes for feedback (adjust as needed)

            // Combine the response bytes to get the current servo position
            int currentPosition = positionResponse[0]; // Assuming single-byte response for position

            // Display the current position and requested position on the telemetry
            telemetry.addData("Requested Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.update();

            // Optional delay to avoid overwhelming the servo with commands
            sleep(20);  // Adjust delay based on your servo's response time
        }
    }
}
