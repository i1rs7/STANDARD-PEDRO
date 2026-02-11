package org.firstinspires.ftc.teamcode.limelight;//package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Import the official goBILDA Pinpoint driver
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "SampleAutoAlign", group = "Linear Opmode")
public class SampleAutoAlign extends LinearOpMode {

    // --- Hardware Definitions ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pinpoint; // The Odometry Computer

    // --- Constants ---
    // PID Controller for turning.
    // If it overshoots: decrease P. If it's too slow: increase P.
    private static final double TURN_P = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. Initialize Motors
        frontLeft = hardwareMap.get(DcMotor.class, "fL");
        frontRight = hardwareMap.get(DcMotor.class, "fR");
        backLeft = hardwareMap.get(DcMotor.class, "bL");
        backRight = hardwareMap.get(DcMotor.class, "bR");

        // Reverse left side motors (standard for most drivetrains, check yours!)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // 2. Initialize Pinpoint Computer
        // Make sure the config name matches your Driver Station ("pinpoint")
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // OFFSETS: Distance from center of robot to the pod in IN
        pinpoint.setOffsets(0, 1.0, DistanceUnit.INCH);

        // Reset the Pinpoint so it knows "Forward" is 0 degrees right now
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Ready! Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // CRITICAL: You must update the Pinpoint every loop to get new data
            pinpoint.update();

            // 3. Get Inputs
            double y = -gamepad1.left_stick_y; // Forward/Back (inverted)
            double x = gamepad1.left_stick_x;  // Strafe Left/Right
            double rx = gamepad1.right_stick_x; // Turn

            // 4. Get Heading from Pinpoint
            // We use Radians because Math.sin/cos expect them
            Pose2D pose = pinpoint.getPosition();
            double currentHeading = pose.getHeading(AngleUnit.RADIANS);

            // 5. Auto-Align Logic
            // If 'A' is held, we override the manual turn (rx) with calculated turn
            if (gamepad1.a) {
                double targetHeading = Math.toRadians(0); // Face Forward

                // Calculate the difference (Error)
                double error = AngleUnit.normalizeRadians(targetHeading - currentHeading);

                // Simple P-Controller: Power = Error * P_Constant
                // We clip it to ensure it doesn't spin too fast (safety)
                rx = Range.clip(error * TURN_P, -0.6, 0.6);
            }

            // 6. Field Centric Math
            // Rotates the joystick input by the negative robot heading
            double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
            double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

            // Denominator ensures we don't ask motors for > 100% power
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // 7. Set Power
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry for Debugging
            telemetry.addData("Heading (Deg)", Math.toDegrees(currentHeading));
            telemetry.addData("Target (Deg)", gamepad1.a ? "0.0" : "Manual");
            telemetry.addData("Turn Power", rx);
            telemetry.update();
        }
    }
}