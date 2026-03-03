/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Solo Standard Drive", group="Linear OpMode")

public class Solo_Standard_Drive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor shootMotor = null;

    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;

    private Servo door = null;


    static final double target_RPM_close = 720;
    static final double target_RPM_far = 1100;
    static final double target_range = 40;
    private double TARGET_FLYWHEEL_RPM;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables to the config in the control hub
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fR");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "bR");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "i");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "oR");
        door = hardwareMap.get(Servo.class, "d");
        shootMotor = hardwareMap.get(DcMotor.class,"shoot");

        // setting direction for all DcMotors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE); // intake up
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setDirection(DcMotor.Direction.FORWARD); // shooter up

        //brakes
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(19,0,0,16);
        outtakeLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        outtakeRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;



            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // END OF DRIVE CODE

            // Intake Code
            if (gamepad1.left_bumper) {
                //intake down
                intakeMotor.setPower(-0.95);
                shootMotor.setPower(-0.95);
            } else if (gamepad1.right_bumper) {
                //intake up
                intakeMotor.setPower(0.95);
                shootMotor.setPower(0.95);
            }
            // shooting code
            else if (gamepad1.b){
                //intake
                intakeMotor.setPower(0.95);
            } else if (gamepad1.y) {
                //shoot down
                shootMotor.setPower(0.95);
            } else {
                shootMotor.setPower(0);
                intakeMotor.setPower(0);
            }

            //outtake code
            if (gamepad1.right_trigger > 0.5) {
                outtakeLeft.setVelocity(target_RPM_close);
                outtakeRight.setVelocity(target_RPM_close);
                TARGET_FLYWHEEL_RPM=target_RPM_close;
            } else if (gamepad1.left_trigger > 0.5) {
                outtakeLeft.setVelocity(target_RPM_far);
                outtakeRight.setVelocity(target_RPM_far);
                TARGET_FLYWHEEL_RPM=target_RPM_far;
            }
            else {
                outtakeLeft.setPower(0);
                outtakeRight.setPower(0);
            }


            if (gamepad1.x) {
                // door up
                door.setPosition(0.60);
            } else if (gamepad1.a) {
                //door down
                door.setPosition(0.30);
            }

            /*if ((outtakeRight.getVelocity() >= TARGET_FLYWHEEL_RPM-target_range &&
                    outtakeRight.getVelocity() <=  TARGET_FLYWHEEL_RPM+target_range) &&
                    (outtakeLeft.getVelocity() >= TARGET_FLYWHEEL_RPM-target_range &&
                            outtakeLeft.getVelocity() <= TARGET_FLYWHEEL_RPM+target_range)) {
                gamepad2.rumble(5);
            }*/

            // Show the elapsed game time and wheel power.
            telemetry.addData("Velocity: ", outtakeLeft.getVelocity());
            telemetry.addData("intake motor power", intakeMotor.getPower());
            telemetry.addData("shoot motor power", shootMotor.getPower());
            telemetry.addData("door position",door.getPosition());
            telemetry.update();

        }
    }
}
