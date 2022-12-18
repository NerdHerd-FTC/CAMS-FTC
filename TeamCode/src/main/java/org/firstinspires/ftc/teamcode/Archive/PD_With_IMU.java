/*
Copyright (c) 2022 REV Robotics, FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of REV Robotics nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This file demonstrates the impact of setting the IMU orientation correctly or incorrectly. This
 * code assumes there is an IMU configured with the name "imu".
 * <p>
 * Note: This OpMode is more of a tool than a code sample. The User Interface portion of this code
 *       goes beyond simply showing how to interface to the IMU.<br>
 *       For a minimal example of interfacing to an IMU, please see the SensorIMUOrthogonal or SensorIMUNonOrthogonal sample OpModes.
 * <p>
 * This sample enables you to re-specify the Hub Mounting orientation dynamically by using gamepad controls.
 * While doing so, the sample will display how Pitch, Roll and Yaw angles change as the hub is moved.
 * <p>
 * The gamepad controls let you change the two parameters that specify how the Control/Expansion Hub is mounted. <br>
 * The first parameter specifies which direction the printed logo on the Hub is pointing. <br>
 * The second parameter specifies which direction the USB connector on the Hub is pointing. <br>
 * All directions are relative to the robot, and left/right is as viewed from behind the robot.
 * <p>
 * How will you know if you have chosen the correct Orientation? With the correct orientation
 * parameters selected, pitch/roll/yaw should act as follows:
 * <p>
 *   Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X) <br>
 *   Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y) <br>
 *   Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z) <br>
 * <p>
 * The Yaw can be reset (to zero) by pressing the Y button on the gamepad (Triangle on a PS4 controller)
 * <p>
 * The rotational velocities should follow the change in corresponding axes.
 */

@Autonomous(name="PID With IMU", group="Concept")
@Disabled
public class PD_With_IMU extends LinearOpMode {
    //IMU Stuff
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;
            
    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;

    //Drive Stuff
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 20;   //gear ratio
    static final double     WHEEL_DIAMETER_INCH     = 3.543307;    // For figuring circumference: 90mm
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);
    // Declare OpMode members.
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    @Override public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward

        updateOrientation();

        imu.resetYaw();

            // Define and Initialize Motors and Servos
            leftDrive = hardwareMap.get(DcMotor.class, "MotorA");
            rightDrive = hardwareMap.get(DcMotor.class, "MotorB");
            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //claw.setPosition(0.0);
            //telemetry message to signify robot waiting;
            telemetry.addData(">", "Robot Ready.  Press Play.");    //
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            move(-24);
            move(24);
        }

    // apply any requested orientation changes.
    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

    void updateIMU(){

        // Display User instructions and IMU data
        telemetry.addData("logo Direction (set with bumpers)", logoFacingDirections[logoFacingDirectionPosition]);
        telemetry.addData("usb Direction (set with triggers)", usbFacingDirections[usbFacingDirectionPosition] + "\n");

        if (orientationIsValid) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        } else {
            telemetry.addData("Error", "Selected orientation on robot is invalid");
        }

        telemetry.update();
    }


    private void move(double targetInches) {

        int location = leftDrive.getCurrentPosition();
        int target = (int) (COUNTS_PER_INCH * targetInches) + location;
        double error = 0.05 * (target - location);
        final double limiter = 0.2;
        final long DELTA_T = 20;

        //K constants
        final double K_P = 10;

        while (opModeIsActive()) {
            location = leftDrive.getCurrentPosition();
            error = 0.05 * (target - location) / COUNTS_PER_INCH;

            //Set linear slide power using PID
            double finalPower = K_P * error * limiter;
            leftDrive.setPower(finalPower);
            rightDrive.setPower(finalPower);

            if (Math.abs(error) <= 0.005) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(3 * DELTA_T);
                break;
            }
            telemetry.addData("Error Number", error);
            telemetry.addData("Final Power", finalPower);
            telemetry.addData("Moving", targetInches);
            telemetry.update();

            updateIMU();
            sleep(DELTA_T);
        }
    }

    private void turn(double targetDegrees) {
        double location = leftDrive.getCurrentPosition();
        double target = (COUNTS_PER_INCH * targetDegrees * 24 / Math.PI) + location;
        double error = 0.05 * (targetDegrees - location);
        final double limiter = 0.2;
        final long DELTA_T = 20;

        //K constants
        final double K_P = 10;

        while (opModeIsActive()) {
            location = leftDrive.getCurrentPosition();
            error = 0.05 * (target - location);

            //Set linear slide power using PID
            double finalPower = K_P * error * limiter;
            leftDrive.setPower(finalPower);
            rightDrive.setPower(-finalPower);

            if (Math.abs(error) <= 0.05) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(3 * DELTA_T);
                break;
            }
            telemetry.addData("Error Number", error);
            telemetry.addData("Final Power", finalPower);
            telemetry.addData("Turning", targetDegrees);
            telemetry.update();

            updateIMU();
            sleep(DELTA_T);
        }
    }
}
