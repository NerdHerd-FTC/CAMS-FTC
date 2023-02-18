/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="+1 Right", group="Robot")
@Disabled
public class PlusOneIMU extends LinearOpMode {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor RV4BMotor1 = null;
    public DcMotor RV4BMotor2 = null;
    public Servo clawFinger = null;

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    IMU imu;

    static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    static final double WHEEL_DIAMETER_INCH = 3.65;    // For figuring circumference: 90mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

    static final double ARM_POWER = 0.65; //for quick adjustments

    static final int  HIGH_JUNCTION_TICKS = 690;
    static final int  MEDIUM_JUNCTION_TICKS = 420;
    static final int  LOW_JUNCTION_TICKS = 290;

    int coneStack = 0; //know how high to reach to get the next cone

    static final double clawOpen = 0.5;
    static final double clawClose = 0.2;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RV4BMotor1 = hardwareMap.get(DcMotor.class, "MotorC");
        RV4BMotor2 = hardwareMap.get(DcMotor.class, "MotorD");
        clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

        //core hex motors are facing opposite each other and will rotate in opposite directions
        RV4BMotor1.setDirection(DcMotor.Direction.FORWARD);
        RV4BMotor2.setDirection(DcMotor.Direction.REVERSE);

        RV4BMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RV4BMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RV4BMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RV4BMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(20);

        //initialize & setup IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[0]; //logo facing UP
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[4]; //usb ports facing to the LEFT
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        clawFinger.setPosition(clawClose);

        //wait for start
        while (!isStarted() && !isStopRequested()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        //start
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        final double startingAngle = orientation.getYaw(AngleUnit.DEGREES);

        //move forward to high junction
        forwardPID(-50.125, startingAngle);

        //turn to high junction
        turnIMU(45);

        //wait until arm is at height
        while (RV4BMotor1.isBusy() && RV4BMotor2.isBusy()) {

        }
        RV4BMotor1.setPower(0);
        RV4BMotor2.setPower(0);

        //open claw
        clawFinger.setPosition(clawOpen);

        //straighten
        turnIMU(0);

        //lower arm back to the ground
        //wait to straighten before lowering to prevent getting stuck on the junction
        armControl(0);

        //close claw
        clawFinger.setPosition(0);
    }

    private void forwardPID(double targetInches, double startingAngle) {
        int location = leftDrive.getCurrentPosition();
        final int target = (int) (COUNTS_PER_INCH * targetInches) + location; //in encoder ticks
        double error = (target - location);
        final long DELTA_T = 20 + (long) telemetry.getMsTransmissionInterval();

        //K constants
        final double K_P_MOVE = 0.0008;
        final double K_D_MOVE = 0.0105;

        final double D_MULT_MOVE = K_D_MOVE / DELTA_T;

        while (opModeIsActive()) {
            location = leftDrive.getCurrentPosition();
            double prevError = error;
            error = (target - location);
            //P
            double P = K_P_MOVE * error;
            //D
            double D = D_MULT_MOVE * (error - prevError);
            //Set power using PID
            double drivePower = P + D; //cap power at += 1

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            orientation = imu.getRobotYawPitchRollAngles();
            double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

            double turnError = startingAngle - currentAngle;

            // Normalize the error to be within +/- 180 degrees
            if (turnError > 180) {
                turnError -= 360;
            } else if (turnError < -180) {
                turnError += 360;
            }

            // Get angle error
            double correction = K_P_MOVE * turnError * 5.969; //convert degree to ticks

            double leftPower  = Math.tanh(drivePower - correction); //normalize power to between +- 1
            double rightPower = Math.tanh(drivePower + correction); //normalize power to between +- 1

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //set RV4B power to zero when motors are off - may be a redundancy
            if (!RV4BMotor1.isBusy() && !RV4BMotor2.isBusy()) {
                RV4BMotor1.setPower(0);
                RV4BMotor2.setPower(0);
            }

            telemetry.addData("Location: ", leftDrive.getCurrentPosition());
            telemetry.addData("Target: ", target);
            telemetry.addData("Error Number: ", error);
            telemetry.addData("Raw Drive Power: ", drivePower);
            telemetry.addData("Raw Turn Power: ", correction);
            telemetry.addData("Left Power: ", leftPower);
            telemetry.addData("Right Power: ", rightPower);
            telemetry.addData("Target Inch: ", targetInches);
            telemetry.addData("Starting Angle: ", startingAngle);
            telemetry.addData("Error: ", turnError);
            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();

            if (Math.abs(error) <= 15) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                telemetry.update();
                break;
            }

            sleep(DELTA_T);
        }
    }

    private void turnIMU(double degrees) {
        //K constants
        final double K_P_TURN = 0.001;
        final double K_D_TURN = 0.03;
        final long DELTA_T = 20 + (long) telemetry.getMsTransmissionInterval();

        final double D_MULT_TURN = K_D_TURN / DELTA_T;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

        double error = degrees - currentAngle;
        //get most efficient angle (imu has angles from -180 to 180)
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        while (opModeIsActive() && Math.abs(error) > 2) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);

            double prevError = error;
            error = degrees - currentAngle;

            //get most efficient angle (imu has angles from -180 to 180)
            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            double P = K_P_TURN * error * 5.969;
            double D = D_MULT_TURN * (prevError - error);

            double power = Math.tanh(P + D); //convert angle to ticks so that the P still applies
            leftDrive.setPower(-power);
            rightDrive.setPower(power);

            //set RV4B power to zero when motors are off - may be a redundancy
            if (!RV4BMotor1.isBusy() && !RV4BMotor2.isBusy()) {
                RV4BMotor1.setPower(0);
                RV4BMotor2.setPower(0);
            }

            telemetry.addLine("ROTATING");
            telemetry.addData("Target", degrees);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
            sleep(20);
        }
    }

    private void armControl(int loc) {
        RV4BMotor1.setTargetPosition(loc);
        RV4BMotor2.setTargetPosition(loc);

        RV4BMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RV4BMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RV4BMotor1.setPower(ARM_POWER);
        RV4BMotor2.setPower(ARM_POWER);
    }

}