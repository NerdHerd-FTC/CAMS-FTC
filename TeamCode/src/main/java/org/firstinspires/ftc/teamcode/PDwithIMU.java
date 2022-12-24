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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="PD with IMU", group="Robot")
@Disabled
public class PDwithIMU extends LinearOpMode
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 20;   //gear ratio
    static final double     WHEEL_DIAMETER_INCH     = 3.8;    // For figuring circumference: 90mm
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

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

        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            forward(24);
        }
    }

    private void forward(double targetInches) {

        int location = leftDrive.getCurrentPosition();
        int target = (int) (COUNTS_PER_INCH * targetInches) + location; //in encoder ticks
        double error = (target - location);
        final long DELTA_T = 20;

        //K constants
        final double K_P_MOVE = 0.01;
        final double K_D_MOVE = 1;

        final double D_MULT_MOVE = K_D_MOVE/DELTA_T;
        while (opModeIsActive()) {
            location = leftDrive.getCurrentPosition();
            double prevError = error;
            error = (target - location) / COUNTS_PER_INCH;
            //P
            double P = K_P_MOVE * error;
            //D
            double D = D_MULT_MOVE * (error - prevError);
            //Set power using PID
            double finalPower = (P + D);
            leftDrive.setPower(finalPower);
            rightDrive.setPower(finalPower);

            if (Math.abs(error) <= 0.01) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(3 * DELTA_T);
                break;
            }
            telemetry.addData("Error Number", error);
            telemetry.addData("Final Power", finalPower);
            telemetry.addData("Moving", targetInches);
            telemetry.update();
            sleep(DELTA_T);
        }
    }
}