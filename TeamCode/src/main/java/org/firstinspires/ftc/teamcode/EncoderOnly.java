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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Encoder Only", group="Robot")
public class EncoderOnly extends LinearOpMode
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor Arm = null;
    public Servo clawFinger = null;

    //drivebase
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 20;   //gear ratio
    static final double     WHEEL_DIAMETER_INCH     = 3.8;    // For figuring circumference: 90mm
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

    //arm
    static final int  TICKS_TO_REACH = 770;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");

        Arm = hardwareMap.get(DcMotor.class, "MotorC");

        clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

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

        Arm.setDirection(DcMotor.Direction.FORWARD);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(50);

        clawFinger.setPosition(0.2); //close claw

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            encoderDrive(0.5, 54, 54); //move forward to high junction
            moveArm(true);
            encoderDrive(0.5, -6.5, 6.5); //turn 45 degrees to the left
            clawFinger.setPosition(0.5); //open
            encoderDrive(0.5, 6.5, -6.5); //turn back 45 degrees
            moveArm(false); //drop arm
            encoderDrive(0.5, -10, -10); //move back 10 inches

            //close loose-ends
            while (Arm.isBusy()) {

            }
            Arm.setPower(0);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //stop arm motion if opMode is stopped
        Arm.setPower(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            if (!Arm.isBusy()) {
                Arm.setPower(0);
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (leftDrive.isBusy() && rightDrive.isBusy())) {

                    if (!Arm.isBusy()) {
                        Arm.setPower(0);
                        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    // Display it for the driver.
                    telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Currently at", " at %7d :%7d",
                            leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                    telemetry.addData("Arm Encoder: ", Arm.getCurrentPosition());
                    telemetry.addData("Arm TARGET: %7d", Arm.getTargetPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);   // optional pause after each move.
            }
        }

    public void moveArm(boolean lift) {
        if (opModeIsActive()) {
            if (lift) {
                Arm.setTargetPosition(TICKS_TO_REACH);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.65);
            } else {
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.65);
            }
        }
    }
}