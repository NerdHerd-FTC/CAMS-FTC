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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;

@Autonomous(name="+1 Right (Encoders)", group="Robot")
public class PlusOneEncoders extends LinearOpMode {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor DR4BMotor1 = null;
    public DcMotor DR4BMotor2 = null;
    public Servo clawFinger = null;

    static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    static final double WHEEL_DIAMETER_INCH = 3.65;    // For figuring circumference: 90mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

    static final double ARM_POWER = 0.65; //for quick adjustments

    static final int HIGH_JUNCTION_TICKS = 690;
    static final int MEDIUM_JUNCTION_TICKS = 420;
    static final int LOW_JUNCTION_TICKS = 290;

    int coneStack = 0; //know how high to reach to get the next cone

    static final double clawOpen = 0.5;
    static final double clawClose = 0.1;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166; //meters

    //above are used for trajectory, not identification; not really useful for us

    // Tag ID 121, 122, 123 from the 36h11 family
    int LEFT = 121;
    int MIDDLE = 122;
    int RIGHT = 123;

    AprilTagDetection tagOfInterest = null;

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

        DR4BMotor1 = hardwareMap.get(DcMotor.class, "MotorC");
        DR4BMotor2 = hardwareMap.get(DcMotor.class, "MotorD");
        clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

        //core hex motors are facing opposite each other and will rotate in opposite directions
        DR4BMotor1.setDirection(DcMotor.Direction.FORWARD);
        DR4BMotor2.setDirection(DcMotor.Direction.REVERSE);

        DR4BMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DR4BMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DR4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DR4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DR4BMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DR4BMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(20);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("CAMERA ERROR=%d", errorCode);
            }
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine(String.format(Locale.US, "Detected tag ID: %d", tagOfInterest.id));
                }
                else
                {
                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("A tag? Nope, never seen one before.");
                    }
                    else
                    {
                        telemetry.addLine(String.format(Locale.US, "Hmm... The last tag I saw was tag %d, but it was a while ago!", tagOfInterest.id));
                    }
                }

            }
            else
            {
                if(tagOfInterest == null)
                {
                    telemetry.addLine("A tag? Nope, never seen one before.");
                }
                else
                {
                    telemetry.addLine(String.format(Locale.US, "Hmm... The last tag I saw was tag %d, but it was a while ago!", tagOfInterest.id));
                }

            }

            telemetry.update();
            sleep(20);
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine(String.format(Locale.US, "Located tag %d!", tagOfInterest.id));
        }
        else
        {
            telemetry.addLine("No tag found... Let's guess!");
        }
        telemetry.update();

        //START:

        clawFinger.setPosition(clawOpen);
        sleep(1000);
        clawFinger.setPosition(clawClose);

        //move forward to high junction
        forwardPID(50.125);

        armControl(HIGH_JUNCTION_TICKS);

        //turn to high junction
        turn(-46);

        forwardPID(5.8);

        //wait until arm is at height
        while (DR4BMotor1.isBusy() && DR4BMotor2.isBusy()) {

        }
        DR4BMotor1.setPower(0);
        DR4BMotor2.setPower(0);

        //open claw
        clawFinger.setPosition(clawOpen);
        sleep(1000);
        clawFinger.setPosition(clawClose);

        //straighten
        forwardPID(-5);

        armControl(0);

        //go to parking location
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            turn(135);
            forwardPID(23);
        }else if(tagOfInterest.id == MIDDLE){
            //Do nothing
        }else{
            turn(-44);
            forwardPID(23);
        }
    }


    private void forwardPID(double targetInches) {
        int location = (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
        final int target = (int) (COUNTS_PER_INCH * targetInches) + location; //in encoder ticks
        double error = (target - location);
        final int DELTA_T = 35;

        //K constants
        final double K_P_MOVE = 0.0008;
        final double K_D_MOVE = 0.0105;

        final double D_MULT_MOVE = K_D_MOVE / DELTA_T;

        while (opModeIsActive() && Math.abs(error) >= 15) {
            location = (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
            double prevError = error;
            error = (target - location);
            //P
            double P = K_P_MOVE * error;
            //D
            double D = D_MULT_MOVE * (error - prevError);
            //Set power using PID
            double drivePower = Math.tanh(P + D); //cap power at += 1

            leftDrive.setPower(drivePower);
            rightDrive.setPower(drivePower);

            //set RV4B power to zero when motors are off - may be a redundancy
            armCheck();

            telemetry.addData("Location: ", leftDrive.getCurrentPosition());
            telemetry.addData("Target: ", target);
            telemetry.addData("Error Number: ", error);
            telemetry.addData("Raw Drive Power: ", drivePower);
            telemetry.addData("Target Inch: ", targetInches);
            telemetry.update();

            sleep(DELTA_T);
        }
    }


    //utilizes 180 to -180
    //right - negative
    //left - positive
    //RELATIVE TO ROBOT
    private void turn(double degrees) {
        //angle to ticks conversion
        //11 inch = 90 deg.
        //11/90 deg = 1 deg.
        final double TICKS_PER_DEGREE = 11.0 / 90 * COUNTS_PER_INCH;
        final double DEGREES_PER_TICK = 90.0/(11*COUNTS_PER_INCH);

        double leftTarget = -TICKS_PER_DEGREE * degrees + leftDrive.getCurrentPosition(); //left negative
        double rightTarget = TICKS_PER_DEGREE * degrees + rightDrive.getCurrentPosition(); //right positive

        double leftError = leftTarget;
        double rightError = rightTarget;

        //K constants
        final double K_P_TURN = 0.0015 * 5.969 * DEGREES_PER_TICK;
        final double K_D_TURN = 0.03 * DEGREES_PER_TICK;
        final int DELTA_T = 35;

        final double D_MULT_TURN = K_D_TURN / DELTA_T;

        while (opModeIsActive() && Math.abs(leftError) >= 15 && Math.abs(rightError) >= 15) {
            double leftLocation = leftDrive.getCurrentPosition();
            double prevLeftError = leftError;
            leftError = leftTarget - leftLocation;

            double P_LEFT = K_P_TURN * leftError;
            //D
            double D_LEFT = D_MULT_TURN * (leftError - prevLeftError);
            //Set power using PID
            double leftPower = P_LEFT + D_LEFT;

            double rightLocation = rightDrive.getCurrentPosition();
            double prevRightError = rightError;
            rightError = rightTarget - rightLocation;

            double P_RIGHT = K_P_TURN * rightError;
            //D
            double D_RIGHT = D_MULT_TURN * (rightError - prevRightError);
            //Set power using PID
            double rightPower = P_RIGHT + D_RIGHT; //cap power at += 1

            leftDrive.setPower(Math.tanh(leftPower));
            rightDrive.setPower(Math.tanh(rightPower));

            //set RV4B power to zero when motors are off - may be a redundancy
            if (!DR4BMotor1.isBusy() && !DR4BMotor2.isBusy()) {
                DR4BMotor1.setPower(0);
                DR4BMotor2.setPower(0);
            }

            telemetry.addData("Left Location", leftDrive.getCurrentPosition());
            telemetry.addData("Left Target: ", leftTarget);
            telemetry.addData("Left Error: ", leftError);
            telemetry.addData("Left Raw Drive Power: ", leftPower);
            telemetry.addData("Left Drive Power: ", leftDrive.getPower());
            telemetry.addLine("\n\n");
            telemetry.addData("Right Location", rightDrive.getCurrentPosition());
            telemetry.addData("Right Target: ", rightTarget);
            telemetry.addData("Right Error: ", rightError);
            telemetry.addData("Right Raw Drive Power: ", rightPower);
            telemetry.addData("Right Drive Power: ", rightDrive.getPower());
            telemetry.update();

            sleep(DELTA_T);
        }
        armCheck();
    }

    private void armControl(int loc) {
        if (opModeIsActive()) {
            DR4BMotor1.setTargetPosition(loc);
            DR4BMotor2.setTargetPosition(loc);

            DR4BMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DR4BMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DR4BMotor1.setPower(ARM_POWER);
            DR4BMotor2.setPower(ARM_POWER);
        }
    }

    //use in the future as a PDF threshold checker
    private void armCheck() {
        if (!DR4BMotor1.isBusy() && !DR4BMotor2.isBusy()) {
            DR4BMotor1.setPower(0);
            DR4BMotor2.setPower(0);
        }
    }
}