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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;

@Autonomous(name="LEFT Plus One", group="Robot")
@Disabled
public class PlusOneAutoLeft extends LinearOpMode
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 20;   //gear ratio
    static final double     WHEEL_DIAMETER_INCH     = 3.5;    // For figuring circumference: 90mm
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

    static final double     RVA_COUNTS_PER_MOTOR_REV    = 288 ;    //Core Hex Motor
    static final double     RVA_DRIVE_GEAR_REDUCTION    = 4.167;   //gear ratio of gears and sprockets (125t/15t * 20t/40t = 4.167)
    static final double     FINAL_GEAR_DIAMETER_INCH     = 1.60;    //diameter of starting, 15t gear (CHECK THIS NUMBER!!)
    static final double     RVA_COUNTS_PER_INCH  = (RVA_COUNTS_PER_MOTOR_REV * RVA_DRIVE_GEAR_REDUCTION) / (FINAL_GEAR_DIAMETER_INCH * Math.PI); //ticks for motor to get one revolution at first gear * number of revolutions needed to have one revolution at output/output gear's circumference in inches (calculates number of ticks needed to get one inch of output)
    static final double     RVA_STARTING_INCHES = 16; //CHECK THIS NUMBER - RVA elevated to height

    static final double     RVA_INCHES_TO_REACH = 33.5 - RVA_STARTING_INCHES; //goes to high junction - adjust this number for claw
    static final double     RVA_TICKS_TO_REACH = RVA_INCHES_TO_REACH * RVA_COUNTS_PER_INCH;

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
    public void runOpMode()
    {
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

        telemetry.setMsTransmissionInterval(50);

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

        //go to location
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            //Move forward ~28.5" = 23.5" + 3" + 2" (clear first tile then clear half of junction diameter then 2 inches for clearance)
            encoderDrive(0.1, 23, 23);
            //turn 90 degrees left
            encoderDrive(0.1, -11, 11);
            //move forward ~24"
            encoderDrive(0.1, 22, 22);
        }else if(tagOfInterest.id == MIDDLE){
            //move forward 87 cm (34.25") to sit in the middle of the two tiles in front
            encoderDrive(0.1, 34, 34);
        }else{
            //Move forward ~28.5" = 23.5" + 3" + 2" (clear first tile then clear half of junction diameter then 2 inches for clearance)
            encoderDrive(0.1, 23, 23);
            //turn 90 degrees right
            encoderDrive(0.1, 11, -11);
            //move forward ~19"
            encoderDrive(0.1, 22, 22);
        }
    }

    //from RobotAutoDriveByEncoder_Linear example
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}