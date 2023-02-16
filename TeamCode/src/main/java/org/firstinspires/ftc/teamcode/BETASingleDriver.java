/*
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
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * TelelOp for Double Reverse Virtual 4 Bar with Macro
 */
@TeleOp(name= "Beta Single Driver - RV4B", group="Robot")
public class BETASingleDriver extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor RV4BMotor1 = null;
    public DcMotor RV4BMotor2 = null;
    public Servo clawFinger = null;

    static final int  HIGH_JUNCTION_TICKS = 690;
    static final int  MEDIUM_JUNCTION_TICKS = 500;
    static final int  LOW_JUNCTION_TICKS = 320;

    static final double MACRO_POWER = 0.6; //for quick adjustments
    static final int ARM_SPEED_MANUAL = 10;
    private ElapsedTime runtime = new ElapsedTime();

    IMU imu;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        int targetPos = 0;
        int error1 = 0;
        int error2 = 0;
        int prevPos1 = 0;
        int prevPos2 = 0;
        double threshold = 0.125;
        double SPEED_MULT = 0.75;
        boolean xStorage = false;
        double targetDegrees = 0;
        boolean turning90 = false;
        double currentAngle = 0;
        double errorTurn;

        //K Variable Bank
        double K_P = 0.0025;
        double K_D = 0.0025;
        double K_ADJ = 0.01;
        int DELTA_T = 35;
        double D_MULT = K_D / DELTA_T;
        double F = 0.07;

        final double K_P_TURN = 0.0015; //0.0015
        final double K_D_TURN = 0.03;
        final double D_MULT_TURN = K_D_TURN / telemetry.getMsTransmissionInterval();;

        //Telemetry update variables:
        String speed = "Normal";
        String fingerPos = "Closed";
        boolean manualControl = false;

        // Define and Initialize Motors and Servos
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");
        RV4BMotor1 = hardwareMap.get(DcMotor.class, "MotorC");
        RV4BMotor2 = hardwareMap.get(DcMotor.class, "MotorD");
        clawFinger = hardwareMap.get(Servo.class, "ServoFinger");
        imu = hardwareMap.get(IMU.class, "imu");

        //core hex motors are facing opposite each other and will rotate in opposite directions
        RV4BMotor1.setDirection(DcMotor.Direction.FORWARD);
        RV4BMotor2.setDirection(DcMotor.Direction.REVERSE);

        RV4BMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RV4BMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RV4BMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RV4BMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        runtime.reset();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            if (drive  >= 0.05 || turn >= 0.05) {
                turning90 = false;
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            //Handle speed multiplication
            if (gamepad1.right_stick_button && !xStorage) {
                xStorage = true;
                if (SPEED_MULT <= 0.35) {
                    SPEED_MULT = 0.75;
                    speed = "Normal";
                }
                else {
                    SPEED_MULT = 0.35;
                    speed = "Slow";
                }
            }
            else{
                xStorage = false;
            }

            //Macros!
            if (gamepad1.y) { //go to high
                targetPos = HIGH_JUNCTION_TICKS;
            }
            else if (gamepad1.b) { //go to medium
                targetPos = MEDIUM_JUNCTION_TICKS;
            }
            else if (gamepad1.x) { //go to low
                targetPos = LOW_JUNCTION_TICKS;
            }
            else if (gamepad1.a) { //go to ground
                targetPos = 0;
                clawFinger.setPosition(0.1);
            }

            if (gamepad1.left_trigger >= 0.4 && RV4BMotor1.getCurrentPosition() > 0 && RV4BMotor2.getCurrentPosition() > 0){ //go down manually
                manualControl = true;
                targetPos = (RV4BMotor1.getCurrentPosition() + RV4BMotor2.getCurrentPosition())/2;
                RV4BMotor1.setPower(-MACRO_POWER);
                RV4BMotor2.setPower(-MACRO_POWER);
            }
            else if (gamepad1.right_trigger >= 0.4 && RV4BMotor1.getCurrentPosition() < 690 && RV4BMotor2.getCurrentPosition() < 690){ //go up manually
                manualControl = true;
                targetPos = (RV4BMotor1.getCurrentPosition() + RV4BMotor2.getCurrentPosition())/2;
                RV4BMotor1.setPower(MACRO_POWER);
                RV4BMotor2.setPower(MACRO_POWER);
            } else {
                manualControl = false;
            }

            double powerPDF1 = 0;
            double powerPDF2 = 0;

            //Base PID
            if (!manualControl) {
                error1 = targetPos - RV4BMotor1.getCurrentPosition();
                double P1 = K_P * error1;
                double D1 = D_MULT * (prevPos1 - RV4BMotor1.getCurrentPosition());
                powerPDF1 = MACRO_POWER * (P1 + D1);
                prevPos1 = RV4BMotor1.getCurrentPosition();

                if (350 < RV4BMotor1.getCurrentPosition() && RV4BMotor1.getCurrentPosition() < 550) {
                    powerPDF1 -= F;
                }

                error2 = targetPos - RV4BMotor2.getCurrentPosition();
                double P2 = K_P * error2;
                double D2 = D_MULT * (prevPos2 - RV4BMotor2.getCurrentPosition());
                powerPDF2 = MACRO_POWER * (P2 + D2);
                prevPos2 = RV4BMotor2.getCurrentPosition();

                if (350 < RV4BMotor2.getCurrentPosition() && RV4BMotor2.getCurrentPosition() < 550) {
                    powerPDF2 -= F;
                }

                if (350 < targetPos && targetPos < 550) {
                    threshold = 0.0625;
                }
                else{
                    threshold = 0.125;
                }

                //If the values are slanting, adjust for it!
/*                int diff = RV4BMotor1.getCurrentPosition() - RV4BMotor2.getCurrentPosition();
                powerPDF1 -= diff * K_ADJ;
                powerPDF2 += diff * K_ADJ;*/

                if (Math.abs(powerPDF1) <= threshold || Math.abs(error1) <= 15){ //if power is less than 0.1 OR error is less than 15, set power to zero
                    powerPDF1 = 0;
                }

                if (Math.abs(powerPDF2) <= threshold || Math.abs(error2) <= 15) { //if power is less than 0.1 OR error is less than 15, set power to zero
                    powerPDF2 = 0;
                }

                //Final RV4B Motor Powers
                RV4BMotor1.setPower(Math.tanh(powerPDF1));
                RV4BMotor2.setPower(Math.tanh(powerPDF2));
            }

            //Handle claw open and close
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                clawFinger.setPosition(0.4); //open
                fingerPos = "Open";
            }
            else if (gamepad1.left_bumper || gamepad1.right_bumper){
                clawFinger.setPosition(0.1); //close
                fingerPos = "Closed";
            }

            //Drive!
            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;
            // Normalize the values so neither exceed +/FinalControlScheme- 1.0
            left = Math.tanh(left);
            right = Math.tanh(right);

            /*//90 degree buttons
            if(gamepad1.dpad_left){
                targetDegrees = 90;
                turning90 = true;
            }
            else if(gamepad1.dpad_right){
                targetDegrees = -90;
                turning90 = true;
            } else if (gamepad1.dpad_up){
                targetDegrees = 0;
                turning90 = true;
            } else if (gamepad1.dpad_down) {
                targetDegrees = 180;
                turning90 = true;
            }

            if (turning90){
                double prevAngle = currentAngle;
                currentAngle = orientation.getYaw(AngleUnit.DEGREES);

                errorTurn = targetDegrees - currentAngle;

                //get most efficient angle (imu has angles from -180 to 180)
               if (errorTurn > 180) {
                    errorTurn -= 360;
                } else if (errorTurn < -180) {
                    errorTurn += 360;
                }

                double turnP = K_P_TURN * errorTurn * 5.969; //convert angle to ticks so that the P still applies
                double turnD = D_MULT_TURN * (currentAngle - prevAngle);

                double powerTurn = Math.tanh(turnP + turnD); //Normalize power to +/- 1.0

                telemetry.addLine("ROTATING");
                telemetry.addData("Turning error", errorTurn);
                telemetry.addData("Turning power P", turnP);
                telemetry.addData("Turning power D", turnD);
                telemetry.addData("Final turning power", powerTurn);

                leftDrive.setPower(-powerTurn);
                rightDrive.setPower(powerTurn);

                *//*if (targetDegrees < currentAngle && Math.abs(errorTurn) > 2) {
                    leftDrive.setPower(-powerTurn);
                    rightDrive.setPower(powerTurn);
                } else if (targetDegrees > currentAngle && Math.abs(errorTurn) > 2) {
                    leftDrive.setPower(powerTurn);
                    rightDrive.setPower(-powerTurn);
                } else {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    turning90 = false;
                }*//*
            }
            else{
                // Output the normalized vales to the motor drives.
                leftDrive.setPower(left * SPEED_MULT);
                rightDrive.setPower(right * SPEED_MULT);
                turning90 = false;
            }*/

            leftDrive.setPower(left * SPEED_MULT);
            rightDrive.setPower(right * SPEED_MULT);

            // Send telemetry message to signify robot running;
            telemetry.addData("Speed", speed);
            telemetry.addData("Stick X",  "%.2f", turn);
            telemetry.addData("Stick Y", "%.2f", (drive * -1));
            telemetry.addData("Left drive motor power", leftDrive.getPower());
            telemetry.addData("Right drive motor power", rightDrive.getPower());
            telemetry.addData("Claw", fingerPos);
            telemetry.addData("RV4B Power A", "%.2f", RV4BMotor1.getPower());
            telemetry.addData("RV4B Power B", "%.2f", RV4BMotor2.getPower());
            telemetry.addData("RV4B Motor A Encoder", RV4BMotor1.getCurrentPosition());
            telemetry.addData("RV4B Motor B Encoder", RV4BMotor2.getCurrentPosition());
            telemetry.addData("RV4B Target", targetPos);
            telemetry.addData("PDF Power 1", powerPDF1);
            telemetry.addData("PDF Power 2", powerPDF2);
            telemetry.addData("RV4B Error 1", error1);
            telemetry.addData("RV4B Error 2", error2);
            telemetry.addData("PDF Power Threshold", threshold);

            //IMU Telemetry
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate); //rotational location

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(DELTA_T);
        }
    }
}