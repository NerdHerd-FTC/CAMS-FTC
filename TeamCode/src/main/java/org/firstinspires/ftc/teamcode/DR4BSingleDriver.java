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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TelelOp for Double Reverse Virtual 4 Bar with Macro
 */
@TeleOp(name= "Single Driver - DR4B", group="Robot")
public class DR4BSingleDriver extends LinearOpMode {
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
        double speedBOOST = 1;

        //K Variable Bank
        double K_P = 0.0025;
        double K_D = 0.0025;
        double K_ADJ = 0.01;
        int DELTA_T = 35;
        double D_MULT = K_D / DELTA_T;
        double F = 0.07;

        /*final double K_P_TURN = 0.0015; //0.0015
        final double K_D_TURN = 0.03;
        final double D_MULT_TURN = K_D_TURN / telemetry.getMsTransmissionInterval();*/

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

            //OPERATOR CONTROL:
            //reset encoders by clicking X & left stick button - intentionally very awkward
            if (gamepad2.x && gamepad2.left_stick_button) {
                RV4BMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RV4BMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addLine("ENCODERS RESET!");
            } else if (Math.abs(gamepad2.left_stick_y) >= 0.4 && Math.abs(gamepad2.right_stick_y) >= 0.4) {
                //manual control of lift, regardless of encoder limits
                double stickPower = (-gamepad2.left_stick_y - gamepad2.right_stick_y)/2;
                double reducedPower = stickPower * 0.4;
                RV4BMotor1.setPower(reducedPower);
                RV4BMotor2.setPower(reducedPower);
            }

            if (Math.abs(gamepad2.right_trigger) >= 0.4 && Math.abs(gamepad2.left_trigger) >= 0.4) {
                //short-term speed boost, increases max power to 0.9
                speedBOOST = 2;
            } else {
                speedBOOST = 1;
            }

            leftDrive.setPower(Math.tanh(left * SPEED_MULT * speedBOOST));
            rightDrive.setPower(Math.tanh(right * SPEED_MULT * speedBOOST));

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

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(DELTA_T);
        }
    }
}