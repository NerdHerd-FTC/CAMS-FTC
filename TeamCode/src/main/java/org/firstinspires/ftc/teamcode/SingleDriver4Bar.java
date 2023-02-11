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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TelelOp for Double Reverse Virtual 4 Bar with Macro
 */
@TeleOp(name= "Single Driver - RV4B", group="Robot")
public class SingleDriver4Bar extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor RV4BMotor1 = null;
    public DcMotor RV4BMotor2 = null;
    public Servo clawFinger = null;

    static final int  HIGH_JUNCTION_TICKS = -685;
    static final int  MEDIUM_JUNCTION_TICKS = -542;
    static final int  LOW_JUNCTION_TICKS = -400;

    static final double MACRO_POWER = 0.4; //for quick adjustments
    static final double ARM_POWER = 0.7;
    static final int ARM_SPEED_MANUAL = 15;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double targetPos = 0;
        int error1 = 0;
        int error2 = 0;
        double SPEED_MULT = 0.75;
        boolean xStorage = false;

        //K Variable Bank
        double K_P = 0.03;
        double K_D = 0.03;
        double K_ADJ = 0.03;
        int DELTA_T = 35;
        double D_MULT = K_D / DELTA_T;
        double F = 0.1;

        //Telemetry update variables:
        String speed = "Normal";
        String fingerPos = "Closed";

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
            drive = gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            //Handle speed multiplication
            if (gamepad1.x && !xStorage) {
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
            }

            if (gamepad1.left_trigger >= 0.4 && RV4BMotor1.getCurrentPosition() < 0 && RV4BMotor2.getCurrentPosition() < 0){ //go down manually
                targetPos += ARM_SPEED_MANUAL;
            }
            else if (gamepad1.right_trigger >= 0.4 && RV4BMotor1.getCurrentPosition() > -690 && RV4BMotor2.getCurrentPosition() > -690){ //go up manually
                targetPos -= ARM_SPEED_MANUAL;
            }

            //Base PID
            int prevError1 = error1;
            error1 = RV4BMotor1.getCurrentPosition();
            double P1 = K_P * error1;
            double D1 = D_MULT * (error1 - prevError1);
            double powerPDF1 = MACRO_POWER * (P1 + D1);
            if (350 < RV4BMotor1.getCurrentPosition() && RV4BMotor1.getCurrentPosition() < 550){
                powerPDF1 -= F;
            }

            int prevError2 = error2;
            error2 = RV4BMotor1.getCurrentPosition();
            double P2 = K_P * error2;
            double D2 = D_MULT * (error2 - prevError2);
            double powerPDF2 = MACRO_POWER * (P2 + D2);
            if (350 < RV4BMotor1.getCurrentPosition() && RV4BMotor1.getCurrentPosition() < 550){
                powerPDF2 -= F;
            }

            //If the values are slanting, adjust for it!
            int diff = RV4BMotor1.getCurrentPosition() - RV4BMotor2.getCurrentPosition();
            powerPDF1 -= diff * K_ADJ;
            powerPDF2 += diff * K_ADJ;

            //Final RV4B Motor Powers
            RV4BMotor1.setPower(Math.tanh(powerPDF1));
            RV4BMotor2.setPower(Math.tanh(powerPDF2));

            //Handle claw open and close
            if (gamepad1.left_bumper){
                clawFinger.setPosition(0.1); //close
                fingerPos = "Closed";
            }
            else if (gamepad1.right_bumper){
                clawFinger.setPosition(0.4); //open
                fingerPos = "Open";
            }


            //Drive!
            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;
            // Normalize the values so neither exceed +/FinalControlScheme- 1.0
            left = Math.tanh(left);
            right = Math.tanh(right);
            // Output the safe vales to the motor drives.
            leftDrive.setPower(left * SPEED_MULT);
            rightDrive.setPower(right * SPEED_MULT);

            // Send telemetry message to signify robot running;
            telemetry.addData("Speed", speed);
            telemetry.addData("Stick X",  "%.2f", turn);
            telemetry.addData("Stick Y", "%.2f", (drive * -1));
            telemetry.addData("Claw", fingerPos);
            telemetry.addData("RV4B Power A", "%.2f", RV4BMotor1.getPower());
            telemetry.addData("RV4B Power B", "%.2f", RV4BMotor2.getPower());
            telemetry.addData("RV4B Motor A Encoder", RV4BMotor1.getCurrentPosition());
            telemetry.addData("RV4B Motor B Encoder", RV4BMotor2.getCurrentPosition());
            telemetry.addData("RV4B Target", targetPos);
            telemetry.addData("PDF Power 1", powerPDF1);
            telemetry.addData("PDF Power 2", powerPDF2);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(DELTA_T);
        }
    }
}