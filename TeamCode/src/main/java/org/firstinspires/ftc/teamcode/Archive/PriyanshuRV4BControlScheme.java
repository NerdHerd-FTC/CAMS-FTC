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
package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TelelOp for Double Reverse Virtual 4 Bar with Macro
 */
@TeleOp(name= "Single Driver - Mr. Sharma", group="Robot")
@Disabled
public class PriyanshuRV4BControlScheme extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor RV4BMotor1 = null;
    public DcMotor RV4BMotor2 = null;
    //public Servo clawFinger = null;

    static final int  TICKS_TO_REACH = 770;
    static final double MACRO_POWER = Math.abs(0.65); //for quick adjustments
    static final double ARM_POWER = Math.abs(0.65); //prevent rogue negatives
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double SPEED_MULT = 0.5;

        //Telemetry update variables:
        String speed = "Normal";
        String fingerPos = "Closed";

        // Define and Initialize Motors and Servos
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");
        RV4BMotor1 = hardwareMap.get(DcMotor.class, "MotorC");
        RV4BMotor2 = hardwareMap.get(DcMotor.class, "MotorD");
        //clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

        //core hex motors are facing opposite each other and will rotate in opposite directions
        RV4BMotor1.setDirection(DcMotor.Direction.FORWARD);
        RV4BMotor2.setDirection(DcMotor.Direction.REVERSE);

        RV4BMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RV4BMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RV4BMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RV4BMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

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
            // Run wheels in POV mode
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y + gamepad1.right_stick_y;
            turn = gamepad1.right_stick_x + gamepad1.left_stick_x;

            //Handle speed multiplication
            if (gamepad1.b) {
                SPEED_MULT = 1;
                speed = "Fast";
            }
            else if (gamepad1.y){
                SPEED_MULT = 0.25;
                speed = "Slow";
            }
            else{
                SPEED_MULT = 0.5;
                speed = "Normal";
            }
            //set power to zero when motors are off
            if (!RV4BMotor1.isBusy() && !RV4BMotor2.isBusy()) {
                RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RV4BMotor1.setPower(0);
                RV4BMotor2.setPower(0);
            }

            //macro
            if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.left_trigger >= 0.4 || gamepad1.right_trigger >= 0.4) { //kill switch!
                RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                RV4BMotor1.setPower(0);
                RV4BMotor2.setPower(0);
            } else if (gamepad1.dpad_up) { //go up
                RV4BMotor1.setTargetPosition(TICKS_TO_REACH);
                RV4BMotor2.setTargetPosition(TICKS_TO_REACH);

                RV4BMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RV4BMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                RV4BMotor1.setPower(MACRO_POWER);
                RV4BMotor2.setPower(MACRO_POWER);
            } else if (gamepad1.dpad_down) { //go down
                RV4BMotor1.setTargetPosition(0);
                RV4BMotor2.setTargetPosition(0);

                RV4BMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RV4BMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                RV4BMotor1.setPower(MACRO_POWER);
                RV4BMotor2.setPower(MACRO_POWER);
            }
/** Doesn't use a macro
            if (gamepad1.left_trigger >= 0.4){ //go down
                RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                RV4BMotor1.setPower(-ARM_POWER);
                RV4BMotor2.setPower(-ARM_POWER);
            }
            else if (gamepad1.right_trigger >= 0.4){ //go up
                RV4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RV4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                RV4BMotor1.setPower(ARM_POWER);
                RV4BMotor2.setPower(ARM_POWER);
            } else if (RV4BMotor1.getMode() == DcMotor.RunMode.RUN_USING_ENCODER && RV4BMotor2.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) { //stop if there's no macro involved & if there are no trigger action
                RV4BMotor1.setPower(0);
                RV4BMotor2.setPower(0);
            }
 */

            //Handle claw open and close
            if (gamepad1.x){
                //clawFinger.setPosition(0.2); //close
                fingerPos = "Closed";
            }
            else if (gamepad1.a){
                //clawFinger.setPosition(0.5); //open
                fingerPos = "Open";
            }

            //Drive!
            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;
            // Output the safe vales to the motor drives. (Use tanh to normalize the values so neither exceeds +/-1 bb)
            leftDrive.setPower(Math.tanh(SPEED_MULT * left));
            rightDrive.setPower(Math.tanh(SPEED_MULT * right));

            // Send telemetry message to signify robot running;
            telemetry.addData("Speed", "String", speed);
            telemetry.addData("Stick X",  "%.2f", turn);
            telemetry.addData("Stick Y", "%.2f", (drive * -1));
            //telemetry.addData("Claw: ", fingerPos);
            telemetry.addData("RV4B Power A", "%.2f", RV4BMotor1.getPower());
            telemetry.addData("RV4B Power B", "%.2f", RV4BMotor2.getPower());
            telemetry.addData("RV4B Motor A Encoder", RV4BMotor1.getCurrentPosition());
            telemetry.addData("RV4B Motor B Encoder", RV4BMotor2.getCurrentPosition());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}