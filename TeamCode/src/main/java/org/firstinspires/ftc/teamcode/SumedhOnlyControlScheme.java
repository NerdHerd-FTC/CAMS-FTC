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
 * Teleop Final Control Scheme - Uses Triggers for Speed Mult
 */
@TeleOp(name= "Sumedh ONLY: Final Control Scheme", group="Robot")
public class SumedhOnlyControlScheme extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  RVAMotor1   = null;
    public DcMotor  RVAMotor2  = null;
    public Servo clawFinger = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double SPEED_MULT = 0.5;

        double ArmPower;
        double fingerTarget = 0.7;
        double wristTarget = 0.3;
        final double CLAW_SPEED = 0.05; //strictly less than 1

        //Telemetry update variables:
        String speed = "Normal";
        String fingerPos = "Closed";

        // Define and Initialize Motors and Servos
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");

        RVAMotor1  = hardwareMap.get(DcMotor.class, "MotorC");
        RVAMotor2 = hardwareMap.get(DcMotor.class, "MotorD");

        clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        RVAMotor1.setDirection(DcMotor.Direction.FORWARD);
        RVAMotor2.setDirection(DcMotor.Direction.REVERSE);

        RVAMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RVAMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RVAMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RVAMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RVAMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RVAMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawFinger.setPosition(0);

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
            drive = -1 * gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            //Handle speed multiplication
            if (gamepad1.a && runtime.seconds() >= 2) {
                runtime.reset();
                if (SPEED_MULT <= 0.25) {
                    SPEED_MULT = 0.5;
                    speed = "Normal";
                }
                else {
                    SPEED_MULT = 0.25;
                    speed = "Slow";
                }
            }

            if (gamepad1.left_trigger >= 0.4){
                ArmPower = -0.65;
            }
            else if (gamepad1.right_trigger >= 0.4){
                ArmPower = 0.65;
            }
            else {
                ArmPower = 0;
            }
            RVAMotor1.setPower(ArmPower);
            RVAMotor2.setPower(ArmPower);

            //Handle claw open and close
            if (gamepad1.left_bumper){
                clawFinger.setPosition(0); //close
                fingerPos = "Closed";
            }
            else if (gamepad1.right_bumper){
                clawFinger.setPosition(0.5); //open
                fingerPos = "Open";
            }

            //Drive!
            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;
            // Normalize the values so neither exceed +/FinalControlScheme- 1.0
            if (left > 1.0) {
                left = 1.0;
            }
            if (right > 1.0){
                right = 1.0;
            }
            // Output the safe vales to the motor drives.
            leftDrive.setPower(Math.pow(left, 3) * SPEED_MULT);
            rightDrive.setPower(Math.pow(right, 3) * SPEED_MULT);

            // Send telemetry message to signify robot running;
            telemetry.addData("Speed: ", "String", speed);
            telemetry.addData("Stick X: ",  "%.2f", turn);
            telemetry.addData("Stick Y: ", "%.2f", (drive * -1));
            telemetry.addData("Fingers are: ", fingerPos);
            telemetry.addData("Power: ", "%.2f", ArmPower);
            telemetry.addData("RVA Motor A Encoder: %7d", RVAMotor1.getCurrentPosition());
            telemetry.addData("RVA Motor B Encoder: %7d", RVAMotor2.getCurrentPosition());
            telemetry.addData("Claw Finger: ", fingerPos);
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}