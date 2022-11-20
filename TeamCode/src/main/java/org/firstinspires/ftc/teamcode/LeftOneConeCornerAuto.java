package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Left One Cone Corner Auto", group="Robot")
public class LeftOneConeCornerAuto extends LinearOpMode {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public static final double MIN_POSITION = 0.0;
    public static final double MAX_POSITION = 0.5;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;

        
        leftDrive = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");
        
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
       
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());
        

        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        waitForStart();

        
      // TODO: Create a Standardized Move In. Function *
      // TODO: Create a Standardized Turn Deg. Function *
      // TODO: * Allow the Changing of Pre Sets in Constants File
      // TODO: Rename Motors for Functionality
      
        while (rightDrive.getCurrentPosition() < 200) {
            leftDrive.setPower(0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Forward");
            telemetry.update();
            sleep(50);
        }
      
        while (rightDrive.getCurrentPosition() > 300) {
            leftDrive.setPower(-0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Turning");
            telemetry.update();
            sleep(50);
        }
      
        while (rightDrive.getCurrentPosition() < 200) {
            leftDrive.setPower(0.1);
            rightDrive.setPower(0.1);
            telemetry.addData("Pushing");
            telemetry.update();
            sleep(50);
        }
      
        while (rightDrive.getCurrentPosition() < 200) {
            leftDrive.setPower(-0.1);
            rightDrive.setPower(-0.1);
            telemetry.addData("Back");
            telemetry.update();
            sleep(50);
        }
      
        while (rightDrive.getCurrentPosition() > 300) {
            leftDrive.setPower(-0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Turning");
            telemetry.update();
            sleep(50);
        }
      
        while (rightDrive.getCurrentPosition() < 1000) {
            leftDrive.setPower(0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Forward");
            telemetry.update();
            sleep(50);
        }

    }
}
