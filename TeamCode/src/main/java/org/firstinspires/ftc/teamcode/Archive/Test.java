
package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
@Disabled
public class Test extends OpMode {

    @Override
    public void init () {
        telemetry.addData ("Hello","Test");

    }

    @Override
    public void loop() {

    }
}