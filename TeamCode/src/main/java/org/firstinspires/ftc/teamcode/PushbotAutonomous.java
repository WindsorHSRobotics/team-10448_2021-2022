package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.PushbotAutonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Autonomous System", group="Autonomous")
//@Disabled
public class PushbotAutonomous extends LinearOpMode {
    HardwarePushbot       robot=new HardwarePushbot();
    private ElapsedTime   runtime = new ElapsedTime();


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData("Status","working");
        telemetry.update();
        getRuntime();
        while (getRuntime()<=30) {
            getRuntime();
            telemetry.addData(String.valueOf(runtime), "Working");
        }
    }
}

