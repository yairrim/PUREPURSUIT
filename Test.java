package teamcode;


import static java.lang.Math.abs;
import static teamcode.MathFunctions.dist2D;

import virtual_robot.controller.LinearOpMode;

import java.util.ArrayList;


/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class Test extends LinearOpMode {
    RobotMovement robotMovement = new RobotMovement();
    ArrayList<CurvePoint> path = new ArrayList();
    ArrayList<CurvePoint> path1 = new ArrayList();
    ArrayList<CurvePoint> path2 = new ArrayList();

    public void runOpMode(){
        robotMovement.robot.init(hardwareMap);

        //set the points for the first Path
        path1.add(new CurvePoint(200,200,1.0,0.5,10,0,1.0));
        path1.add(new CurvePoint(120,150,1,0.5,10,0,1.0));
        path1.add(new CurvePoint(100,83,1,0.5,10,0,1.0));


        //set the points for the second Path
        path2.add(new CurvePoint(300,200,1.0,0.5,50,0,1.0));
        path2.add(new CurvePoint(283,500,1,0.5,10,0,1.0));
        path2.add(new CurvePoint(500,500,1,0.5,10,0,1.0));



        telemetry.addData("finish",path1.get(path1.size() - 1).toPoint().x);
        telemetry.update();
        waitForStart();

        path = path1;  // set the main path as the first path
        purePursuit(); // preform the purePursuit algorithm
        /** do an autonomous task or something */

        path = path2;  // set the main path as the second path
        purePursuit(); // preform the purePursuit algorithm

        while (opModeIsActive()){ // display staff at the end and stop all drive motors


            robotMovement.robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);

            telemetry.addData("DISTANCE : ", dist2D(robotMovement.followMe,robotMovement.BotPoint()));
            telemetry.addData("bot point", robotMovement.BotPoint());

            telemetry.addData("error",robotMovement.error);
            telemetry.addData("steer", robotMovement.steer);
            telemetry.update();
        }

    }




    private void purePursuit(){
        do{
            telemetry.addData("opMose", "is active");
            telemetry.addData("bot point", robotMovement.BotPoint());

            telemetry.addData("error", robotMovement.error);
            telemetry.addData("steer", robotMovement.steer);
            robotMovement.followCurve(path, 1);
            sleep(30);
            telemetry.addData("DISTANCE : ", dist2D(robotMovement.followMe,robotMovement.BotPoint()));

            telemetry.update();
        }while (!endOfPath() && opModeIsActive());


    }


    private boolean endOfPath(){
        return abs(dist2D(path.get(path.size() - 1).toPoint(),robotMovement.BotPoint())) < 15; // 15 can be modified to what you see fit
    }

}
