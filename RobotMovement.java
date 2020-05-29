package teamcode;
import virtual_robot.controller.LinearOpMode;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BinaryOperator;

import static java.lang.Math.*;
import static teamcode.MathFunctions.*;

public class RobotMovement extends LinearOpMode {

    Hardware robot = new Hardware();
    Point followMe;
    static final double P_TURN_COEFF = 0.0005;     // Larger is more responsive, but also less stable
    double error;
    double steer;
    double disaierd_angle;
    private TimeTest times = new TimeTest();
    public void followCurve(ArrayList<CurvePoint> allPoints, double speed) {
        followMe = getFollowPointPath(allPoints, allPoints.get(0).followDistance);
        telemetry.addData("followMe:", "X=> " + followMe.x + " Y => " + followMe.y);


        //double x, double y, double speed, double P_DRIVE_COEFF, ArrayList<CurvePoint> path, double leftSpeed, double rightSpeed, double max
        gotoPosition(followMe, speed,0,0);
    }
    public CurvePoint colsestPathPoint(ArrayList<CurvePoint> path){
        double minDist=0;
        CurvePoint colsest = path.get(0);
        for(CurvePoint p : path){
            if(abs(dist2D(BotPoint(),p.toPoint())) < minDist){
                colsest = p;
                minDist= abs(dist2D(BotPoint(),p.toPoint()));
            }
        }
        return colsest;
    }
    public Point getFollowPointPath(ArrayList<CurvePoint> pathPoints, double followRadius) {
        double total = 0;
        Point followMe = colsestPathPoint(pathPoints).toPoint();
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);
            List<Point> intersections = getCircleLineIntersectionPoint(BotPoint(), followRadius, startLine.toPoint(), endLine.toPoint());
            total += intersections.size();
            double distToEnd = 100000;
            telemetry.addData("intersections.size()",intersections.size());

            for (Point thisIntersection : intersections) {
                //System.out.println(new Point(thisIntersection));
                telemetry.addData("int ==> ",thisIntersection);

                double dist = dist2D(thisIntersection, endLine.toPoint());
                double delta = Math.abs(dist);

                if (delta < distToEnd) {
                    distToEnd = delta;
                    followMe = (thisIntersection);
                }
            }
        }
        telemetry.addData("intersections.size()",total);
        //System.out.println("follow me =" + new Point(followMe.x,followMe.y));
        followMe.x = abs(followMe.x);
        followMe.y = abs(followMe.y);
        return followMe;
    }

    @Override
    public void runOpMode() {
    }

    /**
     * @param p
     * @param speed
     * @return
     */
    public void gotoPosition(Point p, double speed,double preferdAngle,double TurnSpeed) {
        double dist = Math.hypot(p.x - BotPoint().x,p.y - BotPoint().y);
        double absAngle = Math.atan2(p.y - BotPoint().y,p.x - BotPoint().x);
        double reletiveangle = AngleWrap(absAngle);// - Math.toRadians(robot.GetGyroAngle()+ 180));
        double reletiveX = Math.cos(reletiveangle) * dist;
        double reletiveY = Math.sin(reletiveangle) * dist;

        double movementX = reletiveX/(abs(reletiveX) + abs(reletiveY));
        double movementY = reletiveY/ (abs(reletiveX) + abs(reletiveY));

// turning while driving
        double reletiveTurn = reletiveangle - Math.toRadians(180) + preferdAngle;
        double turn = Range.clip(reletiveTurn/Math.toRadians(30),-1,1);
        robot.move(movementX * speed,movementY * speed,turn * TurnSpeed);

    }

    public ArrayList<CurvePoint> gotoPositionV2(Point target, double speed, double P_DRIVE_COEFF, ArrayList<CurvePoint> path) {


        double angleToTarget = Math.toDegrees(Math.atan2(target.y - bot.getPoint().y,target.x - bot.getPoint().x)) - robot.GetGyroAngle();
        angleToTarget = MathFunctions.RotateAngle(angleToTarget,90);
        double distanceToTarget = abs(MathFunctions.dist2D(target,new Point(bot.getPoint().x , bot.getPoint().y)));
        double timeToTravle = (distanceToTarget/10) * times.timeToDrive;
        double timeToTurn   =  ((1/(360/angleToTarget))) * times.timeToRotate;


        double steer = (((1/(times.timeToRotate/timeToTurn))) / abs((1/(times.timeToRotate/timeToTurn)))) - ((1/(times.timeToRotate/timeToTurn)));
        speed = range(((1/(times.timeToDrive/timeToTravle))),1,-1);
        telemetry.addData("time to rotate - " , timeToTurn);
        telemetry.addData("time to Drive - " , timeToTravle);

        telemetry.addData(" ---------- "," ---------- ");

        telemetry.addData("angle to target - " , angleToTarget);
        telemetry.addData("distance to target - " , distanceToTarget);

        telemetry.addData(" ---------- "," ---------- ");

        telemetry.addData("steer - " , steer);
        telemetry.addData("speed" , speed);

        double leftSpeed = speed;
        double rightSpeed = speed;

        if(angleToTarget > 2) {
            leftSpeed  += steer;
            rightSpeed -= steer;
        }else if(angleToTarget < -2){
            leftSpeed  -= steer;
            rightSpeed += steer;
        }

        // Normalize speeds if either one exceeds +/- 1.0;

        robot.setDriveMotorsPower(leftSpeed, Hardware.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower(rightSpeed, Hardware.DRIVE_MOTOR_TYPES.RIGHT);


        telemetry.update();
//        int currentBaseIndex = getIndex(path,currentBase);
//        if(currentBaseIndex > 0){
//            System.out.println(new Point(currentBase.toPoint()));
//            path.remove(currentBaseIndex - 1);
//        }


        return path;
    }

    /**
     *
     * @param speed
     * @param self
     * @param target
     * @return
     */

    private double PIDcontroll(double speed, Point self, Point target){
        double distance = dist2D(self,target);
        double change = (distance / abs(distance));

        return speed * change;
    }

    public Point BotPoint(){
        return new Point(bot.getPoint().x, bot.getPoint().y);
    }

    /**
     *
     * @param angle
     * @param power
     */
    private void turn(double angle,double power) {
        double leftfrontSpeed;
        double rightfrontSpeed;
        double leftbackSpeed;
        double rightbackSpeed;

        double error;
        double steer;;

        error = getError(angle);
        steer = getSteer(error);
        steer = limit(steer,power);
        leftfrontSpeed = robot.driveLeftFront.getPower() + steer;
        rightfrontSpeed = robot.driveRightFront.getPower() - steer;
        leftbackSpeed = robot.driveLeftBack.getPower() + steer;
        rightbackSpeed = robot.driveRightBack.getPower() - steer;


        robot.driveLeftFront.setPower(leftfrontSpeed);
        robot.driveRightFront.setPower(rightfrontSpeed);
        robot.driveLeftBack.setPower(leftbackSpeed);
        robot.driveRightBack.setPower(rightbackSpeed);
    }

    /**
     *
     * @param powerX
     * @param powerY
     */
    public void drive(double powerX, double powerY){


        double power = hypot(powerX,powerY);
        double Degree = robot.GetGyroAngle() + atan2(powerY,powerX);

        telemetry.addData("befor","X - " + powerX + "\n Y - " + powerY);
        powerY = power * sin(Degree);
        powerX = power * cos(Degree);

        telemetry.addData("after","X - " + powerX + "\n Y - " + powerY);

        robot.driveLeftBack.setPower(powerX);
        robot.driveRightFront.setPower(powerX);

        robot.driveRightBack.setPower(powerY);
        robot.driveLeftFront.setPower(powerY);
    }


    /**
     *
     * @param targetAngle
     * @return
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.GetGyroAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @return
     */
    public double getSteer(double error) {

        return error/(35);
    }



    /**
     *
     * @param value
     * @param maxmin
     * @return
     */
    public double limit(double value,double maxmin) {
        return max(-maxmin, min(value, maxmin));
    }
    public int getIndex(ArrayList<CurvePoint> path, CurvePoint obj){
        int index = -1;
        for(int i = 0;i < path.size(); i++){
            if(path.get(i) == obj){
                index = i;
            }
        }
        return index;
    }





}

