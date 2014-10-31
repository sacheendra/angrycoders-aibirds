/*****************************************************************************
 ** ANGRYBIRDS AI AGENT FRAMEWORK
 ** Copyright (c) 2014, XiaoYu (Gary) Ge, Stephen Gould, Jochen Renz
 **  Sahan Abeyasinghe,Jim Keys,  Andrew Wang, Peng Zhang
 ** All rights reserved.
 **This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. 
 **To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ 
 *or send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *****************************************************************************/
package ab.demo;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.ArrayList;


import ab.demo.other.ActionRobot;
import ab.demo.other.Shot;
import ab.planner.TrajectoryPlanner;
import ab.utils.StateUtil;
import ab.vision.ABObject;
import ab.vision.ABType;
import ab.vision.GameStateExtractor.GameState;
import ab.vision.Vision;

public class NaiveAgent implements Runnable {

	private ActionRobot aRobot;
	private Random randomGenerator;
	public int currentLevel = 1;
	public static int time_limit = 12;
	private Map<Integer,Integer> scores = new LinkedHashMap<Integer,Integer>();
	TrajectoryPlanner tp;
	private boolean firstShot;
	private Point prevTarget;
    int[] restartCountArray=new int[30];

	// a standalone implementation of the Naive Agent
	public NaiveAgent() {
		
		aRobot = new ActionRobot();
		tp = new TrajectoryPlanner();
		prevTarget = null;
		firstShot = true;
		randomGenerator = new Random();
		// --- go to the Poached Eggs episode level selection page ---
		ActionRobot.GoFromMainMenuToLevelSelection();
        for(int i=0;i<30;i++)
        {
            restartCountArray[i]=0;
        }
        for(int i=0;i<22;i++)
        {
            scores.put(i,0);
        }
	}

	
	// run the client
	public void run() {

		aRobot.loadLevel(currentLevel);
		while (true) {
			GameState state = solve();
			if (state == GameState.WON) {
                restartCountArray[currentLevel]++;
				try {
					Thread.sleep(3000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				int score = StateUtil.getScore(ActionRobot.proxy);
				if(!scores.containsKey(currentLevel))
					scores.put(currentLevel, score);
				else
				{
					if(scores.get(currentLevel) < score)
						scores.put(currentLevel, score);
				}
				int totalScore = 0;
				for(Integer key: scores.keySet()){

					totalScore += scores.get(key);
					System.out.println(" Level " + key
							+ " Score: " + scores.get(key) + " ");
				}
				System.out.println("Total Score: " + totalScore);
				aRobot.loadLevel(++currentLevel);
				// make a new trajectory planner whenever a new level is entered
				tp = new TrajectoryPlanner();

				// first shot on this level, try high shot first
				firstShot = true;
			} else if (state == GameState.LOST) {
                restartCountArray[currentLevel]++;
				System.out.println("Restart");

				aRobot.restartLevel();
			} else if (state == GameState.LEVEL_SELECTION) {
				System.out
				.println("Unexpected level selection page, go to the last current level : "
						+ currentLevel);
				aRobot.loadLevel(currentLevel);
			} else if (state == GameState.MAIN_MENU) {
				System.out
				.println("Unexpected main menu page, go to the last current level : "
						+ currentLevel);
				ActionRobot.GoFromMainMenuToLevelSelection();
				aRobot.loadLevel(currentLevel);
			} else if (state == GameState.EPISODE_MENU) {
				System.out
				.println("Unexpected episode menu page, go to the last current level : "
						+ currentLevel);
				ActionRobot.GoFromMainMenuToLevelSelection();
				aRobot.loadLevel(currentLevel);
			}

		}

	}

	private double distance(Point p1, Point p2) {
		return Math
				.sqrt((double) ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y)
						* (p1.y - p2.y)));
	}

	public GameState solve()
	{

		// capture Image
		BufferedImage screenshot = ActionRobot.doScreenShot();

		// process image
		Vision vision = new Vision(screenshot);

		// find the slingshot
		Rectangle sling = vision.findSlingshotMBR();

		// confirm the slingshot
		while (sling == null && aRobot.getState() == GameState.PLAYING) {
			System.out
			.println("No slingshot detected. Please remove pop up or zoom out");
			ActionRobot.fullyZoomOut();
			screenshot = ActionRobot.doScreenShot();
			vision = new Vision(screenshot);
			sling = vision.findSlingshotMBR();
		}
        // get all the pigs
 		List<ABObject> pigs = vision.findPigsMBR();
        List<ABObject> Blocks = vision.findBlocksMBR();
		GameState state = aRobot.getState();

		// if there is a sling, then play, otherwise just skip.
		if (sling != null) {
            if (!pigs.isEmpty()) {

                Point releasePoint = null;
                Shot shot = new Shot();
                int dx,dy;
                {
                    // random pick up a pig
                    ArrayList<Point> release = new ArrayList<Point>();
                    ArrayList<List<Point>> traj= new ArrayList<List<Point>>();
                    Point tpt=null;
                    if((restartCountArray[currentLevel]%3)+1==1) {


                        for (int i = 0; i < pigs.size(); i++) {
                            Point _tpt = pigs.get(i).getCenter();
                            ArrayList<Point> pts = tp.estimateLaunchPoint(sling, _tpt);
                            Point Max = pts.get(0);
                            double maxA = Double.MIN_VALUE;
                            for (int j = 0; j < pts.size(); j++) {
                                if (tp.getVelocity(tp.getReleaseAngle(sling, pts.get(j))) > maxA) {
                                    maxA = tp.getVelocity(tp.getReleaseAngle(sling, pts.get(j)));
                                    Max = pts.get(j);
                                }
                            }
                            release.add(Max);
                            traj.add(tp.predictTrajectory(sling, Max));
                        }
                        int f = 0;
                        int max = Integer.MIN_VALUE;
//                    System.out.println("***+++++++++++++===*");

                        for (int i = 0; i < traj.size(); i++) {
                            int count = 0;
                            ArrayList<ABObject> pigTemp = new ArrayList<ABObject>();
                            for (int temp = 0; temp < pigs.size(); temp++) {
                                pigTemp.add(pigs.get(temp));
                            }
                            for (int j = 0; j < traj.get(i).size(); j++) {
                                for (int p = 0; p < pigTemp.size(); p++) {
                                    if (distance(traj.get(i).get(j), pigTemp.get(p).getCenter()) < 10) {
                                        count++;
                                        pigTemp.remove(p);
                                    }
                                }
                            }
                            System.out.println(count);
                            if (max < count) {
                                max = count;
                                f = i;
                            }
                        }


                        releasePoint = release.get(f);
                        tpt = pigs.get(f).getCenter();
                    }
                    else if((restartCountArray[currentLevel]%3)+1==3)
                    {

                        for (int i = 0; i < pigs.size(); i++) {
                            Point _tpt = pigs.get(i).getCenter();
                            ArrayList<Point> pts = tp.estimateLaunchPoint(sling, _tpt);
                            Point min = pts.get(0);
                            double minA = Double.MAX_VALUE;
                            for (int j = 0; j < pts.size(); j++) {
                                if (tp.getVelocity(tp.getReleaseAngle(sling, pts.get(j))) < minA) {
                                    minA = tp.getVelocity(tp.getReleaseAngle(sling, pts.get(j)));
                                    min = pts.get(j);
                                }
                            }
                            release.add(min);
                            traj.add(tp.predictTrajectory(sling, min));
                        }

                        int f = 0;
                        int max = Integer.MIN_VALUE;
//                    System.out.println("***+++++++++++++===*");

                        for (int i = 0; i < traj.size(); i++) {
                            int count = 0;
                            ArrayList<ABObject> pigTemp = new ArrayList<ABObject>();
                            for (int temp = 0; temp < pigs.size(); temp++) {
                                pigTemp.add(pigs.get(temp));
                            }
                            for (int j = 0; j < traj.get(i).size(); j++) {
                                for (int p = 0; p < pigTemp.size(); p++) {
                                    if (distance(traj.get(i).get(j), pigTemp.get(p).getCenter()) < 10) {
                                        count++;
                                        pigTemp.remove(p);
                                    }
                                }
                            }
                            System.out.println(count);
                            if (max < count) {
                                max = count;
                                f = i;
                            }
                        }


                        releasePoint = release.get(f);
                        tpt = pigs.get(f).getCenter();
                    }
                    /*else if(restartCountArray[currentLevel]+1==4) {
                        ABObject pig = pigs.get(randomGenerator.nextInt(pigs.size()));
                        tpt = pig.getCenter();
                        ArrayList<Point> pts = tp.estimateLaunchPoint(sling, tpt);

                        if (pts.size() == 2) {
                            if (randomGenerator.nextInt(6) == 0)
                                releasePoint = pts.get(1);
                            else
                                releasePoint = pts.get(0);
                        } else if (pts.size() == 1)
                            releasePoint = pts.get(0);
                        else if (pts.isEmpty()) {
                            System.out.println("No release point found for the target");
                            System.out.println("Try a shot with 45 degree");
                            releasePoint = tp.findReleasePoint(sling, Math.PI / 4);
                        }
                    }*/
                    else if((restartCountArray[currentLevel]%3)+1==2) {

                        ArrayList<data> pts=new ArrayList<data>(0);
                        for( ABObject pi : pigs )
                        {
                            ArrayList<Point> temp=tp.estimateLaunchPoint(sling, pi.getCenter());
                            if(temp.size()>0)
                            {
                                for(Point p:temp)
                                {
                                    data t=new data();
                                    t.release=p;
                                    t.target=pi.getCenter();
                                    pts.add(t);
                                }
                            }


                        }



                            releasePoint = pts.get(1).release;
                            int weight=Integer.MAX_VALUE;
                            List<ABObject> blocks=vision.findBlocksRealShape();
                            List<ABObject> hill=vision.findHills();
                            for(data p: pts)
                            {
                                List<Point> trajectory=  tp.predictTrajectory(sling,p.release);
                                int w=0;
                                for(ABObject blk:blocks)
                                {
                                    int add=0;
                                    ABType typ=blk.getType();
                                    if(typ.id==10)
                                        add=10;
                                    else if(typ.id==11)
                                        add=40;
                                    else if(typ.id==12)
                                        add=200;

                                    for(Point pt: trajectory)
                                    {
                                        if(blk.contains(pt))
                                        {
                                            w+=add;
                                            break;
                                        }
                                    }


                                }
                                boolean terrain=false;
                                for(Point pt:trajectory)
                                {
                                    for(ABObject blk:blocks)
                                    {
                                        if(blk.contains(pt))
                                        {
                                            break;
                                        }
                                    }

                                    for(ABObject pi: pigs)
                                    {
                                        if(pi.contains(pt))
                                            break;
                                    }

                                    for(ABObject h:hill)
                                    {
                                        if(h.contains(pt))
                                        {
                                            terrain=true;
                                            break;
                                        }
                                    }

                                }
                                /*for(ABObject hil:hill)
                                {
                                    for(Point pt: trajectory)
                                    {
                                        if(hil.contains(pt))
                                        {
                                            w+=200000;
                                            break;
                                        }
                                    }
                                }
                                   */
                                if(terrain)
                                {
                                    w+=200000;
                                }
                                if(w<=weight)
                                {
                                    weight=w;
                                    releasePoint=p.release;
                                    tpt=p.target;
                                }

                                System.out.println("Weight "+w+" "+p.target.getX()+" "+p.target.getY());
                            }
                       /* for( ABObject pi : pigs )
                        {
                            ArrayList<Point> temp=tp.estimateLaunchPoint(sling, pi.getCenter());
                            if(temp!=null) {
                               for(Point p:temp)
                               {
                                   if(p==releasePoint)
                                       tpt=pi.getCenter();
                               }
                            }
                        }*/

                    }
                    // Get the reference point
                    Point refPoint = tp.getReferencePoint(sling);

                    //Calculate the tapping time according the bird type
                    if (releasePoint != null) {
                        double releaseAngle = tp.getReleaseAngle(sling,
                                releasePoint);
                        System.out.println("Release Point: " + releasePoint);
                        System.out.println("Release Angle: "
                                + Math.toDegrees(releaseAngle));
                        int tapInterval = 0;
                        switch (aRobot.getBirdTypeOnSling())
                        {

                            case RedBird:
                                tapInterval = 65; break;               // start of trajectory
                            case YellowBird:
                                tapInterval = 65 + randomGenerator.nextInt(25);break; // 65-90% of the way
                            case WhiteBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlackBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlueBird:
                                tapInterval =  65 + randomGenerator.nextInt(20);break; // 65-85% of the way
                            default:
                                tapInterval =  60;
                        }


                        int tapTime;
                        tapTime= tp.getTapTimeB(sling, releasePoint, tpt, tapInterval);
                        dx = (int)releasePoint.getX() - refPoint.x;
                        dy = (int)releasePoint.getY() - refPoint.y;


                        shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);
                    }
                    else
                    {
                        System.err.println("No Release Point Found");
                        return state;
                    }
                }

                // check whether the slingshot is changed. the change of the slingshot indicates a change in the scale.
                {
                    ActionRobot.fullyZoomOut();
                    screenshot = ActionRobot.doScreenShot();
                    vision = new Vision(screenshot);
                    Rectangle _sling = vision.findSlingshotMBR();
                    if(_sling != null)
                    {
                        double scale_diff = Math.pow((sling.width - _sling.width),2) +  Math.pow((sling.height - _sling.height),2);
                        if(scale_diff < 25)
                        {
                            if(dx < 0)
                            {
                                aRobot.cshoot(shot);
                                state = aRobot.getState();
                                if ( state == GameState.PLAYING )
                                {
                                    screenshot = ActionRobot.doScreenShot();
                                    vision = new Vision(screenshot);
                                    List<Point> traj = vision.findTrajPoints();
                                    tp.adjustTrajectory(traj, sling, releasePoint);
                                    firstShot = false;
                                }
                            }
                        }
                        else
                            System.out.println("Scale is changed, can not execute the shot, will re-segement the image");
                    }
                    else
                        System.out.println("no sling detected, can not execute the shot, will re-segement the image");
                }

            }

		}
		return state;
	}

	public static void main(String args[]) {

		NaiveAgent na = new NaiveAgent();
		if (args.length > 0)
			na.currentLevel = Integer.parseInt(args[0]);
		na.run();

	}

    class data{
        public Point release;
        public Point target;
    }
}
