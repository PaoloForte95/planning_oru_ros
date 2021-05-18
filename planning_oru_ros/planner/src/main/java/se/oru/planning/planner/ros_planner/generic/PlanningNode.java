package se.oru.planning.planner.ros_planner.generic;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import orunav_msgs.Trigger;
import orunav_msgs.RobotReport;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.ServiceException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordinator.ros_coordinator.ComputeTaskServiceMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission.OPERATION_TYPE;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;

public class PlanningNode extends AbstractNodeMain {

	private List<Integer> robotIDs = null;
	private HashMap<Integer,Boolean> activeRobots = new HashMap<Integer,Boolean>();
	private HashMap<Integer,Pose> initialLocations = new HashMap<Integer,Pose>();
	private ConnectedNode node = null;


	private HashMap<Integer, Double> max_accel = null;
	private HashMap<Integer, Double> max_vel = null;
	private String planningFile = null;
	private boolean repeatMissions = false;
	private String reportTopic = "report";
	private String mapFrameID = "map";
	private ConcurrentHashMap<Integer,Boolean> isAssigned = new ConcurrentHashMap<Integer,Boolean>();
	private ConcurrentHashMap<Integer,Boolean> planningSucceed = new ConcurrentHashMap<Integer,Boolean>();
	private ConcurrentHashMap<Integer,Boolean> canDispatch = new ConcurrentHashMap<Integer,Boolean>();

	public static final String ANSI_BLUE = "\u001B[34m" + "\u001B[107m";
	public static final String ANSI_GREEN = "\u001B[32m" + "\u001B[107m";
	public static final String ANSI_RED = "\u001B[31m" + "\u001B[107m";
	public static final String ANSI_RESET = "\u001B[0m";

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("coordinator");
	}
	
	

	@Override
	public void onStart(ConnectedNode connectedNode) {
		
		this.node = connectedNode;

		while (true) {
			try {
				connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}

		//read parameters from launch file
		readParams();

		System.out.print(ANSI_BLUE + "sus"+ (planningFile != null));
		System.out.println(ANSI_RESET);
		isAssigned.put(1, false);
		// This CancellableLoop will be canceled automatically when the node shuts down.
		node.executeCancellableLoop(new CancellableLoop() {
			
			
			@Override
			protected void setup() {
				
				long origin = TimeUnit.NANOSECONDS.toMillis(node.getCurrentTime().totalNsecs());
				
				
				//Sleep to allow loading of motion prims
				try {
					Thread.sleep(10000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				for (final int robotID : robotIDs) {
					Subscriber<orunav_msgs.RobotReport> subscriberGoalTest = node.newSubscriber("robot"+robotID+"/control/report", orunav_msgs.RobotReport._TYPE);
					subscriberGoalTest.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
						@Override
						public void onNewMessage(orunav_msgs.RobotReport message) {
							geometry_msgs.Pose pose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
							orunav_msgs.PoseSteering dummyTask = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE); 
							geometry_msgs.PoseStamped poseStamp = node.getTopicMessageFactory().newFromType(geometry_msgs.PoseStamped._TYPE);	
							geometry_msgs.Point pos = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
		
							if(message.getStatus() == 1 && !Missions.hasMissions(robotID)){
								//Publisher<orunav_msgs.RobotReport> publisherInit = node.newPublisher("/my_robot"+1+"/goal", orunav_msgs.RobotReport._TYPE);
								//Publisher<geometry_msgs.PoseStamped> publisherInit = node.newPublisher("robot"+1+"/goal", geometry_msgs.PoseStamped._TYPE);
								Pose materialPose = new Pose(8.0,15.0,Math.PI/2);	
								pos.setX(materialPose.getX());
								pos.setY(materialPose.getY());
								pos.setZ(0);
								pose.setPosition(pos);
								dummyTask.setPose(pose);
								
								poseStamp.setPose(pose);
								//Mission m = new Mission(1,"currentPose", pose.toString(), null, materialPose);
								//Missions.enqueueMission(m);
								isAssigned.put(robotID, false);
								
							}
							
							//System.out.print(ANSI_BLUE + "FLAG OUT" + isAssigned.get(1));
							//System.out.println(ANSI_RESET);
							
						}
					});
				}
				
			}

			@Override
			protected void loop() throws InterruptedException {
				
				for (final int robotID : robotIDs) {
			
				//System.out.print(ANSI_BLUE + "FLAG OUT" + isAssigned.get(1));
				//System.out.println(ANSI_RESET);
				if (isAssigned.get(robotID) == false) {
					geometry_msgs.Pose pose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
					orunav_msgs.PoseSteering dummyTask = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE); 
					geometry_msgs.PoseStamped poseStamp = node.getTopicMessageFactory().newFromType(geometry_msgs.PoseStamped._TYPE);	
					orunav_msgs.RobotReport myreport = node.getTopicMessageFactory().newFromType(orunav_msgs.RobotReport._TYPE); 
					geometry_msgs.Point pos = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
					
					Pose materialPose = new Pose(8.0,15.0,Math.PI/2);	
					pos.setX(materialPose.getX());
					pos.setY(materialPose.getY());
					pos.setZ(0);
					pose.setPosition(pos);
					dummyTask.setPose(pose);
					poseStamp.setPose(pose);
					
					pos.setX(materialPose.getX());
					pos.setY(materialPose.getY());
					pos.setZ(0);
					pose.setPosition(pos);
					dummyTask.setPose(pose);
					poseStamp.setPose(pose);
					
					isAssigned.put(robotID, true);
					Publisher<geometry_msgs.PoseStamped> publisherInit = node.newPublisher("/robot"+robotID+"/goal", geometry_msgs.PoseStamped._TYPE);
					myreport.setState(dummyTask);
					publisherInit.publish(poseStamp);
					
					try {
						Thread.sleep(5000);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					}
				}
				// TODO: Fix reading from locations file/goal sequence file

				Thread.sleep(1000);
			}
		});
		System.out.print(ANSI_GREEN + "COORDINATION INITIALIZED!");
		System.out.println(ANSI_RESET);
	}

	@SuppressWarnings("unchecked")
	private void readParams() {
		ParameterTree params = node.getParameterTree();
		
		max_vel = new HashMap<Integer, Double>();
		max_accel = new HashMap<Integer, Double>();

		String robotIDsParamName = "/" + node.getName() + "/robot_ids";

		System.out.print(ANSI_BLUE + "Checking for robot_ids parameter.");
		System.out.println(ANSI_RESET);
		while(!params.has(robotIDsParamName)) {
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}			
		}

		try {	
			robotIDs = (List<Integer>) params.getList(robotIDsParamName);

			System.out.print(ANSI_BLUE + "Got RobotIDs ... ");
			System.out.println(ANSI_RESET);

			for (Integer robotID : robotIDs) {

			


				activeRobots.put(robotID, false);
			}

			System.out.print(ANSI_BLUE + "Checking for active_robot_ids parameter.");
			System.out.println(ANSI_RESET);
	
			ArrayList<Integer> defaultList = new ArrayList<Integer>();
			defaultList.add(-1);
			List<Integer> activeIDs = (List<Integer>) params.getList("/" + node.getName() + "/active_robot_ids", defaultList);
			//If param was not specified, assume all robots are active
			if (activeIDs.contains(-1)) {
				System.out.print(ANSI_BLUE + "Assuming all robots are active since active_robot_ids parameter was not specified.");
				System.out.println(ANSI_RESET);
				activeIDs = new ArrayList<Integer>();
				for (int robotID : robotIDs) {
					activeIDs.add(robotID);
					activeRobots.put(robotID, true);
				}
			}


			planningFile = params.getString("/" + node.getName() + "/planning_file", "NULL");
			if (planningFile.equals("NULL")) planningFile = null;
	
			repeatMissions = params.getBoolean("/" + node.getName() + "/repeat_missions", false);
			this.reportTopic = params.getString("/" + node.getName() + "/report_topic", "report");
			this.mapFrameID = params.getString("/" + node.getName() + "/map_frame_id", "map");

		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.print(ANSI_RED + "== Parameter not found ==");
			System.out.println(ANSI_RESET);
			e.printStackTrace();
		}
	}

//	private void readGoalSequenceFile() {
//		try {
//			Scanner in = new Scanner(new FileReader(goalSequenceFile));
//			while (in.hasNextLine()) {
//				String line = in.nextLine().trim();
//				if (line.length() != 0 && !line.startsWith("#")) {
//					String[] oneline = line.split(" |\t");
//					int robotID = Integer.parseInt(oneline[0]);
//					String goalLocation = oneline[1];
//					Mission m = new Mission(robotID, null, goalLocation, null, Missions.getLocation(goalLocation));
//					Missions.enqueueMission(m);
//					System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>> ADDED MISSION " + m);
//				}
//			}
//			in.close();
//		}
//		catch (FileNotFoundException e) { e.printStackTrace(); }
//	}


}
