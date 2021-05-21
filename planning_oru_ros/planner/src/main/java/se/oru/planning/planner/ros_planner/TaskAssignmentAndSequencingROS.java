package se.oru.planning.planner.ros_planner;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.planning.planning_oru.ai_planning.TaskAssignmentAndSequencing2;

import org.metacsp.utility.logging.MetaCSPLogging;
import org.ros.node.ConnectedNode;

import fr.uga.pddl4j.parser.Exp;

import java.util.logging.Logger;

public class TaskAssignmentAndSequencingROS extends TaskAssignmentAndSequencing2 {
	
	protected ConnectedNode node = null;
	
	
	public TaskAssignmentAndSequencingROS(final ConnectedNode connectedNode) {
		this.node = connectedNode;

	}
	
	
	
}
