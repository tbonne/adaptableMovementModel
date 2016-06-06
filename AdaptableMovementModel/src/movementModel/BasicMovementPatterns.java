package movementModel;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.QFunction;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.oomdp.auxiliary.common.SinglePFTF;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.GoalBasedRF;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.singleagent.common.VisualActionObserver;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.EnvironmentServer;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
import burlap.oomdp.visualizer.Visualizer;

import java.awt.*;
import java.util.List;

import movementModel.ExampleGridWorld.DirectionRF;
import movementModel.ExampleGridWorld.DirectionTF;

public class BasicMovementPatterns {
	
	public static final String ATTX = "x";
	public static final String ATTY = "y";
	public static final String ID = "id";
	public static final String DIR = "dir";

	public static final String CLASSAGENT = "agent";
	public static final String CLASSGROUPMATE = "groupMate";
	
	ExampleGridWorld gen;
	Domain domain;
	RewardFunction rf;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	HashableStateFactory hashingFactory;
	Environment env;
	
	public static int timeStep;
	
	public BasicMovementPatterns(){
		
		timeStep=0;
		
		gen = new ExampleGridWorld();
		domain = gen.generateDomain();
		
		State initialState = ExampleGridWorld.getExampleState(domain);
		
		int[] directionsGPS = new int[] {80,80,80,80,80,80,80,70,70,70,70,70,70,70,70,60,60,60,50,50,40,40,30,20,10,0};

		RewardFunction rf = new DirectionRF(directionsGPS);
		TerminalFunction tf = new DirectionTF(directionsGPS.length);
		
		hashingFactory = new SimpleHashableStateFactory(false);//identifierIndependent = True/False ; if false the id's of the objects matter
		
		env = new SimulatedEnvironment(domain, rf, tf, initialState);

		//TerminalExplorer exp = new TerminalExplorer(domain, initialState);
		//exp.explore();

		Visualizer v = gen.getVisualizer();
		//VisualExplorer exp = new VisualExplorer(domain, v, initialState);
		VisualExplorer exp = new VisualExplorer(domain, env, v);

		exp.initGUI();
		
		
	}
	
	public static void main(String[] args) {

		BasicMovementPatterns example = new BasicMovementPatterns();
		String outputPath = "output/";

		//example.BFSExample(outputPath);
		//example.DFSExample(outputPath);
		//example.AStarExample(outputPath);
		//example.valueIterationExample(outputPath);
		//example.QLearningExample(outputPath);
		example.sarsaLearningExample(outputPath);

		//example.experimentAndPlotter();

		example.visualize(outputPath);

	}
	
	/***************************************Learning algorithms ************************************/
	
	public void QLearningExample(String outputPath){
		
		LearningAgent agent = new QLearning(domain, 0.5, hashingFactory, 0., 1.);

		//run learning for 50 episodes
		for(int i = 0; i < 50; i++){
			EpisodeAnalysis ea = agent.runLearningEpisode(env);

			ea.writeToFile(outputPath + "ql_" + i);
			System.out.println(i + ": " + ea.maxTimeStep());

			//reset environment for next learning episode
			timeStep=0;
			env.resetEnvironment();
		}
		
	}
	
	public void sarsaLearningExample(String outputPath){

		LearningAgent agent = new SarsaLam(domain, 0.1, hashingFactory, 0., 0.5, 0.3);

		//run learning for 50 episodes
		for(int i = 0; i < 50; i++){
			EpisodeAnalysis ea = agent.runLearningEpisode(env);

			ea.writeToFile(outputPath + "sarsa_" + i);
			System.out.println(i + ": " + ea.maxTimeStep());

			//reset environment for next learning episode
			timeStep=0;
			env.resetEnvironment();
		}

	}
	
	
	
	/************************ rewards and termination ***********************************/

	public static class DirectionRF implements RewardFunction{

		int[] directionList;
		

		public DirectionRF(int[] dList){
			directionList = dList;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {

			//get direction of agent in next state
			ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
			int direction = agent.getIntValForAttribute(DIR);
			
			System.out.println("RF: step = "+timeStep);
			
			//Correct direction chosen?
			if(directionList[timeStep] == direction){
				return 36.;
			}
			
			return -1;
		}


	}

	public static class DirectionTF implements TerminalFunction{

		int timeStepMax;

		public DirectionTF(int dListSize){
			timeStepMax=dListSize;
		}

		@Override
		public boolean isTerminal(State s) {

			//are they at goal time
			if(timeStep==timeStepMax-1){
				timeStep=0;
				return true;
			}

			timeStep++;
			System.out.println("TF: step = "+timeStep);
			return false;
		}



	}
	
	/******************************** visualize ***************************************/
	
	public void visualize(String outputpath){
		Visualizer v = GridWorldVisualizer.getVisualizer(gen.getMap());
		new EpisodeSequenceVisualizer(v, domain, outputpath);
	}
	
	public void experimentAndPlotter(){

		//different reward function for more interesting results
		//((SimulatedEnvironment)env).setRf(new GoalBasedRF(this.goalCondition, 5.0, -0.1));

		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */
		LearningAgentFactory qLearningFactory = new LearningAgentFactory() {
			@Override
			public String getAgentName() {
				return "Q-Learning";
			}

			@Override
			public LearningAgent generateAgent() {
				return new QLearning(domain, 0.99, hashingFactory, 0.3, 0.1);
			}
		};

		LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {
			@Override
			public String getAgentName() {
				return "SARSA";
			}

			@Override
			public LearningAgent generateAgent() {
				return new SarsaLam(domain, 0.99, hashingFactory, 0.0, 0.1, 1.);
			}
		};

		LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(env, 10, 100, 
												qLearningFactory, sarsaLearningFactory);
		exp.setUpPlottingConfiguration(500, 250, 2, 1000,
				TrialMode.MOSTRECENTANDAVERAGE,
				PerformanceMetric.CUMULATIVESTEPSPEREPISODE,
				PerformanceMetric.AVERAGEEPISODEREWARD);

		exp.startExperiment();
		exp.writeStepAndEpisodeDataToCSV("expData");

	}


}
