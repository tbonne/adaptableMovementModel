package movementModel;

import java.awt.Color;
import java.util.List;

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
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
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
import burlap.tutorials.bd.ExampleGridWorld;
import burlap.tutorials.bpl.BasicBehavior;


public class MoveModel {

	GridWorldDomain gwdg ;
	Domain domain;
	RewardFunction rf;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	HashableStateFactory hashingFactory;
	Environment env;


	public static final String ATTX = "x";
	public static final String ATTY = "y";

	public static final String CLASSAGENT = "agent";
	public static final String CLASSLOCATION = "location";

	public static final String ACTIONNORTH = "north";
	public static final String ACTIONSOUTH = "south";
	public static final String ACTIONEAST = "east";
	public static final String ACTIONWEST = "west";

	public static final String PFAT = "at";

	static int maxTime=1000; //this should be equivalent to the total number of GPS points


	public static void main(String[] args) {

		MoveModel example = new MoveModel();

		String outputPath = "output/"; //directory to record results

		//run example
		example.BFSExample(outputPath);
		//example.DFSExample(outputPath);
		//example.AStarExample(outputPath);
		//example.valueIterationExample(outputPath);
		//example.QLearningExample(outputPath);
		//example.sarsaLearningExample(outputPath);
		
		
		//example.experimentAndPlotter();

		//run the visualizer
		example.visualize(outputPath);
		//valueIterationExample(outputPath);

	}


	public MoveModel(){

		//create the domain
		gwdg = new GridWorldDomain(101, 101);
		//gwdg.setMapToFourRooms();
		domain = gwdg.generateDomain();

		//define the task
		rf = new UniformCostRF();
		tf = new SinglePFTF(domain.getPropFunction(GridWorldDomain.PFATLOCATION));
		goalCondition = new TFGoalCondition(tf);

		//initalize the state of the task
		initialState = GridWorldDomain.getOneAgentNLocationState(domain, 1);
		GridWorldDomain.setAgent(initialState, 50, 50);
		GridWorldDomain.setLocation(initialState, 0, 10, 10);
		hashingFactory = new SimpleHashableStateFactory();

		env = new SimulatedEnvironment(domain, rf, tf, initialState);
		
		
		
		/*
		VisualActionObserver observer = new VisualActionObserver(domain, 
									GridWorldVisualizer.getVisualizer(gwdg.getMap()));
		observer.initGUI();
		env = new EnvironmentServer(env, observer);
		//((SADomain)domain).addActionObserverForAllAction(observer);
		 */
		 
		
	}


	/******************************** reward and terminal functions *************************/

	//Reward function based on the deviation from the actual movement patterns observed
	public static class DirectionalReward implements RewardFunction{

		int goalX;
		int goalY;

		public DirectionalReward(int goalX, int goalY){
			this.goalX = goalX;
			this.goalY = goalY;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {

			//get location of agent in next state
			ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
			int ax = agent.getIntValForAttribute(ATTX);
			int ay = agent.getIntValForAttribute(ATTY);

			//are they at goal location?  (difference in angle)
			if(ax == this.goalX && ay == this.goalY){
				return 100.;
			}

			return -1;
		}
	}

	//changes the terminal state to be the end of the observation period (GPS data)
	public static class TimeTF implements TerminalFunction{
		int timeMax;
		int timeCounter;

		public TimeTF(int max){
			this.timeMax = max;
			this.timeCounter=0;
		}

		@Override
		public boolean isTerminal(State s) {

			//get time of agent in next state
			//ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			//int ax = agent.getIntValForAttribute(ATTX);
			//int ay = agent.getIntValForAttribute(ATTY);
			timeCounter++;


			//are they at goal location?
			if(timeCounter==timeMax){
				return true;
			}

			return false;
		}
	}

	/************************** Planning / Learning methods here**********************************************/

	public void BFSExample(String outputPath){

		DeterministicPlanner planner = new BFS(domain, goalCondition, hashingFactory);
		Policy p = planner.planFromState(initialState);
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "bfs");

	}

	public void DFSExample(String outputPath){

		DeterministicPlanner planner = new DFS(domain, goalCondition, hashingFactory);
		Policy p = planner.planFromState(initialState);
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "dfs");

	}

	public void QLearningExample(String outputPath){

		LearningAgent agent = new QLearning(domain, 0.99, hashingFactory, 0., 1.);

		//run learning for 50 episodes
		for(int i = 0; i < 50; i++){
			EpisodeAnalysis ea = agent.runLearningEpisode(env);

			ea.writeToFile(outputPath + "ql_" + i);
			System.out.println(i + ": " + ea.maxTimeStep());

			//reset environment for next learning episode
			env.resetEnvironment();
		}

	}
	
	public void AStarExample(String outputPath){

		Heuristic mdistHeuristic = new Heuristic() {
			@Override
			public double h(State s) {

				ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
				ObjectInstance location = s.getFirstObjectOfClass(GridWorldDomain.CLASSLOCATION);

				int ax = agent.getIntValForAttribute(GridWorldDomain.ATTX);
				int ay = agent.getIntValForAttribute(GridWorldDomain.ATTY);

				int lx = location.getIntValForAttribute(GridWorldDomain.ATTX);
				int ly = location.getIntValForAttribute(GridWorldDomain.ATTY);

				double mdist = Math.abs(ax-lx) + Math.abs(ay-ly);

				return -mdist;
			}
		};

		DeterministicPlanner planner = new AStar(domain, rf, goalCondition, hashingFactory, mdistHeuristic);
		Policy p = planner.planFromState(initialState);

		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "astar");

	}
	
	public void SarsaLearningExample(String outputPath){
		
		LearningAgent agent = new SarsaLam(domain, 0.99, hashingFactory, 0., 0.5, 0.3);

		//run learning for 50 episodes
		for(int i = 0; i < 50; i++){
			EpisodeAnalysis ea = agent.runLearningEpisode(env);

			ea.writeToFile(outputPath + "sarsa_" + i);
			System.out.println(i + ": " + ea.maxTimeStep());

			//reset environment for next learning episode
			env.resetEnvironment();
		}
		
	}

	/**************************Visualization and output methods here**********************************************/

	public void visualize(String outputpath){
		Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
		new EpisodeSequenceVisualizer(v, domain, outputpath);
	}
	
	public void valueIterationExample(String outputPath){

		Planner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 100);
		Policy p = planner.planFromState(initialState);
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "vi");

		//visualize the value function and policy.
		simpleValueFunctionVis((ValueFunction)planner, p);

	}
	
	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p){

		List<State> allStates = StateReachability.getReachableStates(initialState, 
							(SADomain)domain, hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
						allStates, valueFunction, p);
		gui.initGUI();

	}





}



