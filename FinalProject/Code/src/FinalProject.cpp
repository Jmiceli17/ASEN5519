//============================================================================
// Name        : FinalProject.cpp
// Author      : Joe Miceli
// Version     :
// Copyright   :
// Description : Implementation of SST motion planning algorithm to compute
//			   : optimal spacecraft slew trajectories. Written for final
//			   : project of ASEN5519 course at CU Boulder.
//============================================================================

#include <iostream>
#include <fstream>

#include <ompl/config.h>

#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include <Eigen/Dense>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace en = Eigen;


 bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
 {
     // cast the abstract state type to the type we expect
	 // This is the quaternion part of the state
	 //TODO: Use this for checking exclusion zones
	 const auto *quat = state->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(0);

	 // This is the angular velocity part of the state
	 const auto *ang_vel = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);


     // check validity of state defined by the quaternion and angular velocity
	 bool state_valid = false;
	 double norm = sqrt(pow(ang_vel->values[0],2) + pow(ang_vel->values[1],2) + pow(ang_vel->values[2],2));

	 if (norm < 0.1)		// 5.729 deg/sec
	 {
		 state_valid = true;
	 }

     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
	 return si->satisfiesBounds(state) && (state_valid);
 }

 // Definition of the ODE for the spacecraft model
 // Note: this function must take these arguments
void DynamicSpacecraftODE(const oc::ODESolver::StateType& x, const oc::Control* control, oc::ODESolver::StateType& xdot)
{
	 // Spacecraft mass moment of inertia
	 en::MatrixXf J(3,3);
	 J(0,0) = 0.249;
	 J(0,1) = 0.0;
	 J(0,2) = 0.0;
	 J(1,0) = 0.0;
	 J(1,1) = 0.318;
	 J(1,2) = 0.0;
	 J(2,0) = 0.0;
	 J(2,1) = 0.0;
	 J(2,2) = 0.223;

	 // cast the inputs into what we expect
	 const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;


	 // zero out xdot
	 xdot.resize(x.size(),0);

	 // Non-linear equations of motion
	 xdot[0] = 1.0/2.0 * (x[3]*x[4] - x[2]*x[5] + x[1]*x[6]);
	 xdot[1] = 1.0/2.0 * (x[2]*x[4] + x[3]*x[5] - x[0]*x[6]);
	 xdot[2] = 1.0/2.0 * (-x[1]*x[4] + x[0]*x[5] + x[3]*x[6]);
	 xdot[3] = 1.0/2.0 * (-x[0]*x[4] - x[1]*x[5] - x[2]*x[6]);
	 xdot[4] = 1.0/J(0,0) * (-( J(2,2)-J(1,1) )*x[5]*x[6] + u[0]);
	 xdot[5] = 1.0/J(1,1) * (-( J(0,0)-J(2,2) )*x[4]*x[6] + u[1]);
	 xdot[6] = 1.0/J(2,2) * (-( J(1,1)-J(0,0) )*x[4]*x[5] + u[2]);


}

// Derived class to define the function to be optimized during the planning process
class KineticEnergyObjective : public ob::PathLengthOptimizationObjective
{
public:
	 // Constructor
	 KineticEnergyObjective(const ob::SpaceInformationPtr& si) : ob::PathLengthOptimizationObjective(si)
	 {
	 }

	 ob::Cost stateCost(const ob::State* s) const override
	 {
		 // Quaternion part of the state
		 //const auto *quat = s->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(0);

		 // This is the angular velocity part of the state
		 const auto *ang_vel = s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

		 // Compute the approximate roational kinetic energy
		 double ke = pow(ang_vel->values[0],2) + pow(ang_vel->values[1],2) + pow(ang_vel->values[2],2);

		 return ob::Cost(ke);
	 }
};

// Callback to return the objective function class
ob::OptimizationObjectivePtr getKineticEnergyObjective(const ob::SpaceInformationPtr& si)
{
	return std::make_shared<KineticEnergyObjective>(si);
}


void plan()
{

     // construct the state space we are planning in
	 // in this case, the state space is a composition of SO(3) (quaternions)
	 // and R^3 (angular velocity)

     //// auto space(std::make_shared<ob::SE2StateSpace>());
     auto SO3(std::make_shared<ob::SO3StateSpace>());
     auto R3(std::make_shared<ob::RealVectorStateSpace>(3));

     auto state_space = SO3 + R3;


     // Set the bounds for angular velocity
     ob::RealVectorBounds ang_vel_bounds(3);
     ang_vel_bounds.setLow(-5.0);	// rad
     ang_vel_bounds.setHigh(5.0);	// rad
     R3->setBounds(ang_vel_bounds);

     // create a control space for 3 orthogonally mounted reaction wheels
     auto cspace(std::make_shared<oc::RealVectorControlSpace>(state_space, 3));

     // set the bounds for the control space based on NSS NRWA-T2 reaction wheel
     ob::RealVectorBounds ctrl_bounds(3);
     ctrl_bounds.setLow(-0.1);		// Nm
     ctrl_bounds.setHigh(0.1);		// Nm
     cspace->setBounds(ctrl_bounds);


     // construct an instance of  space information from this control space
     auto si(std::make_shared<oc::SpaceInformation>(state_space, cspace));

     // set state validity checking for this space
     si->setStateValidityChecker(
         [&si](const ob::State *state) { return isStateValid(si.get(), state); });

     // Set the propagation step size
     si->setPropagationStepSize(0.1);

     // set the state propagation routine using the ODESolver to propagate the system
     auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &DynamicSpacecraftODE));
     si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
     // TODO: can add post integration to enforce bounds https://ompl.kavrakilab.org/odeint.html

     si->setup();

     // create a start state
     ob::ScopedState<ob::CompoundStateSpace> start(state_space);

     ob::ScopedState<> q_init(SO3);
     q_init->as<ob::SO3StateSpace::StateType>()->setAxisAngle(0.7071, 0.7071, 0.0, 1.0);	// Initial quaternion in axis-angle representation

     q_init >> start;

     ob::ScopedState<> w_init(R3);
     w_init->as<ob::RealVectorStateSpace::StateType>()->values[0]=0.01;	// Initial x-component of ang vel
     w_init->as<ob::RealVectorStateSpace::StateType>()->values[1]=0.01;	// Initial y-component of ang vel
     w_init->as<ob::RealVectorStateSpace::StateType>()->values[2]=0.01;	// Initial z-component of ang vel

     w_init >> start;

     // create a goal state
     ob::ScopedState<ob::CompoundStateSpace> goal(state_space);
     ob::ScopedState<> q_goal(SO3);
     ob::ScopedState<> w_goal(R3);

     q_goal->as<ob::SO3StateSpace::StateType>()->setIdentity();	// Final quaternion should be an identity (no rotation between body frame and target frame)

     q_goal >> goal;

     // We don't want any angular velocity at the goal
     w_goal->as<ob::RealVectorStateSpace::StateType>()->values[0]=0.0;	// Final x-component of ang vel
     w_goal->as<ob::RealVectorStateSpace::StateType>()->values[1]=0.0;	// Final y-component of ang vel
     w_goal->as<ob::RealVectorStateSpace::StateType>()->values[2]=0.0;	// Final z-component of ang vel

     w_goal >> goal;


     // create a problem instance
     auto pdef(std::make_shared<ob::ProblemDefinition>(si));

     // set the start and goal states and goal threshold
     pdef->setStartAndGoalStates(start, goal, 0.05);


     // Set the optimization objective for this problem
     pdef->setOptimizationObjective(getKineticEnergyObjective(si));

     // create a planner for the defined space, we want to use SST for this problem
     auto planner(std::make_shared<oc::SST>(si));

     // SST configuration
     planner->setPruningRadius(0.1);
     //planner->setSelectionRadius(0.5);
     planner->setSelectionRadius(0.2);


     // set the problem we are trying to solve for the planner
     planner->setProblemDefinition(pdef);

     // perform setup steps for the planner
     planner->setup();


     // print the settings for this space
     si->printSettings(std::cout);

     // print the problem settings
     pdef->print(std::cout);

     // Check that the planner is valid and throw an exception if it's not
     planner->checkValidity();


     // attempt to solve the problem within X seconds of planning time
     ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);


     if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         oc::PathControl path( static_cast<oc::PathControl &>(*pdef->getSolutionPath()) );


    	 std::cout << "Found solution:" << std::endl;

    	 const std::string& outputFile = "SST_PATH.csv";
    	 std::ofstream out(outputFile.c_str());

    	 // Print the path to the screen
    	 path.print(std::cout);

    	 // Log the path to the output file
    	 path.printAsMatrix(out);

    	 // Print the path cost to the screen
    	 std::cout<<"[ PATH COST ]: "<< path.asGeometric().cost(getKineticEnergyObjective(si)).value() <<std::endl;

     }
     else
         std::cout << "No solution found" << std::endl;

     // Clear all planner data
     planner->clear();
}


void planWithSimpleSetup()
{
	 	 // construct the state space we are planning in
		 // in this case, the state space is a composition of SO(3) (quaternions)
		 // and R^3 (angular velocity)

	     //// auto space(std::make_shared<ob::SE2StateSpace>());
	     auto SO3(std::make_shared<ob::SO3StateSpace>());
	     auto R3(std::make_shared<ob::RealVectorStateSpace>(3));

	     auto state_space = SO3 + R3;


	     // Set the bounds for angular velocity
	     ob::RealVectorBounds ang_vel_bounds(3);
	     ang_vel_bounds.setLow(-5.0);	// rad
	     ang_vel_bounds.setHigh(5.0);	// rad
	     R3->setBounds(ang_vel_bounds);


	     // create a control space for 3 orthogonally mounted reaction wheels
	     auto cspace(std::make_shared<oc::RealVectorControlSpace>(state_space, 3));

	     // set the bounds for the control space based on NSS NRWA-T2 reaction wheel
	     ob::RealVectorBounds ctrl_bounds(3);
	     ctrl_bounds.setLow(-0.1);		// Nm
	     ctrl_bounds.setHigh(0.1);		// Nm
	     cspace->setBounds(ctrl_bounds);


	     // define a simple setup class
	     oc::SimpleSetup ss(cspace);

	     // set state validity checking for this space
	     ss.setStateValidityChecker(
	         [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });

	     // set the state propagation routine
	     auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &DynamicSpacecraftODE));
	     ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));


	     // create a start state
		 ob::ScopedState<ob::CompoundStateSpace> start(state_space);
		 ob::ScopedState<> q_init(SO3);
		 q_init->as<ob::SO3StateSpace::StateType>()->setAxisAngle(0.7071, 0.7071, 0.0, 1.0);	// Initial quaternion in axis-angle representation

		 q_init >> start;

		 ob::ScopedState<> w_init(R3);
		 w_init->as<ob::RealVectorStateSpace::StateType>()->values[0]=0.01;	// Initial x-component of ang vel
		 w_init->as<ob::RealVectorStateSpace::StateType>()->values[1]=0.01;	// Initial y-component of ang vel
		 w_init->as<ob::RealVectorStateSpace::StateType>()->values[2]=0.01;	// Initial z-component of ang vel

		 w_init >> start;

		 // create a goal state
		 ob::ScopedState<ob::CompoundStateSpace> goal(state_space);
		 ob::ScopedState<> q_goal(SO3);
		 ob::ScopedState<> w_goal(R3);

		 q_goal->as<ob::SO3StateSpace::StateType>()->setIdentity();	// Final quaternion should be an identity (no rotation between body frame and target frame)

		 q_goal >> goal;

		 // We don't want any angular velocity at the goal
		 w_goal->as<ob::RealVectorStateSpace::StateType>()->values[0]=0.0;	// Final x-component of ang vel
		 w_goal->as<ob::RealVectorStateSpace::StateType>()->values[1]=0.0;	// Final y-component of ang vel
         w_goal->as<ob::RealVectorStateSpace::StateType>()->values[2]=0.0;	// Final z-component of ang vel

		 w_goal >> goal;

		 // set the start and goal states and goal threshold
		 ss.setStartAndGoalStates(start, goal, 0.05);

		 // Set optimization objective
		 ss.setOptimizationObjective(getKineticEnergyObjective(ss.getSpaceInformation()));


		 // Set the planner
		 ss.setPlanner(std::make_shared<oc::SST>(ss.getSpaceInformation()));


	     // attempt to solve the problem within one second of planning time
	     ob::PlannerStatus solved = ss.solve(10.0);


     if (solved)
     {
         std::cout << "Found solution:" << std::endl;

         // print the path to screen
         ss.getSolutionPath().printAsMatrix(std::cout);
		 ss.getSolutionPath().print(std::cout);

     }
     else
         std::cout << "No solution found" << std::endl;

     // Clear planning data
     ss.clear();
}



// Benchmark Post Run event to log a variable we're interested in
// This will be run after each execution of a planner during the benchmarking process
void logPathCost(const ob::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run, oc::SimpleSetup &ss)
{
	 oc::PathControl path(ss.getSolutionPath());
	 double path_cost = path.asGeometric().cost(getKineticEnergyObjective(ss.getSpaceInformation())).value();

	 run["PATH COST REAL"] = std::to_string(path_cost);
	 if (fabs(path_cost)<0.001)
	 {
		 std::cout<< "[WARNING: PATH COST ALARMINGLY SMALL] " << path_cost << std::endl;
	 }

	 // The format of added data is string key, string value pairs,
	 // with the convention that the last word in string key is one of
	 // REAL, INTEGER, BOOLEAN, STRING. (this will be the type of the field
	 // when the log file is processed and saved as a database).
	 // The values are always converted to string.
}

 void benchmark()
 {
	 // construct the state space we are planning in
	 // in this case, the state space is a composition of SO(3) (quaternions)
	 // and R^3 (angular velocity)

	 //// auto space(std::make_shared<ob::SE2StateSpace>());
	 auto SO3(std::make_shared<ob::SO3StateSpace>());
	 auto R3(std::make_shared<ob::RealVectorStateSpace>(3));

	 auto state_space = SO3 + R3;


	 // Set the bounds for angular velocity
	 ob::RealVectorBounds ang_vel_bounds(3);
	 ang_vel_bounds.setLow(-5.0);	// rad
	 ang_vel_bounds.setHigh(5.0);	// rad
	 R3->setBounds(ang_vel_bounds);


	 // create a control space for 3 orthogonally mounted reaction wheels
	 auto cspace(std::make_shared<oc::RealVectorControlSpace>(state_space, 3));

	 // set the bounds for the control space based on NSS NRWA-T2 reaction wheel
	 ob::RealVectorBounds ctrl_bounds(3);
	 ctrl_bounds.setLow(-0.1);		// Nm
	 ctrl_bounds.setHigh(0.1);		// Nm
	 cspace->setBounds(ctrl_bounds);


	 // define a simple setup class
	 oc::SimpleSetup ss(cspace);

	 // set state validity checking for this space
	 ss.setStateValidityChecker(
		 [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });

	 // set the state propagation routine
	 auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &DynamicSpacecraftODE));
	 ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));


	 // create a start state
	 ob::ScopedState<ob::CompoundStateSpace> start(state_space);
	 ob::ScopedState<> q_init(SO3);
	 q_init->as<ob::SO3StateSpace::StateType>()->setAxisAngle(0.7071, 0.7071, 0.0, 1.0);	// Initial quaternion in axis-angle representation

	 q_init >> start;

	 ob::ScopedState<> w_init(R3);
	 w_init->as<ob::RealVectorStateSpace::StateType>()->values[0]=0.01;	// Initial x-component of ang vel
	 w_init->as<ob::RealVectorStateSpace::StateType>()->values[1]=0.01;	// Initial y-component of ang vel
	 w_init->as<ob::RealVectorStateSpace::StateType>()->values[2]=0.01;	// Initial z-component of ang vel

	 w_init >> start;

	 // create a goal state
	 ob::ScopedState<ob::CompoundStateSpace> goal(state_space);
	 ob::ScopedState<> q_goal(SO3);
	 ob::ScopedState<> w_goal(R3);

	 q_goal->as<ob::SO3StateSpace::StateType>()->setIdentity();	// Final quaternion should be an identity (no rotation between body frame and target frame)

	 q_goal >> goal;

	 // We don't want any angular velocity at the goal
	 w_goal->as<ob::RealVectorStateSpace::StateType>()->values[0]=0.0;	// Final x-component of ang vel
	 w_goal->as<ob::RealVectorStateSpace::StateType>()->values[1]=0.0;	// Final y-component of ang vel
	 w_goal->as<ob::RealVectorStateSpace::StateType>()->values[2]=0.0;	// Final z-component of ang vel

	 w_goal >> goal;

	 // set the start and goal states and goal threshold
	 ss.setStartAndGoalStates(start, goal, 0.05);

	 // Set optimization objective
	 ss.setOptimizationObjective(getKineticEnergyObjective(ss.getSpaceInformation()));



	 // create a benchmark object
	 ompl::tools::Benchmark bench(ss, "SST BENCHMARK");


	 // Create pointers to the SST planners were going to configure
	 ob::PlannerPtr SST_DEFAULT(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_1(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_2(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_3(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_4(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_5(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_6(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_7(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_8(new oc::SST(ss.getSpaceInformation()) );
	 ob::PlannerPtr SST_CFG_9(new oc::SST(ss.getSpaceInformation()) );


	 // Give each implementation a unique name
	 SST_DEFAULT->setName("SST_DEFAULT");
	 SST_CFG_1->setName("SST_CONFIG_1");
	 SST_CFG_2->setName("SST_CONFIG_2");
	 SST_CFG_3->setName("SST_CONFIG_3");
	 SST_CFG_4->setName("SST_CONFIG_4");
	 SST_CFG_5->setName("SST_CONFIG_5");
	 SST_CFG_6->setName("SST_CONFIG_6");
	 SST_CFG_7->setName("SST_CONFIG_7");
	 SST_CFG_8->setName("SST_CONFIG_8");
	 SST_CFG_9->setName("SST_CONFIG_9");


	 // Set the parameters for each SST planner and let us know if we are unsuccessful
	 if (! (SST_DEFAULT->params().setParam("pruning_radius", "0.1")&&
			 SST_DEFAULT->params().setParam("selection_radius", "0.2")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 1
	 if (! (SST_CFG_1->params().setParam("pruning_radius", "0.1")&&
			 SST_CFG_1->params().setParam("selection_radius", "0.5")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 2
	 if (! (SST_CFG_2->params().setParam("pruning_radius", "0.2")&&
			 SST_CFG_2->params().setParam("selection_radius", "0.2")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 3
	 if (! (SST_CFG_3->params().setParam("pruning_radius", "0.3")&&
			 SST_CFG_3->params().setParam("selection_radius", "0.2")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 4
	 if (! (SST_CFG_4->params().setParam("pruning_radius", "0.05")&&
			 SST_CFG_4->params().setParam("selection_radius", "0.4")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 5
	 if (! (SST_CFG_5->params().setParam("pruning_radius", "0.01")&&
			 SST_CFG_5->params().setParam("selection_radius", "0.2")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 6
	 if (! (SST_CFG_6->params().setParam("pruning_radius", "0.05")&&
			 SST_CFG_6->params().setParam("selection_radius", "0.5")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 7
	 if (! (SST_CFG_7->params().setParam("pruning_radius", "0.05")&&
			 SST_CFG_7->params().setParam("selection_radius", "0.8")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 8
	 if (! (SST_CFG_8->params().setParam("pruning_radius", "0.05")&&
			 SST_CFG_8->params().setParam("selection_radius", "1.0")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }
	 // SST Configuration 6
	 if (! (SST_CFG_9->params().setParam("pruning_radius", "0.005")&&
			 SST_CFG_9->params().setParam("selection_radius", "0.2")) )
	 {
		 std::cout<<" THIS DIDN'T WORK :( " << std::endl;
	 }

	 // Add each planner to the benchmark object
	 bench.addPlanner(SST_DEFAULT);
	 bench.addPlanner(SST_CFG_1);
	 bench.addPlanner(SST_CFG_2);
	 bench.addPlanner(SST_CFG_3);
	 bench.addPlanner(SST_CFG_4);
	 bench.addPlanner(SST_CFG_5);
	 bench.addPlanner(SST_CFG_6);
	 bench.addPlanner(SST_CFG_7);
	 bench.addPlanner(SST_CFG_8);
	 bench.addPlanner(SST_CFG_9);


	 // Log every solution path cost in a postRunEvent function
	 bench.setPostRunEvent([&](const ob::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run) {
		 logPathCost(planner, run, ss);
	 });

	 // Define the benchmarking parameters, we'll run each planner 100 times for 10.0 seconds
	 // by default, the maxMem attribute is set to 4096 MB
	 ompl::tools::Benchmark::Request request;
	 request.maxTime = 5.0;
	 request.runCount = 100.0;
	 //request.maxMem = 100.0;
	 request.displayProgress = true;

	 // run the benchmarker
	 bench.benchmark(request);

	 // save to timestamped log file (use ompl_benchmark_statistics.py to convert to a database file afterwards)
	 bench.saveResultsToFile();

	 // Clear the planners in the benchmarker
	 bench.clearPlanners();

 }

 int main(int /*argc*/, char ** /*argv*/)
 {
     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

     plan();

     std::cout << std::endl << std::endl;

//     planWithSimpleSetup();

     std::cout << std::endl << std::endl;


     benchmark();

     return 0;
 }
