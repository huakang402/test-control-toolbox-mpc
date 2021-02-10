//
// Created by huakang on 2021/2/9.
//
#include <ct/optcon/optcon.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <test_control_toolbox_mpc/test_mpcConfig.h>
#include <test_control_toolbox_mpc/TestMPC.h>

using namespace ct::core;
using namespace ct::optcon;

int main(int argc, char **argv) {
  // ros
  ros::init(argc, argv, "test_mpc_node");
  ros::NodeHandle nh;
  ros::Publisher state_pub = nh.advertise<test_control_toolbox_mpc::TestMPC>("control_data", 1000);;
  ros::Rate loop_rate(10);

  // mpc
  const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
  const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;

  double w_n = 0.1;
  double zeta = 5.0;
  std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>>
      oscillatorDynamics(
      new ct::core::SecondOrderSystem(w_n, zeta));

  std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>>
      adLinearizer(
      new ct::core::SystemLinearizer<state_dim, control_dim>(oscillatorDynamics));

  std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>>
      intermediateCost(
      new ct::optcon::TermQuadratic<state_dim, control_dim>());
  std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>>
      finalCost(
      new ct::optcon::TermQuadratic<state_dim, control_dim>());
  bool verbose = true;
  intermediateCost->loadConfigFile("/home/huakang/catkin_ws/src/test_control_toolbox/info/mpcCost.info",
                                   "intermediateCost", verbose);
  finalCost->loadConfigFile("/home/huakang/catkin_ws/src/test_control_toolbox/info/mpcCost.info",
                            "finalCost", verbose);

  std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>>
      costFunction(
      new CostFunctionAnalytical<state_dim, control_dim>());
  costFunction->addIntermediateTerm(intermediateCost);
  costFunction->addFinalTerm(finalCost);

  StateVector<state_dim> x0;
  x0.setRandom();

  ct::core::Time timeHorizon = 3.0;

  ContinuousOptConProblem<state_dim, control_dim> optConProblem(
      timeHorizon, x0, oscillatorDynamics, costFunction, adLinearizer);

  NLOptConSettings ilqr_settings;
  ilqr_settings.dt = 0.01;  // the control discretization in [sec]
  ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
  ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
  ilqr_settings.max_iterations = 10;
  ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
  ilqr_settings.lqocp_solver =
      NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
  ilqr_settings.printSummary = true;

  size_t K = ilqr_settings.computeK(timeHorizon);

  FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
  ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
  StateVectorArray<state_dim> x_ref_init(K + 1, x0);
  NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);


  // create an NLOptConSolver instance
  NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);

  // set the initial guess
  iLQR.setInitialGuess(initController);

  // solve the optimal control problem and retrieve the solution
  iLQR.solve();
  ct::core::StateFeedbackController<state_dim, control_dim> initialSolution = iLQR.getSolution();

  // settings for the iLQR instance used in MPC, use the same settings
  NLOptConSettings ilqr_settings_mpc = ilqr_settings;
  // limit the overall number of iLQR iterations (real-time iteration scheme)
  ilqr_settings_mpc.max_iterations = 1;
  // limited the printouts
  ilqr_settings_mpc.printSummary = false;

  // settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
  ct::optcon::mpc_settings mpc_settings;
  mpc_settings.stateForwardIntegration_ = true;
  mpc_settings.postTruncation_ = true;
  mpc_settings.measureDelay_ = true;
  mpc_settings.delayMeasurementMultiplier_ = 1.0;
  mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
  mpc_settings.coldStart_ = false;


  // Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
  MPC<NLOptConSolver<state_dim, control_dim>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);

  // initialize it using the previously computed initial controller
  ilqr_mpc.setInitialGuess(initialSolution);

  auto start_time = std::chrono::high_resolution_clock::now();

  size_t maxNumRuns = 100;

  std::cout << "Starting to run MPC" << std::endl;

  for (size_t i = 0; i < maxNumRuns; i++) {
    // let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
    if (i > 0)
      x0 = 0.1 * StateVector<state_dim>::Random();

    // time which has passed since start of MPC
    auto current_time = std::chrono::high_resolution_clock::now();
    ct::core::Time t =
        1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

    // prepare mpc iteration
    ilqr_mpc.prepareIteration(t);

    // new optimal policy
    ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;

    // TODO publish feedback data
    test_control_toolbox_mpc::TestMPC data;

    // timestamp of the new optimal policy
    ct::core::Time ts_newPolicy;

    current_time = std::chrono::high_resolution_clock::now();
    t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
    bool success = ilqr_mpc.finishIteration(x0, t, newPolicy, ts_newPolicy);

    // break the loop in case the time horizon is reached or solve() failed
    if (ilqr_mpc.timeHorizonReached() | !success)
      break;
  }

  // the summary contains some statistical data about time delays, etc.
  ilqr_mpc.printMpcSummary();
}
