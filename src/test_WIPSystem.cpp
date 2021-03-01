//
// Created by huakang on 2021/2/18.
//

#include <ct/optcon/optcon.h>
#include <ros/ros.h>
#include <test_control_toolbox_mpc/TestWIPSystem.h>

using namespace ct::core;
using namespace ct::optcon;

int main(int argc, char **argv) {
  // ros
  ros::init(argc, argv, "test_wip_node");
  ros::NodeHandle nh;
  ros::Publisher state_pub = nh.advertise<test_control_toolbox_mpc::TestWIPSystem>("WIPSystem_data", 1000);;
  ros::Rate loop_rate(10);

  // set system
  const size_t STATE_DIM = 5;
  const size_t CONTROL_DIM = 2;
  StateMatrix<STATE_DIM> A_continuous;
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B_continuous;
  double t1{}, t2{}, t3{}, t4{}, t5{};
  t1 = nh.param("t1", 78.482);
  t2 = nh.param("t2", -34.592);
  t3 = nh.param("t3", -0.396);
  t4 = nh.param("t4", 1.737);
  t5 = nh.param("t5", 67.667);

  A_continuous << 0, 1, 0, 0, 0,
      t1, 0, 0, 0, 0,
      t2, 0, 0, 1, 0,
      0, 0, 0, 0, 1,
      0, 0, 0, 0, 0;
  B_continuous << 0, 0,
      t3, 0,
      t4, 0,
      0, 0,
      0, t5;
  StateMatrix<STATE_DIM> &A = A_continuous;
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &B = B_continuous;

  std::string costFunctionFile = "/home/huakang/test_mpc_ws/src/test_control_toolbox_mpc/info/mpcCost.info";

  // Set initial conditions
  StateVector<STATE_DIM> x0;
  ct::core::loadMatrix(costFunctionFile, "x0", x0);
  // set initial control
  ControlVector<CONTROL_DIM> u0;
  ct::core::loadMatrix(costFunctionFile, "u0", u0);
  // set FeedbackMatrix
  FeedbackMatrix<STATE_DIM, CONTROL_DIM> FB_Matrix;
  ct::core::loadMatrix(costFunctionFile, "K", FB_Matrix);

  std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>>
      wipSystem(new ct::core::LTISystem<STATE_DIM, CONTROL_DIM>(A, B));
  std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>>
      adLinearizer(new ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>(wipSystem));
  std::shared_ptr<ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>>
      intermediateCost(new ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>());
  std::shared_ptr<ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>>
      finalCost(new ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>());
  bool verbose = true;
  intermediateCost->loadConfigFile(costFunctionFile, "intermediateCost", verbose);
  finalCost->loadConfigFile(costFunctionFile, "finalCost", verbose);

  std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>>
      costFunction(new CostFunctionAnalytical<STATE_DIM, CONTROL_DIM>());
  costFunction->addIntermediateTerm(intermediateCost);
  costFunction->addFinalTerm(finalCost);

//  // set up the box constraints for the control input

//  Eigen::VectorXi sp_control(CONTROL_DIM);
//  sp_control << 1, 1;
//  Eigen::VectorXd u_lb(CONTROL_DIM);
//  Eigen::VectorXd u_ub(CONTROL_DIM);
//  u_lb.setConstant(-35.0);
//  u_ub = -u_lb;
//
//  // constraint terms
//  std::shared_ptr<ControlInputConstraint<STATE_DIM, CONTROL_DIM>> controlInputBound(
//      new ControlInputConstraint<STATE_DIM, CONTROL_DIM>(u_lb, u_ub, sp_control));
//  controlInputBound->setName("ControlInputBound");
//
//  // input box constraint constraint container
//  std::shared_ptr<ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM>> inputBoxConstraints(
//      new ct::optcon::ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM>());
//
//  // add and initialize constraint terms
//  inputBoxConstraints->addIntermediateConstraint(controlInputBound, verbose);
//  inputBoxConstraints->initialize();
//
//  // set up the box constraints for the states
//  Eigen::VectorXi sp_state(STATE_DIM);
//  sp_state << 1, 1, 1, 1, 1;
//  Eigen::VectorXd x_lb(STATE_DIM);
//  Eigen::VectorXd x_ub(STATE_DIM);
//  x_lb << -5, -1, 10, -5, -1;
//  x_ub << 5, 1, 20, 5, 1;
//
//  // constraint terms
//  std::shared_ptr<StateConstraint<STATE_DIM, CONTROL_DIM>>
//      stateBound(new StateConstraint<STATE_DIM, CONTROL_DIM>(x_lb, x_ub, sp_state));
//  stateBound->setName("StateBound");
//
//  // input box constraint constraint container
//  std::shared_ptr<ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM>> stateBoxConstraints(
//      new ct::optcon::ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM>());
//
//  // add and initialize constraint terms
//  stateBoxConstraints->addIntermediateConstraint(stateBound, verbose);
//  stateBoxConstraints->initialize();

  ct::core::Time timeHorizon;
  ct::core::loadScalar(costFunctionFile, "timeHorizon", timeHorizon);

  ContinuousOptConProblem<STATE_DIM, CONTROL_DIM>
      optConProblem(timeHorizon, x0, wipSystem, costFunction, adLinearizer);

//  // add the box constraints to the optimal control problem
//  optConProblem.setInputBoxConstraints(inputBoxConstraints);
//  optConProblem.setStateBoxConstraints(stateBoxConstraints);

  NLOptConSettings ilqr_settings;
  ilqr_settings.dt = nh.param("dt", 0.01);  // the control discretization in [sec]
  ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
  ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
  ilqr_settings.max_iterations = nh.param("ilqr_max_iterations", 10);
  ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
  ilqr_settings.lqocp_solver =
      NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
  ilqr_settings.printSummary = true;

  size_t K = ilqr_settings.computeK(timeHorizon);

  FeedbackArray<STATE_DIM, CONTROL_DIM> u0_fb(K, -FB_Matrix);
  ControlVectorArray<CONTROL_DIM> u0_ff(K, u0);
  StateVectorArray<STATE_DIM> x_ref_init(K + 1, x0);
  NLOptConSolver<STATE_DIM, CONTROL_DIM>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

  // create an NLOptConSolver instance
  NLOptConSolver<STATE_DIM, CONTROL_DIM> iLQR(optConProblem, ilqr_settings);

  // set the initial guess
  iLQR.setInitialGuess(initController);

  // solve the optimal control problem and retrieve the solution
  iLQR.solve();
  ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> initialSolution = iLQR.getSolution();

  StateVector<STATE_DIM> x_0;
  int col = nh.param("col", 100);
  x_0 = initialSolution.x_ref()[col];
  std::cout << "size: " << initialSolution.x_ref().size() << std::endl;
  std::cout << "x0: " << x0 << std::endl;
  std::cout << "x_0: " << x_0 << std::endl;

  // publish data of iLQR
//  for (u_int i = 0; i < initialSolution.x_ref().size(); i++) {
//    test_control_toolbox_mpc::TestWIPSystem data;
//    data.x_alpha = initialSolution.x_ref()[i](0);
//    data.x_alpha_dot = initialSolution.x_ref()[i](1);
//    data.x_vel = initialSolution.x_ref()[i](2);
//    data.x_theta = initialSolution.x_ref()[i](3);
//    data.x_theta_dot = initialSolution.x_ref()[i](4);
//    data.u_vel = initialSolution.uff()[i](0);
//    data.u_w = initialSolution.uff()[i](1);
//    state_pub.publish(data);
//  }

  // settings for the iLQR instance used in MPC, use the same settings
  NLOptConSettings ilqr_settings_mpc = ilqr_settings;
  // limit the overall number of iLQR iterations (real-time iteration scheme)
  ilqr_settings_mpc.max_iterations = nh.param("mpc_max_iterations", 10);
  // limited the printouts
  ilqr_settings_mpc.printSummary = false;

  // settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
  ct::optcon::mpc_settings mpc_settings;
  mpc_settings.stateForwardIntegration_ = true;
  mpc_settings.postTruncation_ = true;
  mpc_settings.measureDelay_ = true;
  mpc_settings.delayMeasurementMultiplier_ = nh.param("delayMeasurementMultiplier", 1.0);
  mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
  mpc_settings.coldStart_ = false;

  // Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
  MPC<NLOptConSolver<STATE_DIM, CONTROL_DIM>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);

  // initialize it using the previously computed initial controller
  ilqr_mpc.setInitialGuess(initialSolution);

  auto start_time = std::chrono::high_resolution_clock::now();

  size_t maxNumRuns = nh.param("maxNumRuns", 100);

  std::cout << "Starting to run MPC" << std::endl;
  for (size_t i = 0; i < maxNumRuns; i++) {
    // let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
//    if (i > 0)
//      x0 = 0.1 * StateVector<STATE_DIM>::Random();

    // time which has passed since start of MPC
    auto current_time = std::chrono::high_resolution_clock::now();
    ct::core::Time t =
        1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

    // prepare mpc iteration
    ilqr_mpc.prepareIteration(t);

    // new optimal policy
    ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> newPolicy;

    // timestamp of the new optimal policy
    ct::core::Time ts_newPolicy;

    current_time = std::chrono::high_resolution_clock::now();
    t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
    bool success = ilqr_mpc.finishIteration(x0, t, newPolicy, ts_newPolicy);

    // publish data of MPC
    ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> solution = iLQR.getSolution();
    for (u_int j = 0; j < 500; j++) {
      test_control_toolbox_mpc::TestWIPSystem data;
      data.x_alpha = solution.x_ref()[j](0);
      data.x_alpha_dot = solution.x_ref()[j](1);
      data.x_vel = solution.x_ref()[j](2);
      data.x_theta = solution.x_ref()[j](3);
      data.x_theta_dot = solution.x_ref()[j](4);
      data.u_vel = solution.uff()[j](0);
      data.u_w = solution.uff()[j](1);
      state_pub.publish(data);
    }

    // break the loop in case the time horizon is reached or solve() failed
    if (ilqr_mpc.timeHorizonReached() | !success)
      break;
  }

  // the summary contains some statistical data about time delays, etc.
  ilqr_mpc.printMpcSummary();
}