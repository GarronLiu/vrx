#pragma once

#include <ct/core/core.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace std;

namespace usv
{
namespace tpl
{
template <typename SCALAR>
struct USVParameters
{
  // For safety, these parameters cannot be modified
  SCALAR Xu_;
  SCALAR Xu_absu_;
  SCALAR Xvr_;
  SCALAR Xrr_;
  SCALAR Yv_;
  SCALAR Yr_;
  SCALAR Yv_absv_;
  SCALAR Yr_absr_;
  SCALAR Yuv_;
  SCALAR Yur_;
  SCALAR Yabsv_r_;
  SCALAR Yv_absr_;
  SCALAR Mv_;
  SCALAR Mr_;
  SCALAR Mv_absv_;
  SCALAR Mr_absr_;
  SCALAR Muv_;
  SCALAR Mur_;
  SCALAR Mabsv_r_;
  SCALAR Mv_absr_;
  // WAM-V rigid-body and VRX aft-thruster parameters.
  SCALAR mass_;
  SCALAR yawInertia_;
  SCALAR leftThrusterY_;
  SCALAR rightThrusterY_;
  SCALAR maxThrustCmd_;
  SCALAR maxForceFwd_;
  SCALAR maxForceRev_;
  SCALAR thrustCmdThreshold_;

  // Generalized logistic function used by the VRX thrust plugin.
  SCALAR forwardGlfA_;
  SCALAR forwardGlfK_;
  SCALAR forwardGlfB_;
  SCALAR forwardGlfV_;
  SCALAR forwardGlfC_;
  SCALAR forwardGlfM_;
  SCALAR reverseGlfA_;
  SCALAR reverseGlfK_;
  SCALAR reverseGlfB_;
  SCALAR reverseGlfV_;
  SCALAR reverseGlfC_;
  SCALAR reverseGlfM_;

  SCALAR minInput_;
  SCALAR maxInput_;
  SCALAR maxInputRate_;
  SCALAR maxSurgeSpeed_;
  SCALAR minSurgeSpeed_;
  SCALAR maxAngularSpeed_;
};

template <typename T>
static inline void printValue(std::ostream& stream, const T& value, const std::string& name, bool updated = true,
                              long printWidth = 80)
{
  const std::string nameString = " #### '" + name + "'";
  stream << nameString;

  printWidth = std::max<long>(printWidth, nameString.size() + 15);
  stream.width(printWidth - nameString.size());
  const char fill = stream.fill('.');

  if (updated)
  {
    stream << value << '\n';
  }
  else
  {
    stream << value << " (default)\n";
  }

  stream.fill(fill);
}

template <typename T>
inline void loadPtreeValue(const boost::property_tree::ptree& pt, T& value, const std::string& name, bool verbose,
                           long printWidth = 80)
{
  bool updated = true;

  try
  {
    value = pt.get<T>(name);
  }
  catch (const boost::property_tree::ptree_bad_path&)
  {
    updated = false;
  }

  if (verbose)
  {
    const std::string nameString = name.substr(name.find_last_of('.') + 1);
    printValue(std::cout, value, nameString, updated, printWidth);
  }
}

template <typename SCALAR>
inline USVParameters<SCALAR> loadSettings(const std::string& filename, const std::string& fieldName = "USVParameters",
                                          bool verbose = true)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  USVParameters<SCALAR> settings;

  if (verbose)
  {
    std::cout << "\n #### USV Parameters:";
    std::cout << "\n #### =============================================================================\n";
  }

  loadPtreeValue(pt, settings.Xu_, fieldName + ".Xu", verbose);
  loadPtreeValue(pt, settings.Xu_absu_, fieldName + ".Xu_absu", verbose);
  loadPtreeValue(pt, settings.Xvr_, fieldName + ".Xvr", verbose);
  loadPtreeValue(pt, settings.Xrr_, fieldName + ".Xrr", verbose);

  loadPtreeValue(pt, settings.Yv_, fieldName + ".Yv", verbose);
  loadPtreeValue(pt, settings.Yr_, fieldName + ".Yr", verbose);
  loadPtreeValue(pt, settings.Yv_absv_, fieldName + ".Yv_absv", verbose);
  loadPtreeValue(pt, settings.Yr_absr_, fieldName + ".Yr_absr", verbose);
  loadPtreeValue(pt, settings.Yuv_, fieldName + ".Yuv", verbose);
  loadPtreeValue(pt, settings.Yur_, fieldName + ".Yur", verbose);
  loadPtreeValue(pt, settings.Yabsv_r_, fieldName + ".Yabsv_r", verbose);
  loadPtreeValue(pt, settings.Yv_absr_, fieldName + ".Yv_absr", verbose);

  loadPtreeValue(pt, settings.Mv_, fieldName + ".Mv", verbose);
  loadPtreeValue(pt, settings.Mr_, fieldName + ".Mr", verbose);
  loadPtreeValue(pt, settings.Mv_absv_, fieldName + ".Mv_absv", verbose);
  loadPtreeValue(pt, settings.Mr_absr_, fieldName + ".Mr_absr", verbose);
  loadPtreeValue(pt, settings.Muv_, fieldName + ".Muv", verbose);
  loadPtreeValue(pt, settings.Mur_, fieldName + ".Mur", verbose);
  loadPtreeValue(pt, settings.Mabsv_r_, fieldName + ".Mabsv_r", verbose);
  loadPtreeValue(pt, settings.Mv_absr_, fieldName + ".Mv_absr", verbose);

  loadPtreeValue(pt, settings.mass_, fieldName + ".mass", verbose);
  loadPtreeValue(pt, settings.yawInertia_, fieldName + ".yawInertia", verbose);
  loadPtreeValue(pt, settings.leftThrusterY_, fieldName + ".leftThrusterY", verbose);
  loadPtreeValue(pt, settings.rightThrusterY_, fieldName + ".rightThrusterY", verbose);
  loadPtreeValue(pt, settings.maxThrustCmd_, fieldName + ".maxThrustCmd", verbose);
  loadPtreeValue(pt, settings.maxForceFwd_, fieldName + ".maxForceFwd", verbose);
  loadPtreeValue(pt, settings.maxForceRev_, fieldName + ".maxForceRev", verbose);
  loadPtreeValue(pt, settings.thrustCmdThreshold_, fieldName + ".thrustCmdThreshold", verbose);

  loadPtreeValue(pt, settings.forwardGlfA_, fieldName + ".forwardGlfA", verbose);
  loadPtreeValue(pt, settings.forwardGlfK_, fieldName + ".forwardGlfK", verbose);
  loadPtreeValue(pt, settings.forwardGlfB_, fieldName + ".forwardGlfB", verbose);
  loadPtreeValue(pt, settings.forwardGlfV_, fieldName + ".forwardGlfV", verbose);
  loadPtreeValue(pt, settings.forwardGlfC_, fieldName + ".forwardGlfC", verbose);
  loadPtreeValue(pt, settings.forwardGlfM_, fieldName + ".forwardGlfM", verbose);
  loadPtreeValue(pt, settings.reverseGlfA_, fieldName + ".reverseGlfA", verbose);
  loadPtreeValue(pt, settings.reverseGlfK_, fieldName + ".reverseGlfK", verbose);
  loadPtreeValue(pt, settings.reverseGlfB_, fieldName + ".reverseGlfB", verbose);
  loadPtreeValue(pt, settings.reverseGlfV_, fieldName + ".reverseGlfV", verbose);
  loadPtreeValue(pt, settings.reverseGlfC_, fieldName + ".reverseGlfC", verbose);
  loadPtreeValue(pt, settings.reverseGlfM_, fieldName + ".reverseGlfM", verbose);

  loadPtreeValue(pt, settings.minInput_, fieldName + ".minInput", verbose);
  loadPtreeValue(pt, settings.maxInput_, fieldName + ".maxInput", verbose);
  loadPtreeValue(pt, settings.maxInputRate_, fieldName + ".maxInputRate", verbose);
  loadPtreeValue(pt, settings.maxSurgeSpeed_, fieldName + ".maxSurgeSpeed", verbose);
  loadPtreeValue(pt, settings.minSurgeSpeed_, fieldName + ".minSurgeSpeed", verbose);
  loadPtreeValue(pt, settings.maxAngularSpeed_, fieldName + ".maxAngularSpeed", verbose);

  if (verbose)
  {
    std::cout << " #### =============================================================================" << std::endl;
  }

  return settings;
}

template <typename SCALAR>
class USVDynamicModel : public ct::core::ControlledSystem<10, 2, SCALAR>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const size_t STATE_DIM = 10;   // x y cos(psi) sin(psi) psi u v r force_l force_r
  static const size_t CONTROL_DIM = 2;  // left/right thrust-force rates

  USVDynamicModel() = delete;

  USVDynamicModel(const USVParameters<double>& usvParameters) : params_(std::move(usvParameters))
  {
    std::cout << "USV model parameters loaded!" << std::endl;
  }

  USVDynamicModel(const USVDynamicModel<SCALAR>& other) : params_(std::move(other.params_))
  {
  }  //自定义模型时，拷贝构造函数也要有相应的实现，否则求解会有问题

  void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& state, const SCALAR& t,
                                 const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
                                 ct::core::StateVector<STATE_DIM, SCALAR>& derivative) override
  {
    // State: [x, y, cos(psi), sin(psi), psi, u, v, r, force_l, force_r].
    // The planar WAM-V model is
    //   x_dot   = u cos(psi) - v sin(psi)
    //   y_dot   = u sin(psi) + v cos(psi)
    //   psi_dot = r
    //   m(u_dot - vr) = tau_x - X_u u - X_uu |u|u
    //   m(v_dot + ur) = tau_y - Y_v v
    //   I_z r_dot     = tau_n - N_r r
    //   tau_x         = F_l + F_r
    //   tau_n         = -y_l F_l - y_r F_r.
    //
    // Parameters in model_wamv_nominal.info are the corresponding
    // mass/inertia-normalized acceleration coefficients.  The additional
    // polynomial terms are retained for the existing identified-model
    // configurations, where they may be nonzero.
    // SCALAR x = state(0);
    // SCALAR y = state(1);
    SCALAR cpsi = state(2);
    SCALAR spsi = state(3);
    SCALAR psi = state(4);
    SCALAR u = state(5);
    SCALAR v = state(6);
    SCALAR r = state(7);
    SCALAR force_l = state(8);
    SCALAR force_r = state(9);

    SCALAR uu = u * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(u);
    SCALAR vv = v * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v);
    SCALAR rr = r * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(r);
    SCALAR absv_r = r * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v);
    SCALAR v_absr = v * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(r);

    derivative(0) = u * cpsi - v * spsi;
    derivative(1) = u * spsi + v * cpsi;
    derivative(2) = -ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi) * r;
    derivative(3) = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi) * r;
    derivative(4) = r;

    const SCALAR surge_dynamics = params_.Xu_ * u + params_.Xu_absu_ * uu + params_.Xvr_ * v * r +
                                  params_.Xrr_ * r * r;
    const SCALAR sway_dynamics = params_.Yv_ * v + params_.Yr_ * r + params_.Yv_absv_ * vv +
                                 params_.Yr_absr_ * rr + params_.Yuv_ * u * v + params_.Yur_ * u * r +
                                 params_.Yabsv_r_ * absv_r + params_.Yv_absr_ * v_absr;
    const SCALAR yaw_dynamics = params_.Mv_ * v + params_.Mr_ * r + params_.Mv_absv_ * vv +
                                params_.Mr_absr_ * rr + params_.Muv_ * u * v + params_.Mur_ * u * r +
                                params_.Mabsv_r_ * absv_r + params_.Mv_absr_ * v_absr;

    const SCALAR surge_thrust = (force_l + force_r) / params_.mass_;
    const SCALAR yaw_thrust =
        (-params_.leftThrusterY_ * force_l - params_.rightThrusterY_ * force_r) / params_.yawInertia_;

    derivative(5) = surge_dynamics + surge_thrust;
    derivative(6) = sway_dynamics;
    derivative(7) = yaw_dynamics + yaw_thrust;
    derivative(8) = control(0);
    derivative(9) = control(1);
  }

  USVDynamicModel* clone() const override
  {
    return new USVDynamicModel(*this);
  }

  ~USVDynamicModel(){};

  /// Convert a desired physical thrust [N] to the normalized command used by
  /// the VRX GLF thruster plugin. The result is constrained to the documented
  /// command range [-maxThrustCmd, maxThrustCmd].
  double thrustForceToCommand(double force) const
  {
    const double min_force = static_cast<double>(params_.minInput_);
    const double max_force = static_cast<double>(params_.maxInput_);
    force = std::max(min_force, std::min(max_force, force));

    const double threshold = static_cast<double>(params_.thrustCmdThreshold_);
    const double reverse_at_threshold =
        glfDouble(threshold, params_.reverseGlfA_, params_.reverseGlfK_, params_.reverseGlfB_,
                  params_.reverseGlfV_, params_.reverseGlfC_, params_.reverseGlfM_);
    const double forward_at_threshold =
        glfDouble(threshold, params_.forwardGlfA_, params_.forwardGlfK_, params_.forwardGlfB_,
                  params_.forwardGlfV_, params_.forwardGlfC_, params_.forwardGlfM_);
    // Commands are published as Float32. Use representable values on the
    // correct side of the plugin's strict 0.01 branch comparison.
    const float threshold_float = static_cast<float>(threshold);
    const double reverse_cmd_limit = static_cast<double>(
        std::nextafter(threshold_float, -std::numeric_limits<float>::infinity()));
    const double forward_cmd_limit = static_cast<double>(
        std::nextafter(threshold_float, std::numeric_limits<float>::infinity()));

    double normalized_cmd;
    if (force <= reverse_at_threshold)
    {
      normalized_cmd = inverseGlf(force, params_.reverseGlfA_, params_.reverseGlfK_, params_.reverseGlfB_,
                                  params_.reverseGlfV_, params_.reverseGlfC_, params_.reverseGlfM_);
      normalized_cmd = std::min(normalized_cmd, reverse_cmd_limit);
    }
    else if (force >= forward_at_threshold)
    {
      normalized_cmd = inverseGlf(force, params_.forwardGlfA_, params_.forwardGlfK_, params_.forwardGlfB_,
                                  params_.forwardGlfV_, params_.forwardGlfC_, params_.forwardGlfM_);
      normalized_cmd = std::max(normalized_cmd, forward_cmd_limit);
    }
    else
    {
      // The two GLF branches leave a small unattainable force interval around
      // zero. Select the closer branch boundary instead of returning an
      // invalid inverse.
      if (force - reverse_at_threshold < forward_at_threshold - force)
      {
        normalized_cmd = reverse_cmd_limit;
      }
      else
      {
        normalized_cmd = forward_cmd_limit;
      }
    }

    const double normalized_limit = 1.0;
    normalized_cmd = std::max(-normalized_limit, std::min(normalized_limit, normalized_cmd));
    return normalized_cmd * static_cast<double>(params_.maxThrustCmd_);
  }

private:
  static double glfDouble(double x, double A, double K, double B, double v, double C, double M)
  {
    return A + (K - A) / std::pow(C + std::exp(-B * (x - M)), 1.0 / v);
  }

  static double inverseGlf(double force, double A, double K, double B, double v, double C, double M)
  {
    const double ratio = (K - A) / (force - A);
    const double exponential = std::pow(ratio, v) - C;
    if (!(ratio > 0.0) || !(exponential > 0.0))
      return 0.0;
    return M - std::log(exponential) / B;
  }

  USVParameters<double> params_;
};

/// @brief use gaussian process regression to model the system dynamics
/// @tparam SCALAR
/// @tparam STATE_DIMS
/// @tparam CONTROL_DIMS
template <size_t STATE_DIMS, size_t CONTROL_DIMS, typename SCALAR>
class GaussianProcessModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = STATE_DIMS;
  static const size_t CONTROL_DIM = CONTROL_DIMS;
  static const size_t INPUT_DIM = STATE_DIM + CONTROL_DIM;
  std::string state_names[STATE_DIM] = { "u", "v", "r" };
  GaussianProcessModel(const std::string& file_prefix, bool verbose = true)
  {
    // initialize std vectors
    inducePointsNum_.resize(STATE_DIM);
    hyperParametersNum_.resize(STATE_DIM);

    hyperParameters.resize(STATE_DIM);

    inducePoints.resize(STATE_DIM);
    induceTargets.resize(STATE_DIM);
    induceCovarianceMatrix.resize(STATE_DIM);

    alpha_vec.resize(STATE_DIM);
    L_vec.resize(STATE_DIM);

    std::string filename;
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      filename = file_prefix + "/output_" + state_names[dim] + "_sparse.txt";
      std::cout << "loading induce points from " << filename << std::endl;
      std::ifstream infile(filename);
      if (!infile.is_open())
      {
        std::cerr << "Error: cannot open file " << filename << std::endl;
        return;
      }
      int stage = 0;
      std::string s;
      while (infile.good())
      {
        getline(infile, s);
        // ignore empty lines and comments
        if (s.length() != 0 && s.at(0) != '#')
        {
          std::stringstream ss(s);
          if (stage > 2)
          {
            std::string value;
            std::vector<double> row;
            while (std::getline(ss, value, ' '))
            {
              row.push_back(std::stod(value));
            }
            if (row.size() == INPUT_DIM + 1)
            {
              induceTargets[dim].push_back(SCALAR(row[0]));
              // induceTargets[dim].push_back(SCALAR(0.0));
              inducePoints[dim].push_back((Eigen::Matrix<SCALAR, INPUT_DIM, 1>() << SCALAR(row[1]), SCALAR(row[2]),
                                           SCALAR(row[3]), SCALAR(row[4]), SCALAR(row[5]))
                                              .finished());
            }
          }
          else if (stage == 0)
          {
            // ss >> INPUT_DIM;
          }
          else if (stage == 1)
          {
            // CovFactory factory;
            // cf = factory.create(input_dim, s);
            // cf->loghyper_changed = 0;
          }
          else if (stage == 2)
          {
            Eigen::Matrix<double, Eigen::Dynamic, 1> loghypers(INPUT_DIM + 2);
            for (size_t i = 0; i < INPUT_DIM + 2; ++i)
            {
              ss >> loghypers[i];
            }
            // covSEard loghyperparams: log([length scale, signal variance, noise variance])
            hyperParametersNum_[dim] = INPUT_DIM + 2;
            hyperParameters[dim].resize(hyperParametersNum_[dim]);
            for (size_t i = 0; i < INPUT_DIM; ++i)
            {
              hyperParameters[dim](i) = ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(SCALAR(loghypers[i]));
            }
            hyperParameters[dim](INPUT_DIM) =
                ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(SCALAR(2 * loghypers[INPUT_DIM]));
            hyperParameters[dim](INPUT_DIM + 1) =
                ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(SCALAR(2 * loghypers[INPUT_DIM + 1]));
          }
          stage++;
        }
      }
      infile.close();
      if (stage < 3)
      {
        std::cerr << "fatal error while reading " << filename << std::endl;
        exit(EXIT_FAILURE);
      }
      inducePointsNum_[dim] = inducePoints[dim].size();
      std::cout << inducePointsNum_[dim] << " inducedPoints loaded! " << std::endl;
    }
    updateAlpha_LMatrix();
  }

  void updateHyperParameters();

  void recursiveBatchUpdate(const std::vector<Eigen::VectorXd>& gpTargets, double lambda = 1e-4)
  {
    SCALAR lambda_ = SCALAR(lambda);
    if (lambda_ < 0 || lambda_ > 1)
    {
      std::cerr << "Error: lambda should be in [0, 1]!" << std::endl;
      return;
    }
    size_t batch_size = gpTargets.size();
    if (batch_size == 0)
    {
      std::cerr << "Error: no training data provided!" << std::endl;
      return;
    }
    ROS_INFO("Updating GP with %zu training points", batch_size);
    std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> batch_targets(STATE_DIM);
    std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic, Eigen::Dynamic>> Hk_tilde_vec(STATE_DIM);
    std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> Vk_vec(STATE_DIM);
    std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> rk_vec(STATE_DIM);
    std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> Sk_vec(STATE_DIM);
    std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> Gk_tilde_vec(STATE_DIM);

    Eigen::Matrix<SCALAR,INPUT_DIM, Eigen::Dynamic> X_k;
    X_k.resize(INPUT_DIM, batch_size);
    for (size_t i = 0; i < batch_size; i++)
    {
      X_k.col(i) = (Eigen::Matrix<SCALAR, INPUT_DIM, 1>() << gpTargets[i](3), gpTargets[i](4), gpTargets[i](5),
                                                  gpTargets[i](6), gpTargets[i](7)).finished();
    }

    //init
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      batch_targets[dim].resize(batch_size);
      batch_targets[dim].setZero();
      Hk_tilde_vec[dim].resize(batch_size, inducePointsNum_[dim]);
      Hk_tilde_vec[dim].setZero();
      Vk_vec[dim].resize(batch_size, batch_size);
      Vk_vec[dim].setZero();
      rk_vec[dim].resize(batch_size);
      rk_vec[dim].setZero();
      Sk_vec[dim].resize(batch_size, batch_size);
      Sk_vec[dim].setZero();
      Gk_tilde_vec[dim].resize(inducePointsNum_[dim], batch_size);
      Gk_tilde_vec[dim].setZero();
    }

    //filling H_tilde, V_k, r_k, S_k, G_k
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      // Kxr matrix
      for (size_t i = 0; i < batch_size; i++)
      {
        for(size_t j = 0; j < inducePointsNum_[dim]; j++)
        {
          Hk_tilde_vec[dim](i, j) = computeKernel(X_k.col(i), inducePoints[dim][j], dim);
        }
      }

      // Vk = K_xx - Kxr * krr^-1 * K_xr^T + noise variance * I
      for (size_t i = 0; i < batch_size; i++)
      {
        for(size_t j = 0; j <= i; j++)
        { 
          Vk_vec[dim](i, j) = computeKernel(X_k.col(i), X_k.col(j), dim);
        }
      }
      Vk_vec[dim] = Vk_vec[dim].template selfadjointView<Eigen::Lower>();
      Vk_vec[dim].diagonal().array() += hyperParameters[dim](INPUT_DIM + 1); // noise variance 
      Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> Qkk;
      Qkk = L_Kuu_vec[dim].template triangularView<Eigen::Lower>().solve(Hk_tilde_vec[dim].transpose());
      L_Kuu_vec[dim].template triangularView<Eigen::Lower>().adjoint().solveInPlace(Qkk);
      Qkk = Hk_tilde_vec[dim] * Qkk;
      Vk_vec[dim] -= Qkk;

      for(size_t i = 0; i < batch_size; i++)
      {
        batch_targets[dim](i) = gpTargets[i](dim);
      }

      rk_vec[dim] = batch_targets[dim] - Hk_tilde_vec[dim] * alpha_vec[dim];

      //fill the G_k and S_k matrix
      Gk_tilde_vec[dim] = cov_tilde_vec[dim] * Hk_tilde_vec[dim].transpose();
      Sk_vec[dim] = Vk_vec[dim] + Hk_tilde_vec[dim] *  Gk_tilde_vec[dim];
      Sk_vec[dim].diagonal().array() += lambda_;  // add jitter to the diagonal for numerical stability

      // Cholesky decomposition of Sk
      Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> L_Sk;
      L_Sk = Sk_vec[dim].llt().matrixL();

      Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> y;
      y =  L_Sk.template triangularView<Eigen::Lower>().solve(rk_vec[dim]);
      L_Sk.template triangularView<Eigen::Lower>().adjoint().solveInPlace(y);
      y = Gk_tilde_vec[dim] * y;
      
      ROS_INFO("Updated mean vector for dimension %zu", dim);
      std::cout << "mean[" << dim << "] = " << alpha_vec[dim].transpose() << std::endl;
      std::cout << "correction value" << y.transpose() << std::endl;
      
      alpha_vec[dim] += y;  // update the mean vector

      //update the covariance matrix
      Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> cov_tilde_temp;
      cov_tilde_temp = L_Sk.template triangularView<Eigen::Lower>().solve(Gk_tilde_vec[dim].transpose());
      cov_tilde_vec[dim] -= cov_tilde_temp.transpose() * cov_tilde_temp;

      auto Kuu = L_Kuu_vec[dim]*L_Kuu_vec[dim].transpose();
      L_vec[dim] = Kuu - Kuu* cov_tilde_vec[dim]*Kuu;
      L_vec[dim].diagonal().array() += 1e-8;
      L_vec[dim] = L_vec[dim].llt().matrixL();
      L_Kuu_vec[dim].template triangularView<Eigen::Lower>().solveInPlace(L_vec[dim]);
      L_Kuu_vec[dim].template triangularView<Eigen::Lower>().adjoint().solveInPlace(L_vec[dim]);
    }
  };

  ~GaussianProcessModel(){};

  void predict(const Eigen::Matrix<SCALAR, GaussianProcessModel::INPUT_DIM, 1>& state_augmented, Eigen::Matrix<SCALAR, GaussianProcessModel::STATE_DIM, 1>& output)
  { 
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      assert(inducePoints[dim][0].rows() == INPUT_DIM);
      SCALAR mean = alpha_vec[dim](0) * computeKernel(state_augmented, inducePoints[dim][0], dim);
      for (size_t i = 1; i < inducePointsNum_[dim]; i++)
      {
        mean += alpha_vec[dim](i) * computeKernel(state_augmented, inducePoints[dim][i], dim);
      }
      output(dim) = mean;
    }
  };

  void covariance(const Eigen::Matrix<SCALAR, INPUT_DIM, 1>& state_augmented_1,
                    Eigen::Matrix<SCALAR, STATE_DIM, 1>& output)
  {
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> v;
      v.resize(inducePointsNum_[dim]);
      v.setZero();
      for (size_t i = 0; i < inducePointsNum_[dim]; i++)
      {
        v(i) = computeKernel(state_augmented_1, inducePoints[dim][i], dim);
      }
      v = L_vec[dim].transpose() * v;
      output(dim) = computeKernel(state_augmented_1, state_augmented_1, dim) - v.dot(v);
    }
  };

  // 更新alpha_vec和L_vec的函数
  void setAlphaAndLVectors(const std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>>& new_alpha_vec, 
                           const std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>& new_L_vec)
  {
    if (new_alpha_vec.size() != STATE_DIM)
    {
      std::cerr << "Error: new_alpha_vec size mismatch!" << std::endl;
      return;
    }
    if (new_L_vec.size() != STATE_DIM)
    {
      std::cerr << "Error: new_L_vec size mismatch!" << std::endl;
      return;
    }
    for (size_t dim = 0; dim < STATE_DIM; ++dim)
    {
      if (new_alpha_vec[dim].size() != alpha_vec[dim].size())
      {
        std::cerr << "Error: new_alpha_vec[" << dim << "] size mismatch!" << std::endl;
        continue;
      }
      alpha_vec[dim] = new_alpha_vec[dim].template cast<SCALAR>();
      if (new_L_vec[dim].size() != L_vec[dim].size())
      {
        std::cerr << "Error: new_L_vec[" << dim << "] size mismatch!" << std::endl;
        continue;
      }
      L_vec[dim] = new_L_vec[dim].template cast<SCALAR>();
    }
  }

  const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& getAlphaVec() const
  {
    return alpha_vec;
  }

  const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>>& getLVec() const
  {
    return L_vec;
  }

  GaussianProcessModel* clone() const
  {
    return new GaussianProcessModel(*this);
  }

private:
  std::vector<size_t> inducePointsNum_;
  std::vector<size_t> hyperParametersNum_;  // 0: noise variance, 1: signal variance, 2~: length scale

  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> hyperParameters;

  std::vector<std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>> inducePoints;  // z_ind = (x_ind; u_ind)
  std::vector<std::vector<SCALAR>> induceTargets;                                   // y_ind
  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> induceCovarianceMatrix;

  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> L_Kuu_vec;
  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> alpha_vec;
  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> L_vec;
  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> cov_tilde_vec;

  void updateAlpha_LMatrix()
  {
    L_Kuu_vec.resize(STATE_DIM);
    cov_tilde_vec.resize(STATE_DIM);
    L_vec.resize(STATE_DIM);

    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      L_Kuu_vec[dim].resize(inducePointsNum_[dim], inducePointsNum_[dim]);
      for (size_t i = 0; i < inducePointsNum_[dim]; ++i)
      {
        for (size_t j = 0; j <= i; ++j)
        {
          L_Kuu_vec[dim](i, j) = computeKernel(inducePoints[dim][i], inducePoints[dim][j], dim);
        }
      }

      L_Kuu_vec[dim].diagonal().array() += hyperParameters[dim](INPUT_DIM + 1);  // noise variance
      L_Kuu_vec[dim] = L_Kuu_vec[dim].template selfadjointView<Eigen::Lower>().llt().matrixL();

      Eigen::Map<const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> y(&induceTargets[dim][0], induceTargets[dim].size());
      alpha_vec[dim] = L_Kuu_vec[dim].template triangularView<Eigen::Lower>().solve(y);
      L_Kuu_vec[dim].template triangularView<Eigen::Lower>().adjoint().solveInPlace(alpha_vec[dim]);

      cov_tilde_vec[dim].resize(inducePointsNum_[dim], inducePointsNum_[dim]);
      cov_tilde_vec[dim].setIdentity();
      L_Kuu_vec[dim].template triangularView<Eigen::Lower>().solveInPlace(cov_tilde_vec[dim]);
      L_Kuu_vec[dim].template triangularView<Eigen::Lower>().adjoint().solveInPlace(cov_tilde_vec[dim]);

      L_vec[dim].resize(inducePointsNum_[dim], inducePointsNum_[dim]);
      L_vec[dim].setZero();
    }

  };

  // ard squared exponential kernel
  SCALAR computeKernel(const Eigen::Matrix<SCALAR, INPUT_DIM, 1>& state_augmented_1,
                       const Eigen::Matrix<SCALAR, INPUT_DIM, 1>& state_augmented_2, size_t dim)
  {
    SCALAR z =
        (state_augmented_1 - state_augmented_2).cwiseQuotient(hyperParameters[dim].head(INPUT_DIM)).squaredNorm();
    return hyperParameters[dim](INPUT_DIM) * ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(-0.5 * z);
  };
};

template <typename SCALAR>
class USVCorrectionModel : public ct::core::ControlledSystem<10, 2, SCALAR>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = 10;  // x y cos(psi) sin(psi) psi  u v r
  static const size_t VELOCITY_STATE_DIM = 3;
  static const size_t CONTROL_DIM = 2;  // dual thrusts

  using GaussianProcessModel_t = GaussianProcessModel<VELOCITY_STATE_DIM, CONTROL_DIM, SCALAR>;
  using USVDynamicModel_t = USVDynamicModel<SCALAR>;

  USVCorrectionModel() = delete;

  USVCorrectionModel(std::shared_ptr<USVDynamicModel_t> usvNominalModel,
                     std::shared_ptr<GaussianProcessModel_t> gpModel)
    : usvNominalModel_(usvNominalModel), gpModel_(gpModel)
  {
    std::cout << "USV Corrected Model Generated!" << std::endl;
  }

  USVCorrectionModel(const USVCorrectionModel<SCALAR>& other)
  {
    if (other.usvNominalModel_)
    {
      usvNominalModel_ = std::shared_ptr<USVDynamicModel_t>(other.usvNominalModel_->clone());
    }
    if (other.gpModel_)
    {
      gpModel_ = std::shared_ptr<GaussianProcessModel_t>(other.gpModel_->clone());
    }
  }  //自定义模型时，拷贝构造函数也要有相应的实现，否则求解会有问题

  void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& state, const SCALAR& t,
                                 const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
                                 ct::core::StateVector<STATE_DIM, SCALAR>& derivative) override
  {
    ct::core::StateVector<VELOCITY_STATE_DIM, SCALAR> gp_correction;
    gpModel_->predict(state.tail(5), gp_correction);

    usvNominalModel_->computeControlledDynamics(state, t, control, derivative);

    derivative(5) += gp_correction(0);
    derivative(6) += gp_correction(1);
    derivative(7) += gp_correction(2);
  }

  USVCorrectionModel* clone() const override
  {
    return new USVCorrectionModel(*this);
  }

  ~USVCorrectionModel(){};

private:
  std::shared_ptr<GaussianProcessModel_t> gpModel_;
  std::shared_ptr<USVDynamicModel_t> usvNominalModel_;
};

}  // namespace tpl
typedef tpl::USVDynamicModel<double> USVDynamicModel;
typedef tpl::USVCorrectionModel<double> USVCorrectionModel;
typedef tpl::GaussianProcessModel<3, 2, double> GaussianProcessModel;

}  // namespace usv
