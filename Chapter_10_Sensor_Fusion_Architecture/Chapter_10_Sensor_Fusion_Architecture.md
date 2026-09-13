**Volume 15 Perception and Sensor Architecture**


# Chapter 10. Sensor Fusion Architecture

##  

## 10.01. Kalman Filter Basics

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

A Kalman filter is a recursive estimation method used to infer the internal state of a dynamic system from measurements that contain noise and uncertainty. In sensor fusion architecture, the state may represent position, velocity, acceleration, orientation, or sensor bias. Rather than trusting one measurement directly, the filter continuously combines a predicted state with new observations to obtain a statistically improved estimate.

The fundamental idea is to represent both the estimated state and its uncertainty. The state vector contains the variables that the system needs to estimate, while the covariance matrix describes how uncertain those estimates are and how errors between state variables are correlated. This explicit representation of uncertainty distinguishes Kalman filtering from simple averaging or deterministic correction methods and enables systematic fusion of heterogeneous sensors.

The prediction stage estimates how the state evolves between measurements. A mathematical process model propagates the previous state forward using elapsed time, system dynamics, and optional control inputs. For a mobile robot, a simple model may predict position from velocity and predict velocity from acceleration. The covariance is propagated at the same time so that uncertainty normally increases while the system operates without corrective observations.

Process noise represents uncertainty that cannot be captured perfectly by the motion model. Wheel slip, vibration, terrain irregularity, unmodeled acceleration, actuator variation, and changing environmental conditions can all cause the real system to deviate from its predicted behavior. The process-noise covariance determines how strongly the filter acknowledges these unknown dynamics. If it is configured too small, the estimator can become excessively confident in an inaccurate prediction.

The measurement stage begins when a sensor observation becomes available. A measurement model describes how the internal state should appear in the sensor domain. The difference between the actual observation and the predicted observation is called the innovation or residual. This quantity provides the information required to correct the predicted state and also serves as an important diagnostic indicator for detecting inconsistent measurements or incorrect models.

The Kalman gain determines the relative influence of prediction and measurement during correction. It is calculated from predicted state uncertainty, measurement uncertainty, and the measurement model. When the prediction is uncertain and the sensor is reliable, the gain gives greater influence to the measurement. When sensor noise is large, the estimator relies more strongly on its predicted state. This weighting is recomputed recursively as uncertainty changes.

Measurement-noise covariance describes the expected uncertainty of each sensor observation. GNSS position, IMU acceleration, wheel odometry, LiDAR localization, radar velocity, and camera-based estimates have different error characteristics and therefore should not be treated equally. Accurate covariance modeling allows the fusion algorithm to assign appropriate confidence to each information source instead of assuming that all sensor outputs have identical reliability.

The complete Kalman filtering cycle therefore consists of repeated prediction and correction. Prediction propagates the state and covariance forward in time, while correction incorporates available observations and reduces uncertainty where useful information is provided. Because the previous estimate becomes the starting point for the next cycle, the algorithm operates recursively and does not need to retain the complete history of all earlier measurements.

The classical Kalman filter assumes that system dynamics and measurement relationships can be represented by linear models and that relevant uncertainty can be approximated by Gaussian distributions. Under these assumptions, it provides an efficient minimum-error estimation framework. Many robotic systems, however, contain nonlinear motion, rotation, coordinate transformations, range measurements, and sensor models, requiring extensions of the basic formulation.

The Extended Kalman Filter, commonly called EKF, addresses nonlinear systems by linearizing nonlinear process and measurement functions around the current estimate. Jacobian matrices describe the local relationship between small changes in the state and corresponding changes in system behavior. EKF is widely used in robotics because navigation states involving orientation, inertial measurements, GNSS observations, and vehicle motion are naturally nonlinear.

For more strongly nonlinear estimation problems, alternatives such as the Unscented Kalman Filter can be considered. Instead of locally linearizing the nonlinear functions, the UKF propagates a carefully selected set of representative sigma points through those functions. This can capture nonlinear transformations more accurately in some applications, although it generally requires greater computation than a basic linear Kalman filter or a carefully optimized EKF.

In an AMR navigation architecture, an IMU can provide high-rate acceleration and angular-rate information for rapid state propagation, while GNSS RTK can periodically constrain absolute position outdoors. Wheel odometry can contribute short-term motion information, and LiDAR or camera localization can provide additional pose observations. The Kalman framework combines these measurements according to their modeled uncertainty rather than simply selecting one sensor as authoritative.

This complementary behavior is especially important because different sensors fail in different ways. GNSS may provide accurate global coordinates in open environments but degrade near buildings or under obstruction. IMU measurements remain available at high frequency but accumulate drift when integrated over time. Wheel odometry can be locally stable but becomes inaccurate during slip. Fusion allows one sensing modality to constrain errors accumulated by another.

Sensor bias must also be considered as part of the estimation architecture. Gyroscope bias and accelerometer bias can produce substantial navigation drift even when the instantaneous measurement error appears small. A practical filter can include these biases directly in the state vector and estimate them together with motion states. This transforms slowly varying sensor errors from hidden disturbances into variables that can be continuously observed and corrected.

Covariance tuning is consequently one of the most important engineering tasks in practical Kalman filtering. Process and measurement covariance values should reflect actual sensor behavior, vehicle dynamics, mounting conditions, vibration, sampling rate, and operating environment. Incorrect tuning can produce estimates that appear smooth while being physically wrong, or estimates that respond excessively to noisy measurements and become unstable or inconsistent.

Time synchronization is equally critical in multi-sensor filtering. A mathematically correct estimator can still generate large errors when measurements correspond to different physical times. GNSS, IMU, LiDAR, radar, camera, and wheel-encoder data should therefore carry reliable timestamps referenced to a common time base. Hardware timestamping, PTP synchronization, trigger signals, and carefully characterized communication latency can significantly improve fusion accuracy.

Measurement validation should normally precede state correction. Innovation magnitude can be compared with expected statistical bounds to identify observations that are inconsistent with the predicted state. Outlier rejection or adaptive weighting can prevent a temporary GNSS jump, false visual localization result, corrupted wheel measurement, or abnormal radar observation from abruptly disturbing the fused state. This makes filtering part of a broader fault-aware perception architecture.

The filter update rate should be selected according to vehicle dynamics, sensor frequencies, and compute capability. High-rate inertial prediction may operate much faster than GNSS, LiDAR, or camera corrections. A practical fusion implementation therefore needs to support asynchronous observations while maintaining consistent state propagation between timestamps. Buffering and delayed-measurement handling may be necessary when processing pipelines introduce different and variable latencies.

For Physical AI systems, the Kalman filter is best understood as an uncertainty-aware state estimation layer between raw sensing and higher-level intelligence. Perception, localization, planning, control, safety monitoring, and world modeling depend on a coherent estimate of the robot\'s physical state. The filter converts noisy and partially redundant measurements into a continuously updated representation that downstream algorithms can consume with associated confidence information.

The basic Kalman filter is therefore not merely a noise-removal algorithm. It establishes a systematic relationship among physical models, sensor observations, uncertainty, timing, and recursive correction. Understanding this relationship provides the foundation for more advanced sensor fusion architectures, including EKF-based navigation, inertial fusion, fault-tolerant estimation, and tightly synchronized multi-sensor systems used in AMRs, autonomous vehicles, and UAVs.

칼만 필터(Kalman Filter)는 잡음(Noise)과 불확실성(Uncertainty)이 포함된 측정값으로부터 동적 시스템(Dynamic System)의 내부 상태(State)를 추정하기 위한 재귀적 추정 방법(Recursive Estimation Method)이다. 센서 융합 구조(Sensor Fusion Architecture)에서 상태는 위치(Position), 속도(Velocity), 가속도(Acceleration), 자세(Orientation), 센서 바이어스(Sensor Bias) 등을 나타낼 수 있다. 하나의 측정값을 직접 신뢰하는 대신, 필터는 예측 상태(Predicted State)와 새로운 관측값(Observation)을 지속적으로 결합하여 통계적으로 개선된 추정값을 얻는다.

기본적인 개념은 추정 상태(Estimated State)와 그 불확실성을 함께 표현하는 것이다. 상태 벡터(State Vector)는 시스템이 추정해야 하는 변수들을 포함하며, 공분산 행렬(Covariance Matrix)은 이러한 추정값이 얼마나 불확실한지와 상태 변수 간 오차가 어떻게 상관되어 있는지를 나타낸다. 이러한 명시적 불확실성 표현은 칼만 필터를 단순 평균(Simple Averaging)이나 결정론적 보정(Deterministic Correction) 방식과 구별하며, 서로 다른 센서를 체계적으로 융합할 수 있게 한다.

예측 단계(Prediction Stage)는 측정 사이의 시간 동안 상태가 어떻게 변화하는지를 추정한다. 수학적 프로세스 모델(Process Model)은 경과 시간(Elapsed Time), 시스템 동역학(System Dynamics), 선택적인 제어 입력(Control Input)을 사용하여 이전 상태를 미래 시점으로 전파한다. 이동 로봇(Mobile Robot)의 간단한 모델에서는 속도로부터 위치를 예측하고 가속도로부터 속도를 예측할 수 있다. 동시에 공분산도 전파되므로 보정 관측값이 없는 동안에는 일반적으로 불확실성이 증가한다.

프로세스 잡음(Process Noise)은 운동 모델(Motion Model)이 완벽하게 표현할 수 없는 불확실성을 나타낸다. 휠 슬립(Wheel Slip), 진동(Vibration), 지형 불규칙성(Terrain Irregularity), 모델링되지 않은 가속도(Unmodeled Acceleration), 액추에이터 편차(Actuator Variation), 환경 조건 변화 등이 실제 시스템을 예측된 동작에서 벗어나게 할 수 있다. 프로세스 잡음 공분산(Process-Noise Covariance)은 필터가 이러한 알려지지 않은 동역학을 어느 정도 인정할지를 결정한다. 이 값을 지나치게 작게 설정하면 추정기가 부정확한 예측을 과도하게 신뢰할 수 있다.

측정 단계(Measurement Stage)는 센서 관측값(Sensor Observation)이 사용 가능해질 때 시작된다. 측정 모델(Measurement Model)은 내부 상태가 센서 영역(Sensor Domain)에서 어떻게 나타나야 하는지를 정의한다. 실제 관측값과 예측 관측값 사이의 차이를 혁신값 또는 잔차(Innovation or Residual)라고 한다. 이 값은 예측 상태를 보정하는 데 필요한 정보를 제공하며, 동시에 일관되지 않은 측정값이나 잘못된 모델을 검출하기 위한 중요한 진단 지표(Diagnostic Indicator)로 활용된다.

칼만 이득(Kalman Gain)은 보정 과정에서 예측값과 측정값이 각각 어느 정도 영향을 미칠지를 결정한다. 이는 예측 상태의 불확실성, 측정 불확실성, 측정 모델을 기반으로 계산된다. 예측의 불확실성이 크고 센서의 신뢰성이 높으면 측정값의 영향이 증가한다. 반대로 센서 잡음(Sensor Noise)이 크면 추정기는 예측 상태에 더 의존한다. 이러한 가중치는 불확실성이 변화함에 따라 재귀적으로 다시 계산된다.

측정 잡음 공분산(Measurement-Noise Covariance)은 각 센서 관측값에 예상되는 불확실성을 표현한다. 위성항법시스템 위치(GNSS Position), 관성측정장치 가속도(IMU Acceleration), 휠 오도메트리(Wheel Odometry), 라이다 위치추정(LiDAR Localization), 레이더 속도(Radar Velocity), 카메라 기반 추정(Camera-Based Estimate)은 서로 다른 오차 특성을 가지므로 동일하게 취급해서는 안 된다. 정확한 공분산 모델링(Covariance Modeling)을 통해 융합 알고리즘은 모든 센서 출력을 동일한 신뢰도로 가정하지 않고 각 정보원에 적절한 신뢰도를 부여할 수 있다.

따라서 전체 칼만 필터링(Kalman Filtering) 과정은 반복적인 예측(Prediction)과 보정(Correction)으로 구성된다. 예측은 상태와 공분산을 시간에 따라 앞으로 전파하고, 보정은 사용 가능한 관측값을 반영하여 유용한 정보가 제공되는 영역의 불확실성을 감소시킨다. 이전 추정값이 다음 주기의 시작점이 되므로 알고리즘은 재귀적으로 동작하며, 과거의 모든 측정 이력을 완전히 저장할 필요가 없다.

고전적 칼만 필터(Classical Kalman Filter)는 시스템 동역학과 측정 관계가 선형 모델(Linear Model)로 표현될 수 있고 관련 불확실성이 가우시안 분포(Gaussian Distribution)로 근사될 수 있다고 가정한다. 이러한 조건에서 칼만 필터는 효율적인 최소 오차 추정(Minimum-Error Estimation) 프레임워크를 제공한다. 그러나 많은 로봇 시스템에는 비선형 운동(Nonlinear Motion), 회전(Rotation), 좌표 변환(Coordinate Transformation), 거리 측정(Range Measurement), 센서 모델 등이 포함되므로 기본 공식의 확장이 필요하다.

확장 칼만 필터(Extended Kalman Filter, EKF)는 현재 추정값 주변에서 비선형 프로세스 및 측정 함수(Nonlinear Process and Measurement Function)를 선형화함으로써 비선형 시스템을 처리한다. 야코비안 행렬(Jacobian Matrix)은 상태의 작은 변화와 이에 따른 시스템 동작 변화 사이의 국소적 관계를 표현한다. 자세, 관성 측정, 위성항법시스템 관측, 차량 운동이 본질적으로 비선형이기 때문에 확장 칼만 필터는 로보틱스(Robotics) 분야에서 널리 사용된다.

더 강한 비선형성을 갖는 추정 문제에서는 무향 칼만 필터(Unscented Kalman Filter, UKF)와 같은 대안을 고려할 수 있다. 비선형 함수를 국소적으로 선형화하는 대신, 무향 칼만 필터는 신중하게 선택된 대표 시그마 포인트(Sigma Point) 집합을 비선형 함수를 통해 전파한다. 일부 응용에서는 비선형 변환을 더욱 정확하게 표현할 수 있지만, 일반적으로 기본 선형 칼만 필터나 효율적으로 최적화된 확장 칼만 필터보다 더 많은 계산량을 요구한다.

자율이동로봇 내비게이션 구조(AMR Navigation Architecture)에서 관성측정장치(IMU)는 빠른 상태 전파를 위해 고주파 가속도와 각속도(Angular Rate) 정보를 제공할 수 있으며, 실외에서는 실시간 이동측위 위성항법시스템(GNSS RTK)이 절대 위치를 주기적으로 보정할 수 있다. 휠 오도메트리는 단기 운동 정보를 제공하고, 라이다 또는 카메라 위치추정은 추가적인 자세 관측값(Pose Observation)을 제공한다. 칼만 프레임워크는 하나의 센서를 절대적인 기준으로 선택하는 대신 모델링된 불확실성에 따라 이러한 측정값을 결합한다.

이러한 상호보완적 동작(Complementary Behavior)은 센서마다 서로 다른 방식으로 성능이 저하되거나 실패하기 때문에 특히 중요하다. 위성항법시스템은 개방된 환경에서 정확한 전역 좌표(Global Coordinate)를 제공할 수 있지만 건물 주변이나 신호가 차단되는 환경에서는 성능이 저하될 수 있다. 관성측정장치는 높은 주파수로 지속적인 측정을 제공하지만 시간에 따라 적분하면 드리프트(Drift)가 누적된다. 휠 오도메트리는 국부적으로 안정적일 수 있지만 슬립(Slip)이 발생하면 오차가 증가한다. 센서 융합은 한 센서 방식이 다른 센서에서 누적되는 오차를 제한하도록 한다.

센서 바이어스(Sensor Bias) 역시 추정 구조에서 고려해야 한다. 자이로스코프 바이어스(Gyroscope Bias)와 가속도계 바이어스(Accelerometer Bias)는 순간적인 측정 오차가 작아 보이더라도 상당한 내비게이션 드리프트(Navigation Drift)를 발생시킬 수 있다. 실용적인 필터에서는 이러한 바이어스를 상태 벡터에 직접 포함하여 운동 상태와 함께 추정할 수 있다. 이를 통해 천천히 변화하는 센서 오차를 숨겨진 외란(Hidden Disturbance)이 아니라 지속적으로 관측하고 보정할 수 있는 변수로 변환할 수 있다.

따라서 공분산 튜닝(Covariance Tuning)은 실제 칼만 필터링에서 가장 중요한 엔지니어링 작업 중 하나이다. 프로세스 및 측정 공분산 값은 실제 센서 특성, 차량 동역학, 장착 조건, 진동, 샘플링 주파수(Sampling Rate), 운용 환경을 반영해야 한다. 잘못된 튜닝은 추정 결과를 부드럽게 보이게 하면서도 물리적으로 잘못된 상태를 생성하거나, 잡음이 많은 측정값에 지나치게 민감하게 반응하여 추정 결과를 불안정하거나 비일관적으로 만들 수 있다.

다중 센서 필터링(Multi-Sensor Filtering)에서는 시간 동기화(Time Synchronization)도 매우 중요하다. 수학적으로 올바른 추정기라도 측정값들이 서로 다른 실제 시점에 대응한다면 큰 오차를 발생시킬 수 있다. 따라서 위성항법시스템, 관성측정장치, 라이다, 레이더, 카메라, 휠 인코더(Wheel Encoder) 데이터는 공통 시간 기준(Common Time Base)에 연결된 신뢰성 높은 타임스탬프(Timestamp)를 가져야 한다. 하드웨어 타임스탬핑(Hardware Timestamping), 정밀 시간 프로토콜(Precision Time Protocol, PTP), 트리거 신호(Trigger Signal), 통신 지연(Communication Latency)의 정밀한 특성화는 융합 정확도를 크게 향상시킬 수 있다.

일반적으로 상태 보정(State Correction)에 앞서 측정값 검증(Measurement Validation)이 수행되어야 한다. 혁신값의 크기를 예상 통계 범위(Expected Statistical Bound)와 비교하여 예측 상태와 일치하지 않는 관측값을 식별할 수 있다. 이상치 제거(Outlier Rejection) 또는 적응형 가중치(Adaptive Weighting)를 적용하면 일시적인 위성항법시스템 위치 점프, 잘못된 비전 위치추정 결과, 손상된 휠 측정값, 비정상적인 레이더 관측값 등이 융합 상태를 갑작스럽게 교란하는 것을 방지할 수 있다. 이를 통해 필터링은 더 광범위한 고장 인지형 인지 구조(Fault-Aware Perception Architecture)의 일부가 된다.

필터 업데이트 주기(Filter Update Rate)는 차량 동역학, 센서 주파수, 컴퓨팅 성능에 따라 결정되어야 한다. 고주파 관성 예측(High-Rate Inertial Prediction)은 위성항법시스템, 라이다 또는 카메라 보정보다 훨씬 빠르게 동작할 수 있다. 따라서 실용적인 융합 구현은 서로 다른 시점에 도착하는 비동기 관측값(Asynchronous Observation)을 지원하면서 각 타임스탬프 사이에서 일관된 상태 전파를 유지해야 한다. 처리 파이프라인에서 서로 다른 가변 지연이 발생하는 경우 버퍼링(Buffering)과 지연 측정 처리(Delayed-Measurement Handling)가 필요할 수 있다.

피지컬 AI 시스템(Physical AI System)에서 칼만 필터는 원시 센싱(Raw Sensing)과 상위 수준 지능(Higher-Level Intelligence) 사이에 위치하는 불확실성 인지형 상태 추정 계층(Uncertainty-Aware State Estimation Layer)으로 이해하는 것이 적절하다. 인지(Perception), 위치추정(Localization), 경로 계획(Planning), 제어(Control), 안전 모니터링(Safety Monitoring), 월드 모델링(World Modeling)은 모두 로봇의 물리적 상태에 대한 일관된 추정값에 의존한다. 필터는 잡음이 많고 부분적으로 중복되는 측정값을 하위 알고리즘이 관련 신뢰도 정보와 함께 사용할 수 있는 지속적으로 갱신되는 표현으로 변환한다.

따라서 기본 칼만 필터(Basic Kalman Filter)는 단순한 잡음 제거 알고리즘(Noise-Removal Algorithm)이 아니다. 이는 물리 모델(Physical Model), 센서 관측(Sensor Observation), 불확실성, 시간, 재귀적 보정(Recursive Correction) 사이의 체계적인 관계를 확립한다. 이러한 관계에 대한 이해는 확장 칼만 필터 기반 내비게이션(EKF-Based Navigation), 관성 센서 융합(Inertial Fusion), 고장 허용 추정(Fault-Tolerant Estimation), 그리고 자율이동로봇(AMR), 자율주행차(Autonomous Vehicle), 무인항공기(UAV)에 사용되는 정밀 동기화 다중 센서 시스템(Tightly Synchronized Multi-Sensor System)과 같은 더욱 발전된 센서 융합 구조의 기반을 제공한다.

##  

## 10.02. Particle Filter Basics

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

A particle filter is a recursive Bayesian estimation method used to represent and track uncertain system states with a collection of discrete hypotheses called particles. Each particle represents one possible state of the system and carries an associated weight describing its probability. Unlike a Kalman filter, a particle filter does not require the state uncertainty to follow a Gaussian distribution or the system equations to be linear.

The state distribution is approximated by many particles distributed across the possible state space. A particle may contain position, velocity, orientation, sensor bias, landmark association, or other variables required by the estimation problem. Together, the particles approximate the posterior probability distribution of the system state, allowing the estimator to represent asymmetric, multimodal, or highly nonlinear uncertainty.

Particle filtering is based on the Bayesian estimation principle. The estimator maintains a belief about the current state and recursively updates that belief as the system moves and new measurements arrive. The previous posterior distribution becomes the prior information for the next estimation cycle. This recursive structure makes particle filtering suitable for continuous localization and tracking problems in autonomous robots and other dynamic systems.

The prediction stage propagates every particle through a process or motion model. For a mobile robot, the model may use commanded velocity, steering angle, wheel odometry, inertial measurements, or vehicle kinematics to estimate how each hypothetical state changes over time. Random process noise is introduced during propagation so that the particle population represents uncertainty caused by motion errors, wheel slip, terrain changes, and unmodeled dynamics.

After prediction, the particles usually spread across the state space because uncertainty accumulates during motion. The amount and direction of this spread depend on the process model and its noise distribution. A highly accurate motion model produces a relatively concentrated particle population, while uncertain motion produces wider dispersion. This mechanism allows particle filters to represent complex uncertainty without forcing it into a single covariance matrix.

When a new sensor observation becomes available, the measurement model evaluates how well each predicted particle explains that observation. Measurements may originate from GNSS, LiDAR, radar, cameras, wheel encoders, or other perception sensors. The predicted observation associated with each particle is compared with the actual measurement, and a likelihood value is calculated to express their consistency.

The likelihood becomes the basis for particle weighting. Particles that are highly consistent with the sensor measurement receive larger weights, while particles that poorly explain the observation receive smaller weights. After normalization, the weights approximate the relative probability of each hypothesis. The resulting weighted population represents the updated posterior distribution after incorporating the latest sensor information.

A fundamental operation of particle filtering is resampling. During resampling, particles with high weights are selected more frequently to form the next population, while particles with very low weights are gradually removed. This concentrates computational resources around regions of the state space that are supported by observations. The new population then becomes the starting distribution for the next prediction and measurement cycle.

Repeated resampling can create particle degeneracy or sample impoverishment. Degeneracy occurs when most particles carry negligible weights and only a few particles dominate the distribution. Sample impoverishment occurs when repeated copies of high-weight particles reduce diversity. Effective implementations therefore monitor particle quality and may use adaptive resampling, additional noise, improved proposal distributions, or other mechanisms to preserve useful hypotheses.

The effective sample size is commonly used to determine whether resampling is necessary. Instead of resampling after every measurement, the estimator can examine how concentrated the normalized particle weights have become. If many particles still contribute meaningfully, resampling can be postponed. If only a small number of particles dominate, resampling can be triggered to redistribute computational effort toward more probable state regions.

Particle count strongly influences both estimation quality and computational cost. Too few particles may fail to represent important hypotheses, particularly when the state space is large or the probability distribution contains multiple modes. Increasing the number of particles generally improves distribution coverage but requires additional processing for prediction, measurement evaluation, normalization, and resampling. Real-time systems therefore require a balance between accuracy and available compute resources.

One of the major advantages of particle filtering is its ability to maintain multiple hypotheses simultaneously. During robot localization, for example, several different positions may initially appear consistent with similar environmental features. A Gaussian estimator may struggle to represent these separated possibilities, whereas a particle filter can maintain clusters in several locations until subsequent observations provide enough evidence to eliminate incorrect hypotheses.

This property is particularly useful for global localization and kidnapped-robot recovery. When the initial robot pose is unknown, particles can be distributed over a large map rather than initialized around one assumed location. As LiDAR, camera, GNSS, or other observations arrive, inconsistent hypotheses lose weight while particles near the true pose become dominant. The estimator can therefore converge from substantial initial uncertainty.

Particle filters are also well suited to nonlinear motion and measurement models. Vehicle turning dynamics, range observations, bearing measurements, obstacle geometry, and visual perception can introduce nonlinear relationships between the physical state and sensor observations. Because particles are propagated directly through these models, explicit local linearization such as the Jacobian calculations required by an Extended Kalman Filter is generally unnecessary.

The flexibility of particle filtering comes with significant computational requirements. Every particle must be propagated and evaluated against measurements, and complex sensor models can make likelihood computation expensive. LiDAR scan matching or image-based likelihood evaluation across thousands of particles may dominate processing time. Efficient implementations therefore use reduced representations, parallel processing, hierarchical estimation, or carefully designed observation models.

Measurement modeling is critical to filter performance. A theoretically powerful particle filter can still fail if its likelihood function does not accurately represent sensor behavior. Measurement noise, environmental ambiguity, occlusion, multipath effects, false detections, map errors, and changing operating conditions should be reflected appropriately. Overly sharp likelihood models can eliminate valid hypotheses, while overly broad models may prevent convergence.

Time synchronization also remains important even though the estimator can represent complex uncertainty. Particles predicted to one timestamp should be compared with measurements corresponding to that same physical time. Delayed camera processing, LiDAR scan duration, GNSS latency, radar timestamps, and asynchronous IMU updates can otherwise produce systematic likelihood errors. Timestamp management and latency compensation are therefore integral parts of practical particle-filter architecture.

Particle filters can operate together with other estimation methods rather than replacing them completely. A system may use a particle filter for global localization while employing an Extended Kalman Filter for high-rate local motion estimation. Particle-based estimation can resolve multiple global hypotheses, after which a Gaussian estimator provides efficient continuous tracking. Such hybrid architectures exploit the complementary strengths of different probabilistic estimation techniques.

In an AMR, autonomous vehicle, or UAV architecture, particle filtering can support localization, object tracking, terrain-state estimation, fault diagnosis, and other problems involving ambiguous or non-Gaussian uncertainty. The method is particularly valuable when sensor observations produce several plausible explanations or when the system must recover from a major localization failure rather than merely correct small errors around a known state.

The output of a particle filter may be represented by the highest-weight particle, a weighted mean, a dominant particle cluster, or the complete probability distribution. The appropriate representation depends on the downstream application. Planning and control may require a single pose estimate, while safety monitoring or decision-making can benefit from information about multiple hypotheses and the uncertainty distributed among them.

For Physical AI systems, particle filtering provides a mechanism for reasoning about physical state when uncertainty cannot be adequately summarized by one mean and covariance. It enables perception and localization systems to preserve alternative explanations until sensor evidence becomes sufficiently strong. This capability is valuable in dynamic, partially observable, and ambiguous environments where premature commitment to a single hypothesis can cause incorrect decisions.

The particle filter should therefore be understood as a sampling-based Bayesian estimation framework rather than simply another noise-filtering technique. Its prediction, likelihood evaluation, weighting, normalization, and resampling stages recursively transform a population of hypotheses into an updated belief about the physical world. This foundation supports advanced sensor fusion, probabilistic localization, tracking, and robust state estimation within modern robotic perception architectures.

파티클 필터(Particle Filter)는 파티클(Particle)이라고 불리는 이산적인 가설(Discrete Hypothesis)의 집합을 이용하여 불확실한 시스템 상태를 표현하고 추적하는 재귀적 베이지안 추정 방법(Recursive Bayesian Estimation Method)이다. 각 파티클은 시스템이 가질 수 있는 하나의 상태를 나타내며, 해당 상태의 확률을 나타내는 가중치(Weight)를 갖는다. 칼만 필터(Kalman Filter)와 달리 파티클 필터는 상태 불확실성이 가우시안 분포(Gaussian Distribution)를 따르거나 시스템 방정식이 선형(Linear)이어야 한다는 조건을 요구하지 않는다.

상태 분포(State Distribution)는 가능한 상태 공간(State Space)에 분포된 다수의 파티클을 통해 근사된다. 하나의 파티클에는 위치(Position), 속도(Velocity), 자세(Orientation), 센서 바이어스(Sensor Bias), 랜드마크 연관성(Landmark Association) 또는 추정 문제에 필요한 다른 변수들이 포함될 수 있다. 전체 파티클은 시스템 상태의 사후 확률 분포(Posterior Probability Distribution)를 근사하며, 이를 통해 비대칭(Asymmetric), 다중 모드(Multimodal), 또는 강한 비선형(Highly Nonlinear) 불확실성을 표현할 수 있다.

파티클 필터링(Particle Filtering)은 베이지안 추정 원리(Bayesian Estimation Principle)를 기반으로 한다. 추정기는 현재 상태에 대한 믿음(Belief)을 유지하고 시스템이 이동하거나 새로운 측정값이 입력될 때마다 이를 재귀적으로 갱신한다. 이전의 사후 분포(Posterior Distribution)는 다음 추정 주기의 사전 정보(Prior Information)가 된다. 이러한 재귀적 구조는 파티클 필터를 자율 로봇과 기타 동적 시스템의 연속적인 위치추정(Localization) 및 추적(Tracking) 문제에 적합하게 만든다.

예측 단계(Prediction Stage)에서는 모든 파티클을 프로세스 모델(Process Model) 또는 운동 모델(Motion Model)을 통해 전파한다. 이동 로봇에서는 명령 속도(Commanded Velocity), 조향각(Steering Angle), 휠 오도메트리(Wheel Odometry), 관성 측정값(Inertial Measurement), 차량 운동학(Vehicle Kinematics) 등을 사용하여 각 가상 상태가 시간에 따라 어떻게 변화하는지 추정할 수 있다. 전파 과정에는 무작위 프로세스 잡음(Random Process Noise)이 추가되어 운동 오차, 휠 슬립(Wheel Slip), 지형 변화, 모델링되지 않은 동역학(Unmodeled Dynamics)으로 발생하는 불확실성을 파티클 집합에 반영한다.

예측 이후에는 이동 과정에서 불확실성이 누적되기 때문에 일반적으로 파티클이 상태 공간 전체로 퍼지게 된다. 이러한 분산의 크기와 방향은 프로세스 모델과 잡음 분포(Noise Distribution)에 의해 결정된다. 정확도가 높은 운동 모델은 비교적 집중된 파티클 분포를 생성하는 반면, 불확실한 운동은 더 넓은 분산을 만든다. 이 메커니즘을 통해 파티클 필터는 복잡한 불확실성을 하나의 공분산 행렬(Covariance Matrix)로 강제하여 표현하지 않고 직접 나타낼 수 있다.

새로운 센서 관측값(Sensor Observation)이 입력되면 측정 모델(Measurement Model)은 각각의 예측 파티클이 해당 관측값을 얼마나 잘 설명하는지를 평가한다. 측정값은 위성항법시스템(GNSS), 라이다(LiDAR), 레이더(Radar), 카메라(Camera), 휠 인코더(Wheel Encoder) 또는 기타 인지 센서(Perception Sensor)에서 얻을 수 있다. 각 파티클에 대응하는 예측 관측값(Predicted Observation)을 실제 측정값과 비교하고, 두 값의 일치 정도를 나타내는 우도값(Likelihood)을 계산한다.

우도값(Likelihood)은 파티클 가중치(Particle Weighting)를 결정하는 기준이 된다. 센서 측정값과 높은 일관성을 갖는 파티클에는 큰 가중치가 부여되고, 관측값을 제대로 설명하지 못하는 파티클에는 작은 가중치가 부여된다. 정규화(Normalization) 이후 각 가중치는 개별 가설의 상대적인 확률을 근사한다. 이렇게 생성된 가중 파티클 집합(Weighted Particle Population)은 최신 센서 정보를 반영한 갱신된 사후 분포를 나타낸다.

파티클 필터링의 핵심 연산 중 하나는 재표본추출(Resampling)이다. 재표본추출 과정에서는 높은 가중치를 갖는 파티클이 다음 파티클 집합을 구성하기 위해 더 자주 선택되고, 매우 낮은 가중치를 갖는 파티클은 점차 제거된다. 이를 통해 계산 자원(Computational Resource)을 관측값에 의해 지지되는 상태 공간 영역에 집중시킬 수 있다. 새롭게 구성된 파티클 집합은 다음 예측 및 측정 주기의 시작 분포가 된다.

반복적인 재표본추출은 파티클 퇴화(Particle Degeneracy) 또는 표본 빈곤화(Sample Impoverishment)를 발생시킬 수 있다. 퇴화는 대부분의 파티클이 거의 무시할 수 있는 가중치를 갖고 소수의 파티클만 분포를 지배하는 현상이다. 표본 빈곤화는 높은 가중치를 갖는 파티클이 반복적으로 복제되면서 다양성이 감소하는 현상이다. 따라서 효과적인 구현에서는 파티클 품질을 모니터링하고 적응형 재표본추출(Adaptive Resampling), 추가 잡음, 개선된 제안 분포(Proposal Distribution) 등의 방법으로 유효한 가설의 다양성을 유지한다.

유효 표본 크기(Effective Sample Size)는 일반적으로 재표본추출의 필요성을 판단하는 데 사용된다. 모든 측정값이 입력될 때마다 재표본추출을 수행하는 대신, 추정기는 정규화된 파티클 가중치가 얼마나 특정 파티클에 집중되었는지를 평가할 수 있다. 많은 파티클이 여전히 의미 있는 기여를 하고 있다면 재표본추출을 연기할 수 있다. 반대로 소수의 파티클이 대부분의 가중치를 차지한다면 계산 자원을 더 가능성 높은 상태 영역으로 재분배하기 위해 재표본추출을 수행할 수 있다.

파티클 수(Particle Count)는 추정 품질과 계산 비용 모두에 큰 영향을 미친다. 파티클 수가 너무 적으면 특히 상태 공간이 크거나 확률 분포에 여러 모드가 존재하는 경우 중요한 가설을 제대로 표현하지 못할 수 있다. 파티클 수를 증가시키면 일반적으로 분포의 표현 범위가 향상되지만 예측, 측정 평가, 정규화, 재표본추출에 필요한 계산량도 증가한다. 따라서 실시간 시스템(Real-Time System)에서는 정확도와 사용 가능한 컴퓨팅 자원 사이의 균형이 필요하다.

파티클 필터링의 주요 장점 중 하나는 여러 가설(Multiple Hypotheses)을 동시에 유지할 수 있다는 점이다. 예를 들어 로봇 위치추정 과정에서 서로 비슷한 환경 특징 때문에 여러 위치가 동시에 올바른 후보로 보일 수 있다. 가우시안 추정기(Gaussian Estimator)는 서로 분리된 이러한 가능성을 표현하는 데 어려움이 있을 수 있지만, 파티클 필터는 이후 관측값이 잘못된 가설을 제거할 만큼 충분한 정보를 제공할 때까지 여러 위치에 파티클 클러스터(Particle Cluster)를 유지할 수 있다.

이러한 특성은 전역 위치추정(Global Localization)과 납치된 로봇 복구(Kidnapped-Robot Recovery)에 특히 유용하다. 로봇의 초기 자세(Initial Robot Pose)를 알 수 없는 경우 하나의 추정 위치 주변에 파티클을 초기화하는 대신 넓은 지도 영역 전체에 분포시킬 수 있다. 라이다, 카메라, 위성항법시스템 또는 기타 관측값이 입력되면 일치하지 않는 가설의 가중치는 감소하고 실제 자세 근처의 파티클이 우세해진다. 따라서 추정기는 상당한 초기 불확실성이 존재하는 상황에서도 실제 상태로 수렴할 수 있다.

파티클 필터는 비선형 운동 모델(Nonlinear Motion Model)과 비선형 측정 모델(Nonlinear Measurement Model)에도 적합하다. 차량의 회전 동역학(Turning Dynamics), 거리 관측(Range Observation), 방위각 측정(Bearing Measurement), 장애물 형상(Obstacle Geometry), 시각 인지(Visual Perception)는 물리적 상태와 센서 관측 사이에 비선형 관계를 만들 수 있다. 파티클은 이러한 모델을 직접 통과하여 전파되므로 확장 칼만 필터(Extended Kalman Filter, EKF)에서 필요한 야코비안 계산(Jacobian Calculation)과 같은 명시적인 국소 선형화(Local Linearization)가 일반적으로 필요하지 않다.

파티클 필터링의 유연성에는 상당한 계산 비용(Computational Cost)이 따른다. 모든 파티클을 전파하고 측정값에 대해 평가해야 하며, 복잡한 센서 모델은 우도 계산(Likelihood Computation)을 매우 많은 연산이 필요한 작업으로 만들 수 있다. 수천 개의 파티클에 대해 수행되는 라이다 스캔 매칭(LiDAR Scan Matching)이나 이미지 기반 우도 평가(Image-Based Likelihood Evaluation)는 전체 처리 시간의 대부분을 차지할 수 있다. 따라서 효율적인 구현에서는 축소된 표현(Reduced Representation), 병렬 처리(Parallel Processing), 계층적 추정(Hierarchical Estimation), 최적화된 관측 모델 등을 활용한다.

측정 모델링(Measurement Modeling)은 필터 성능에 결정적인 영향을 미친다. 이론적으로 강력한 파티클 필터도 우도 함수(Likelihood Function)가 실제 센서 동작을 정확하게 표현하지 못하면 실패할 수 있다. 측정 잡음(Measurement Noise), 환경적 모호성(Environmental Ambiguity), 가림(Occlusion), 다중경로 효과(Multipath Effect), 오검출(False Detection), 지도 오차(Map Error), 운용 조건 변화 등을 적절하게 반영해야 한다. 지나치게 날카로운 우도 모델은 유효한 가설을 제거할 수 있고, 지나치게 넓은 모델은 필터의 수렴을 방해할 수 있다.

추정기가 복잡한 불확실성을 표현할 수 있더라도 시간 동기화(Time Synchronization)는 여전히 중요하다. 특정 타임스탬프(Timestamp)까지 예측된 파티클은 동일한 물리적 시점에 해당하는 측정값과 비교되어야 한다. 카메라 처리 지연(Delayed Camera Processing), 라이다 스캔 시간(LiDAR Scan Duration), 위성항법시스템 지연(GNSS Latency), 레이더 타임스탬프(Radar Timestamp), 비동기 관성측정장치 업데이트(Asynchronous IMU Update) 등이 제대로 처리되지 않으면 체계적인 우도 오차(Systematic Likelihood Error)가 발생할 수 있다. 따라서 타임스탬프 관리와 지연 보상(Latency Compensation)은 실제 파티클 필터 구조의 필수 요소이다.

파티클 필터는 다른 추정 방법을 완전히 대체하기보다 함께 사용할 수도 있다. 시스템은 전역 위치추정에는 파티클 필터를 사용하고 고주파 국부 운동 추정(High-Rate Local Motion Estimation)에는 확장 칼만 필터를 사용할 수 있다. 파티클 기반 추정(Particle-Based Estimation)이 여러 전역 가설을 해결한 이후 가우시안 추정기(Gaussian Estimator)가 효율적인 연속 추적을 담당할 수 있다. 이러한 하이브리드 구조(Hybrid Architecture)는 서로 다른 확률적 추정 기법(Probabilistic Estimation Technique)의 상호보완적인 장점을 활용한다.

자율이동로봇(AMR), 자율주행차(Autonomous Vehicle), 무인항공기(UAV) 구조에서 파티클 필터링은 위치추정, 객체 추적(Object Tracking), 지형 상태 추정(Terrain-State Estimation), 고장 진단(Fault Diagnosis), 그리고 모호하거나 비가우시안 불확실성(Non-Gaussian Uncertainty)을 포함하는 다양한 문제를 지원할 수 있다. 특히 센서 관측값이 여러 개의 가능한 설명을 생성하거나 작은 오차를 보정하는 수준을 넘어 심각한 위치추정 실패로부터 시스템이 복구해야 하는 상황에서 유용하다.

파티클 필터의 출력은 최고 가중치 파티클(Highest-Weight Particle), 가중 평균(Weighted Mean), 지배적인 파티클 클러스터(Dominant Particle Cluster), 또는 전체 확률 분포(Complete Probability Distribution)로 표현할 수 있다. 어떤 표현을 사용할지는 하위 응용 시스템(Downstream Application)에 따라 달라진다. 경로 계획(Planning)과 제어(Control)는 하나의 자세 추정값을 필요로 할 수 있지만, 안전 모니터링(Safety Monitoring)이나 의사결정(Decision-Making)은 여러 가설과 그 사이에 분포된 불확실성 정보를 활용할 수 있다.

피지컬 AI 시스템(Physical AI System)에서 파티클 필터링은 하나의 평균(Mean)과 공분산(Covariance)만으로 충분히 표현하기 어려운 물리적 상태의 불확실성을 추론하는 메커니즘을 제공한다. 인지 및 위치추정 시스템은 센서 증거가 충분히 강해질 때까지 서로 다른 대안적 설명(Alternative Explanation)을 유지할 수 있다. 이러한 능력은 하나의 가설을 성급하게 선택하면 잘못된 의사결정이 발생할 수 있는 동적이고 부분 관측 가능한 환경(Partially Observable Environment)과 모호한 환경에서 특히 중요하다.

따라서 파티클 필터(Particle Filter)는 단순한 또 하나의 잡음 필터링 기법(Noise-Filtering Technique)이 아니라 표본 기반 베이지안 추정 프레임워크(Sampling-Based Bayesian Estimation Framework)로 이해해야 한다. 예측(Prediction), 우도 평가(Likelihood Evaluation), 가중치 부여(Weighting), 정규화(Normalization), 재표본추출(Resampling) 단계는 가설 집합을 물리적 세계에 대한 갱신된 믿음으로 재귀적으로 변환한다. 이러한 기반은 현대 로봇 인지 구조(Robotic Perception Architecture)의 고급 센서 융합(Advanced Sensor Fusion), 확률적 위치추정(Probabilistic Localization), 추적(Tracking), 강건한 상태 추정(Robust State Estimation)을 지원한다.

##  

## 10.03. Sensor Fusion Middleware

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

Sensor fusion middleware is the software layer that connects heterogeneous sensors with estimation, perception, localization, and decision-making algorithms. Rather than allowing every fusion algorithm to communicate directly with individual sensor drivers, middleware provides standardized interfaces for data transport, timestamps, coordinate frames, metadata, quality information, and system status. This separation improves modularity and allows sensing hardware or fusion algorithms to evolve independently.

A modern robotic platform may simultaneously use cameras, LiDARs, radar, GNSS, IMUs, wheel encoders, ultrasonic sensors, and specialized inspection sensors. These devices generate fundamentally different data types at different rates and resolutions. Sensor fusion middleware provides a common integration environment in which raw measurements can be received, validated, transformed, synchronized, buffered, and distributed to multiple consumers without tightly coupling application software to sensor-specific interfaces.

The middleware architecture normally separates sensor acquisition from fusion processing. Device drivers communicate with physical sensors through interfaces such as Ethernet, CAN, CAN FD, SPI, UART, USB, or dedicated camera links and convert hardware-specific messages into standardized software representations. Higher-level fusion components can then process these representations without needing detailed knowledge of the underlying electrical interface, packet structure, or manufacturer-specific communication protocol.

Standardized message definitions are essential because fusion algorithms depend on consistent interpretation of sensor data. A message should identify not only the measurement itself but also its timestamp, coordinate frame, sequence information, covariance or uncertainty when available, and relevant sensor status. For example, an IMU message may contain acceleration and angular velocity, while a GNSS message may provide position, fix quality, covariance, and timing information required for navigation fusion.

Time synchronization is one of the most important middleware responsibilities. Measurements generated by different sensors must be associated with the physical time at which they were captured rather than simply the time at which software received them. Hardware timestamps, Precision Time Protocol (PTP), GNSS-derived time, trigger signals, and synchronized system clocks can establish a common temporal reference from sensors through computing nodes.

Middleware must also handle sensors operating at very different frequencies. An IMU may produce hundreds or thousands of measurements per second, while GNSS may update at a much lower rate and cameras or LiDARs may generate larger frames at intermediate frequencies. Fusion software therefore requires asynchronous data handling, queues, timestamp-based association, interpolation, and sometimes state propagation so that observations can be combined at meaningful temporal reference points.

Coordinate-frame management provides the spatial equivalent of time synchronization. Each sensor measures the environment relative to its own physical mounting frame, while navigation and perception algorithms may operate in robot, vehicle, odometry, map, or global coordinate frames. Middleware should provide consistent transformation relationships derived from sensor extrinsic calibration so that measurements from physically separated devices can be interpreted within a common geometric reference.

In ROS 2-based robotic architectures, middleware commonly uses publish-subscribe communication to decouple data producers from consumers. Sensor drivers publish standardized messages, while localization, perception, mapping, monitoring, recording, and visualization components subscribe to the information they require. This architecture allows one sensor stream to support multiple applications simultaneously and enables components to be replaced without redesigning the complete communication path.

Data Distribution Service (DDS) provides the communication foundation for many ROS 2 deployments and supports configurable Quality of Service (QoS) policies. Reliability, history depth, durability, deadline, lifespan, and delivery behavior can be selected according to the characteristics of each sensor stream. High-bandwidth camera or LiDAR data may require different communication policies from low-bandwidth safety status, GNSS information, or control-related state estimates.

Middleware design must consider bandwidth because modern perception sensors can generate substantial data volumes. Multiple high-resolution cameras and 3D LiDARs can place significant loads on Ethernet links, memory bandwidth, CPU resources, and inter-process communication. Unnecessary serialization and copying can increase latency and processor utilization. Shared memory, zero-copy transport, hardware acceleration, and efficient message layouts can therefore become important for high-performance robotic platforms.

Buffer management is closely related to latency and synchronization. Fusion algorithms often require measurements from nearby timestamps, which means middleware must retain limited histories of sensor messages. Buffers that are too small can discard useful observations before corresponding data arrives, while excessively large queues increase memory consumption and may cause stale information to propagate through the system. Queue depth should therefore reflect sensor frequency, jitter, processing delay, and fusion requirements.

Sensor fusion middleware should preserve uncertainty information rather than transporting only nominal measurement values. Covariance matrices, confidence scores, signal quality, GNSS fix status, tracking confidence, or other sensor-specific quality indicators can help downstream estimators determine how strongly each observation should influence the fused state. This information is particularly important for Kalman filters, particle filters, probabilistic localization, and fault-aware fusion algorithms.

Validation can be performed at several middleware boundaries before data reaches the estimator. Messages can be checked for invalid timestamps, missing fields, impossible ranges, communication corruption, excessive delay, sequence discontinuities, or abnormal sensor status. Detecting these conditions early prevents malformed or stale measurements from silently entering the fusion pipeline and provides diagnostic information that can be used by monitoring and safety functions.

Fault isolation is especially important in autonomous systems because a sensor may continue transmitting syntactically valid data even when its measurements are physically incorrect. Middleware can expose health states, timeout conditions, update-rate deviations, synchronization errors, and confidence degradation to higher-level fault-tolerant fusion logic. The fusion system can then reduce the influence of a degraded sensor, reject its observations, or transition to an alternative sensing configuration.

Latency should be treated as an end-to-end architectural property rather than only as network transmission time. Sensor exposure or scanning, internal device processing, communication, driver execution, serialization, queueing, fusion computation, and scheduling all contribute to the age of information. Middleware instrumentation should therefore measure timestamps at important processing boundaries so that engineers can determine where delay and jitter are introduced.

Deterministic behavior becomes increasingly important when fused state estimates are consumed by motion control or safety-related functions. General perception pipelines may tolerate moderate timing variation, but high-speed control and safety monitoring require bounded latency and predictable execution. Real-time operating-system configuration, thread priorities, CPU affinity, deterministic networking, and carefully controlled middleware queues can improve temporal behavior when the application requires stronger guarantees.

A scalable middleware architecture should support distributed computing because sensor processing may span several compute devices. An AMR can use an embedded controller for vehicle interfaces, a Jetson-class computer for perception, and an edge PC or GPU platform for advanced AI processing. Middleware must maintain consistent message definitions, timestamps, coordinate frames, and communication behavior across these computing boundaries while avoiding unnecessary duplication of high-bandwidth data.

Recording and replay are also fundamental middleware capabilities for development and validation. Sensor streams, timestamps, transforms, system states, and diagnostic information can be recorded during real operation and replayed later through the same interfaces used by live sensors. This enables repeatable algorithm development, regression testing, fault investigation, dataset construction, and comparison of alternative fusion algorithms without requiring the physical robot for every experiment.

Simulation should use interfaces that closely resemble those of real sensors whenever practical. If simulated cameras, LiDARs, GNSS receivers, IMUs, and vehicle states publish compatible messages, the same fusion components can operate in simulation and on physical hardware with limited modification. This supports software-in-the-loop and hardware-in-the-loop workflows while reducing the architectural gap between development, verification, and deployment environments.

Configuration management is necessary because sensor fusion behavior depends on calibration parameters, coordinate transformations, noise models, update rates, network settings, and algorithm-specific parameters. Middleware should provide controlled mechanisms for loading and versioning these values. Recording configuration versions together with datasets and test results improves reproducibility and helps engineers determine whether performance changes originate from software, calibration, hardware, or parameter modifications.

For Physical AI systems, sensor fusion middleware forms the operational bridge between physical sensing and machine intelligence. It converts independently operating sensors into a temporally and spatially coherent information infrastructure from which localization, mapping, perception, planning, control, safety monitoring, and world modeling can obtain reliable inputs. Its quality directly influences whether higher-level AI receives a consistent representation of the physical environment.

Sensor fusion middleware should therefore be considered part of the perception architecture rather than merely a communication utility. Reliable transport, common timestamps, coordinate transforms, uncertainty metadata, buffering, QoS, diagnostics, recording, and fault handling collectively determine the integrity of information reaching fusion algorithms. A well-designed middleware layer allows Kalman filters, particle filters, and advanced Physical AI models to operate on synchronized, traceable, and trustworthy sensor information.

센서 융합 미들웨어(Sensor Fusion Middleware)는 서로 다른 종류의 센서를 추정(Estimation), 인지(Perception), 위치추정(Localization), 의사결정(Decision-Making) 알고리즘과 연결하는 소프트웨어 계층(Software Layer)이다. 각각의 융합 알고리즘이 개별 센서 드라이버와 직접 통신하도록 하는 대신, 미들웨어는 데이터 전송, 타임스탬프(Timestamp), 좌표 프레임(Coordinate Frame), 메타데이터(Metadata), 품질 정보(Quality Information), 시스템 상태(System Status)를 위한 표준화된 인터페이스를 제공한다. 이러한 분리는 모듈성(Modularity)을 향상시키며 센싱 하드웨어와 융합 알고리즘이 서로 독립적으로 발전할 수 있도록 한다.

현대적인 로봇 플랫폼(Robotic Platform)은 카메라(Camera), 라이다(LiDAR), 레이더(Radar), 위성항법시스템(GNSS), 관성측정장치(IMU), 휠 인코더(Wheel Encoder), 초음파 센서(Ultrasonic Sensor), 특수 점검 센서(Specialized Inspection Sensor)를 동시에 사용할 수 있다. 이러한 장치는 서로 다른 주기와 해상도로 근본적으로 다른 유형의 데이터를 생성한다. 센서 융합 미들웨어는 원시 측정값(Raw Measurement)을 수신하고 검증하며 변환, 동기화, 버퍼링(Buffering)한 후 여러 소비자(Consumer)에게 전달할 수 있는 공통 통합 환경(Common Integration Environment)을 제공한다.

미들웨어 구조(Middleware Architecture)는 일반적으로 센서 획득(Sensor Acquisition)과 융합 처리(Fusion Processing)를 분리한다. 장치 드라이버(Device Driver)는 이더넷(Ethernet), CAN, CAN FD, SPI, UART, USB 또는 전용 카메라 링크(Dedicated Camera Link)와 같은 인터페이스를 통해 물리적 센서와 통신하고 하드웨어별 메시지를 표준화된 소프트웨어 표현(Standardized Software Representation)으로 변환한다. 상위 수준 융합 구성요소는 기본 전기 인터페이스, 패킷 구조 또는 제조사별 통신 프로토콜에 대한 세부 지식 없이 이러한 표현을 처리할 수 있다.

표준화된 메시지 정의(Standardized Message Definition)는 융합 알고리즘이 센서 데이터를 일관되게 해석하기 위해 필수적이다. 메시지는 측정값 자체뿐만 아니라 타임스탬프, 좌표 프레임, 시퀀스 정보(Sequence Information), 사용 가능한 경우 공분산(Covariance) 또는 불확실성(Uncertainty), 관련 센서 상태를 식별해야 한다. 예를 들어 관성측정장치 메시지는 가속도와 각속도(Angular Velocity)를 포함할 수 있으며, 위성항법시스템 메시지는 내비게이션 융합(Navigation Fusion)에 필요한 위치, 측위 품질(Fix Quality), 공분산, 시간 정보를 제공할 수 있다.

시간 동기화(Time Synchronization)는 가장 중요한 미들웨어 책임 중 하나이다. 서로 다른 센서에서 생성된 측정값은 소프트웨어가 이를 수신한 시간이 아니라 실제로 측정이 이루어진 물리적 시간(Physical Time)을 기준으로 연결되어야 한다. 하드웨어 타임스탬프(Hardware Timestamp), 정밀 시간 프로토콜(Precision Time Protocol, PTP), 위성항법시스템 기반 시간(GNSS-Derived Time), 트리거 신호(Trigger Signal), 동기화된 시스템 클록(Synchronized System Clock)을 사용하여 센서에서 컴퓨팅 노드까지 공통 시간 기준(Common Temporal Reference)을 구축할 수 있다.

미들웨어는 매우 서로 다른 주파수로 동작하는 센서도 처리해야 한다. 관성측정장치는 초당 수백 또는 수천 개의 측정값을 생성할 수 있지만 위성항법시스템은 훨씬 낮은 주기로 갱신될 수 있으며, 카메라나 라이다는 중간 주기에서 더 큰 프레임(Frame)을 생성할 수 있다. 따라서 융합 소프트웨어에는 비동기 데이터 처리(Asynchronous Data Handling), 큐(Queue), 타임스탬프 기반 연관(Timestamp-Based Association), 보간(Interpolation), 경우에 따라 상태 전파(State Propagation)가 필요하며 이를 통해 의미 있는 시간 기준점에서 관측값을 결합할 수 있다.

좌표 프레임 관리(Coordinate-Frame Management)는 시간 동기화에 대응하는 공간적 기능을 제공한다. 각각의 센서는 자체적인 물리적 장착 프레임(Mounting Frame)을 기준으로 환경을 측정하지만 내비게이션과 인지 알고리즘은 로봇(Robot), 차량(Vehicle), 오도메트리(Odometry), 지도(Map), 전역 좌표(Global Coordinate) 프레임에서 동작할 수 있다. 미들웨어는 센서 외부 파라미터 보정(Sensor Extrinsic Calibration)으로부터 얻어진 일관된 변환 관계(Transformation Relationship)를 제공하여 물리적으로 떨어져 장착된 장치의 측정값을 공통 기하학적 기준(Common Geometric Reference)에서 해석할 수 있도록 해야 한다.

ROS 2 기반 로봇 구조(ROS 2-Based Robotic Architecture)에서 미들웨어는 일반적으로 발행-구독 통신(Publish-Subscribe Communication)을 사용하여 데이터 생산자(Data Producer)와 소비자(Data Consumer)를 분리한다. 센서 드라이버는 표준화된 메시지를 발행(Publish)하고 위치추정, 인지, 매핑(Mapping), 모니터링(Monitoring), 기록(Recording), 시각화(Visualization) 구성요소는 필요한 정보를 구독(Subscribe)한다. 이러한 구조는 하나의 센서 스트림(Sensor Stream)을 여러 응용 프로그램이 동시에 사용할 수 있게 하며 전체 통신 경로를 다시 설계하지 않고도 구성요소를 교체할 수 있도록 한다.

데이터 분산 서비스(Data Distribution Service, DDS)는 많은 ROS 2 시스템에서 통신 기반을 제공하며 설정 가능한 서비스 품질(Quality of Service, QoS) 정책을 지원한다. 신뢰성(Reliability), 이력 깊이(History Depth), 지속성(Durability), 데드라인(Deadline), 수명(Lifespan), 전달 동작(Delivery Behavior)을 각 센서 스트림의 특성에 따라 선택할 수 있다. 고대역폭 카메라 또는 라이다 데이터는 저대역폭 안전 상태, 위성항법시스템 정보, 제어 관련 상태 추정값과 서로 다른 통신 정책을 요구할 수 있다.

현대적인 인지 센서가 상당한 데이터 양을 생성할 수 있으므로 미들웨어 설계에서는 대역폭(Bandwidth)을 고려해야 한다. 여러 고해상도 카메라와 3차원 라이다(3D LiDAR)는 이더넷 링크, 메모리 대역폭(Memory Bandwidth), 중앙처리장치 자원(CPU Resource), 프로세스 간 통신(Inter-Process Communication)에 상당한 부하를 줄 수 있다. 불필요한 직렬화(Serialization)와 데이터 복사는 지연시간(Latency)과 프로세서 사용률을 증가시킨다. 따라서 고성능 로봇 플랫폼에서는 공유 메모리(Shared Memory), 무복사 전송(Zero-Copy Transport), 하드웨어 가속(Hardware Acceleration), 효율적인 메시지 구조가 중요해질 수 있다.

버퍼 관리(Buffer Management)는 지연시간과 동기화에 밀접하게 연관되어 있다. 융합 알고리즘은 종종 서로 가까운 타임스탬프의 측정값을 필요로 하므로 미들웨어는 제한된 범위의 센서 메시지 이력(Message History)을 유지해야 한다. 버퍼가 너무 작으면 대응하는 데이터가 도착하기 전에 유용한 관측값이 삭제될 수 있고, 지나치게 큰 큐는 메모리 소비를 증가시키고 오래된 정보(Stale Information)가 시스템을 통해 전달되도록 만들 수 있다. 따라서 큐 깊이(Queue Depth)는 센서 주파수, 지터(Jitter), 처리 지연, 융합 요구사항을 반영해야 한다.

센서 융합 미들웨어는 명목상의 측정값(Nominal Measurement Value)만 전송하는 것이 아니라 불확실성 정보도 보존해야 한다. 공분산 행렬(Covariance Matrix), 신뢰도 점수(Confidence Score), 신호 품질(Signal Quality), 위성항법시스템 측위 상태(GNSS Fix Status), 추적 신뢰도(Tracking Confidence), 기타 센서별 품질 지표는 하위 추정기가 각각의 관측값을 융합 상태에 어느 정도 반영해야 하는지 결정하는 데 도움을 준다. 이러한 정보는 칼만 필터(Kalman Filter), 파티클 필터(Particle Filter), 확률적 위치추정(Probabilistic Localization), 고장 인지형 융합 알고리즘(Fault-Aware Fusion Algorithm)에서 특히 중요하다.

데이터가 추정기에 도달하기 전에 여러 미들웨어 경계(Middleware Boundary)에서 검증(Validation)을 수행할 수 있다. 메시지의 잘못된 타임스탬프, 누락된 필드(Missing Field), 물리적으로 불가능한 범위, 통신 손상(Communication Corruption), 과도한 지연, 시퀀스 불연속(Sequence Discontinuity), 비정상적인 센서 상태 등을 검사할 수 있다. 이러한 상태를 조기에 검출하면 잘못되거나 오래된 측정값이 융합 파이프라인(Fusion Pipeline)에 조용히 유입되는 것을 방지하고 모니터링 및 안전 기능에서 사용할 수 있는 진단 정보를 제공한다.

자율 시스템(Autonomous System)에서는 센서가 문법적으로 정상적인 데이터를 계속 전송하면서도 물리적으로 잘못된 측정값을 생성할 수 있기 때문에 고장 격리(Fault Isolation)가 특히 중요하다. 미들웨어는 상태 정보(Health State), 타임아웃 조건(Timeout Condition), 업데이트 주기 편차(Update-Rate Deviation), 동기화 오류(Synchronization Error), 신뢰도 저하(Confidence Degradation)를 상위 고장 허용 융합 로직(Fault-Tolerant Fusion Logic)에 제공할 수 있다. 융합 시스템은 이를 이용해 성능이 저하된 센서의 영향을 줄이거나 관측값을 거부하거나 대체 센싱 구성으로 전환할 수 있다.

지연시간은 단순한 네트워크 전송 시간이 아니라 종단간 구조적 특성(End-to-End Architectural Property)으로 다루어야 한다. 센서 노출 또는 스캐닝(Sensor Exposure or Scanning), 장치 내부 처리, 통신, 드라이버 실행, 직렬화, 큐잉(Queueing), 융합 연산, 스케줄링(Scheduling)이 모두 정보의 시간적 노후도(Age of Information)에 영향을 준다. 따라서 미들웨어 계측(Middleware Instrumentation)은 주요 처리 경계에서 타임스탬프를 측정하여 엔지니어가 지연과 지터가 발생하는 위치를 파악할 수 있도록 해야 한다.

융합된 상태 추정값이 모션 제어(Motion Control) 또는 안전 관련 기능에서 사용되는 경우 결정론적 동작(Deterministic Behavior)의 중요성이 더욱 커진다. 일반적인 인지 파이프라인은 어느 정도의 시간 변동을 허용할 수 있지만 고속 제어와 안전 모니터링에는 제한된 지연시간(Bounded Latency)과 예측 가능한 실행이 필요하다. 실시간 운영체제 설정(Real-Time Operating-System Configuration), 스레드 우선순위(Thread Priority), CPU 친화도(CPU Affinity), 결정론적 네트워킹(Deterministic Networking), 정밀하게 제어되는 미들웨어 큐를 통해 더욱 강한 시간적 보장이 필요한 응용 시스템의 동작을 개선할 수 있다.

확장 가능한 미들웨어 구조(Scalable Middleware Architecture)는 센서 처리가 여러 컴퓨팅 장치에 분산될 수 있으므로 분산 컴퓨팅(Distributed Computing)을 지원해야 한다. 자율이동로봇(AMR)은 차량 인터페이스를 위한 임베디드 제어기(Embedded Controller), 인지를 위한 젯슨급 컴퓨터(Jetson-Class Computer), 고급 인공지능 처리를 위한 엣지 PC(Edge PC) 또는 GPU 플랫폼을 사용할 수 있다. 미들웨어는 고대역폭 데이터의 불필요한 복제를 방지하면서 이러한 컴퓨팅 경계 전반에서 일관된 메시지 정의, 타임스탬프, 좌표 프레임, 통신 동작을 유지해야 한다.

기록 및 재생(Recording and Replay)도 개발과 검증을 위한 핵심 미들웨어 기능이다. 실제 운용 과정에서 센서 스트림, 타임스탬프, 변환 정보(Transform), 시스템 상태, 진단 정보를 기록한 후 실제 센서가 사용하는 것과 동일한 인터페이스를 통해 다시 재생할 수 있다. 이를 통해 매번 실제 로봇을 사용하지 않고도 반복 가능한 알고리즘 개발, 회귀 시험(Regression Testing), 고장 분석(Fault Investigation), 데이터셋 구축(Dataset Construction), 대체 융합 알고리즘 비교를 수행할 수 있다.

시뮬레이션(Simulation)은 가능한 경우 실제 센서와 매우 유사한 인터페이스를 사용해야 한다. 시뮬레이션된 카메라, 라이다, 위성항법시스템 수신기, 관성측정장치, 차량 상태가 호환 가능한 메시지를 발행한다면 동일한 융합 구성요소를 제한적인 수정만으로 시뮬레이션과 실제 하드웨어에서 모두 사용할 수 있다. 이는 소프트웨어 인 더 루프(Software-in-the-Loop)와 하드웨어 인 더 루프(Hardware-in-the-Loop) 개발을 지원하면서 개발, 검증, 실제 배포 환경 사이의 구조적 차이를 줄여준다.

센서 융합 동작은 보정 파라미터(Calibration Parameter), 좌표 변환, 잡음 모델(Noise Model), 업데이트 주기, 네트워크 설정, 알고리즘별 파라미터에 의존하므로 구성 관리(Configuration Management)가 필요하다. 미들웨어는 이러한 값을 불러오고 버전 관리(Versioning)할 수 있는 통제된 메커니즘을 제공해야 한다. 데이터셋 및 시험 결과와 함께 구성 버전을 기록하면 재현성(Reproducibility)이 향상되며 성능 변화가 소프트웨어, 보정, 하드웨어 또는 파라미터 변경 중 어디에서 발생했는지를 엔지니어가 판단하는 데 도움이 된다.

피지컬 AI 시스템(Physical AI System)에서 센서 융합 미들웨어는 물리적 센싱(Physical Sensing)과 기계 지능(Machine Intelligence)을 연결하는 운용적 가교(Operational Bridge)를 형성한다. 독립적으로 동작하는 센서들을 시간적·공간적으로 일관된 정보 인프라(Information Infrastructure)로 변환하며, 위치추정, 매핑, 인지, 경로 계획(Planning), 제어(Control), 안전 모니터링(Safety Monitoring), 월드 모델링(World Modeling)이 신뢰할 수 있는 입력을 얻도록 한다. 미들웨어의 품질은 상위 수준 인공지능이 물리적 환경에 대한 일관된 표현을 받을 수 있는지를 직접적으로 좌우한다.

따라서 센서 융합 미들웨어는 단순한 통신 유틸리티(Communication Utility)가 아니라 인지 구조(Perception Architecture)의 일부로 이해해야 한다. 신뢰성 있는 전송(Reliable Transport), 공통 타임스탬프, 좌표 변환(Coordinate Transform), 불확실성 메타데이터(Uncertainty Metadata), 버퍼링, 서비스 품질(QoS), 진단(Diagnostics), 기록, 고장 처리는 융합 알고리즘에 도달하는 정보의 무결성(Information Integrity)을 공동으로 결정한다. 잘 설계된 미들웨어 계층은 칼만 필터, 파티클 필터, 고급 피지컬 AI 모델(Advanced Physical AI Model)이 동기화되고 추적 가능하며 신뢰할 수 있는 센서 정보를 기반으로 동작하도록 한다.

##  

## 10.04. Fault Tolerant Fusion

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

Fault-tolerant fusion is a sensor fusion architecture designed to maintain reliable state estimation when one or more sensors become inaccurate, unavailable, delayed, corrupted, or physically inconsistent. Instead of assuming that every incoming measurement is valid, the fusion system continuously evaluates sensor health and measurement credibility. Its objective is to detect abnormal information, isolate its influence, and preserve the best possible estimate using the remaining trustworthy sensing resources.

Autonomous robots depend on multiple sensors because no individual sensing technology remains reliable under every operating condition. GNSS can degrade near buildings or under obstruction, cameras can suffer from darkness or glare, LiDAR can be affected by contamination or adverse weather, radar may generate ambiguous reflections, and wheel odometry can become inaccurate during slip. Fault-tolerant fusion uses this diversity as functional redundancy rather than treating multiple sensors merely as additional data sources.

Sensor faults can appear in several forms and may develop either suddenly or gradually. Complete loss of communication is relatively easy to identify, but subtle failures are more difficult because the sensor may continue transmitting plausible-looking measurements. Bias drift, scale-factor errors, frozen values, timestamp errors, increasing noise, intermittent communication, calibration changes, and environmental degradation can all corrupt fusion results without producing an obvious hardware failure indication.

Fault detection begins by observing both sensor-level status and measurement-level consistency. Middleware can monitor communication timeouts, update rates, sequence numbers, device diagnostics, timestamps, temperature, and internal sensor status. The estimator can independently evaluate whether each observation is statistically compatible with the predicted system state. Combining these two information sources provides stronger fault detection than relying only on a sensor\'s self-reported health status.

Innovation or residual analysis is particularly useful in Kalman-filter-based fusion. The difference between a measured observation and its predicted value indicates whether the measurement agrees with the current state estimate. Residuals that repeatedly exceed statistically expected limits can indicate sensor degradation, incorrect covariance, calibration error, timing problems, or an actual fault. Normalized innovation tests allow different measurement types to be evaluated using comparable statistical criteria.

A single abnormal residual should not automatically be interpreted as a permanent sensor failure. Dynamic motion, temporary occlusion, multipath GNSS reception, unusual terrain, or imperfect models can briefly produce large discrepancies. Practical fault logic therefore considers persistence, frequency, magnitude, operating context, and agreement with other sensors. This prevents unnecessary sensor isolation while still enabling rapid response when evidence of a genuine fault becomes sufficiently strong.

Fault isolation determines which sensing source is responsible after abnormal behavior has been detected. Cross-comparison between redundant or complementary sensors can help distinguish a faulty measurement from legitimate vehicle motion. For example, disagreement between GNSS and the fused trajectory can be compared with IMU, wheel odometry, LiDAR localization, and map constraints. Agreement among several independent sources increases confidence that the inconsistent sensor should be treated as degraded.

Once a sensor is considered unreliable, the fusion system can reduce its influence rather than immediately removing it. Adaptive covariance adjustment increases the measurement uncertainty assigned to suspicious data, causing a probabilistic estimator to trust that observation less strongly. This provides graceful degradation when sensor quality deteriorates gradually. If the fault becomes severe or persistent, the measurement can eventually be rejected or the sensor can be excluded entirely.

Measurement gating provides another protection mechanism. Before an observation is accepted into the estimator, its innovation can be compared with a predefined statistical acceptance region. Measurements outside this region are rejected, delayed for additional validation, or processed with reduced confidence. Gating is especially useful for suppressing isolated GNSS jumps, incorrect object associations, false visual localization results, and other outliers that could abruptly disturb the estimated state.

Redundancy should be designed according to sensing function rather than simply by duplicating identical devices. GNSS, IMU, LiDAR, radar, cameras, and wheel odometry observe different physical properties and fail under different environmental conditions. Diverse redundancy therefore provides stronger resilience against common-cause failures. For example, combining satellite positioning with inertial and environment-relative localization can maintain navigation when one sensing principle becomes temporarily unreliable.

A fault-tolerant architecture can maintain multiple fusion modes corresponding to available sensor combinations. Normal operation may use all sensors, while degraded modes can operate without GNSS, camera localization, LiDAR localization, or wheel odometry. Each mode should define which measurements remain available, how uncertainty changes, and what operational capability can still be supported. Mode transitions should be controlled and observable rather than occurring as undocumented algorithmic side effects.

State uncertainty must increase when information sources are lost. A fusion system should never continue reporting the same confidence after an important sensor becomes unavailable unless equivalent information is provided elsewhere. Covariance growth or another explicit uncertainty representation communicates this degradation to planning, control, and safety functions. Higher-level systems can then reduce speed, increase safety margins, restrict maneuvers, or request a controlled stop when estimation quality becomes insufficient.

Fault recovery is as important as fault detection. A sensor that temporarily degraded may later return to normal operation, but immediately restoring full influence can introduce discontinuities or reintroduce an unresolved fault. Recovery logic can require a period of stable measurements, successful consistency checks, valid synchronization, and acceptable diagnostic status before reintegration. The sensor weight can then be increased gradually to produce a controlled transition back to normal fusion.

Time faults deserve special attention because otherwise accurate measurements can become dangerous when associated with the wrong physical time. Clock drift, delayed packets, timestamp corruption, synchronization loss, or excessive processing latency can make spatially correct data inconsistent with the current robot state. Fault-tolerant fusion should therefore monitor timestamp validity, message age, synchronization quality, and latency in addition to checking measurement values themselves.

Coordinate-frame and calibration faults can similarly create systematic inconsistencies. A shifted LiDAR mounting position, incorrect camera extrinsic calibration, IMU alignment error, or wrong transform configuration may cause measurements to disagree continuously even though each sensor is internally healthy. Fusion diagnostics should distinguish between device failure and transformation or calibration failure because the appropriate recovery strategy and maintenance action are different.

Fault-tolerant fusion middleware should distribute sensor-health information together with measurement data. Health states may indicate normal, degraded, unavailable, recovering, or isolated operation and can include confidence, fault codes, synchronization status, and data age. This information enables localization, perception, planning, control, safety monitoring, recording, and fleet diagnostics to share a consistent understanding of the sensing system\'s current capability.

Kalman filters can support fault tolerance through innovation monitoring, adaptive covariance, measurement gating, and dynamic sensor inclusion. Particle filters can reduce the likelihood associated with inconsistent observations or change measurement models when sensing conditions deteriorate. More advanced systems may operate parallel estimators using different sensor subsets and compare their outputs, providing additional evidence for fault detection and isolation when computational resources permit.

Graceful degradation is a central design objective. The system should not move directly from fully autonomous operation to complete failure whenever a single sensor is lost. Instead, available functionality should decrease according to the remaining sensing capability and estimation confidence. An outdoor AMR losing RTK corrections, for example, may continue temporarily using inertial, odometry, and LiDAR information while operating with reduced speed and increased uncertainty.

Safety-related decisions should be separated from the estimator\'s desire to continue producing an output. A fusion algorithm can mathematically generate a state estimate even when uncertainty has become too large for safe autonomous operation. Safety supervision must therefore evaluate estimation integrity and determine whether the resulting state remains suitable for the current mission. If required confidence cannot be maintained, the system should transition toward a defined minimal-risk behavior.

Diagnostics and recording are essential for validating fault-tolerant behavior. Sensor measurements, residuals, covariance values, health states, rejected observations, mode transitions, synchronization status, and fault decisions should be logged with consistent timestamps. These records allow engineers to reconstruct field incidents, verify whether isolation decisions were appropriate, improve thresholds, and reproduce failures through recorded-data replay or simulation.

Testing should deliberately introduce sensor faults rather than validating only normal operation. Useful scenarios include sensor disconnection, frozen measurements, bias injection, increased noise, packet delay, timestamp offsets, calibration errors, GNSS degradation, wheel slip, and intermittent communication. Simulation, software-in-the-loop, hardware-in-the-loop, and controlled field testing can verify whether faults are detected, isolated, tolerated, and recovered without unacceptable state-estimation behavior.

For Physical AI systems, fault-tolerant fusion provides an integrity layer between uncertain physical sensing and autonomous intelligence. Planning, control, world modeling, and safety functions should receive not only the estimated state but also information describing how trustworthy that estimate is and which sensors currently support it. This allows autonomous behavior to adapt to sensing capability rather than assuming that perception quality remains constant throughout operation.

Fault-tolerant fusion should therefore be understood as a continuous process of estimation, health assessment, consistency checking, isolation, adaptation, and recovery. Its effectiveness depends on sensor diversity, accurate uncertainty models, synchronized data, middleware diagnostics, robust estimation, and explicit degraded operating modes. Together, these mechanisms enable AMRs, autonomous vehicles, UAVs, and other Physical AI platforms to maintain controlled behavior when real-world sensing inevitably becomes imperfect.

고장 허용 융합(Fault-Tolerant Fusion)은 하나 이상의 센서가 부정확해지거나 사용 불가능해지고, 지연되거나 손상되며, 물리적으로 일관되지 않은 상태가 발생하더라도 신뢰할 수 있는 상태 추정(State Estimation)을 유지하도록 설계된 센서 융합 구조(Sensor Fusion Architecture)이다. 모든 입력 측정값이 유효하다고 가정하는 대신 융합 시스템은 센서 상태(Sensor Health)와 측정 신뢰성(Measurement Credibility)을 지속적으로 평가한다. 목적은 비정상 정보를 검출하고 그 영향을 격리하며, 남아 있는 신뢰 가능한 센싱 자원을 활용하여 가능한 최선의 추정값을 유지하는 것이다.

자율 로봇(Autonomous Robot)은 모든 운용 조건에서 항상 신뢰할 수 있는 단일 센싱 기술이 존재하지 않기 때문에 여러 센서에 의존한다. 위성항법시스템(GNSS)은 건물 주변이나 신호 차단 환경에서 성능이 저하될 수 있고, 카메라는 어둠이나 눈부심의 영향을 받을 수 있으며, 라이다(LiDAR)는 오염이나 악천후의 영향을 받을 수 있다. 레이더(Radar)는 모호한 반사를 생성할 수 있고 휠 오도메트리(Wheel Odometry)는 슬립(Slip)이 발생하면 부정확해질 수 있다. 고장 허용 융합은 다중 센서를 단순한 추가 데이터 소스가 아니라 기능적 중복성(Functional Redundancy)으로 활용한다.

센서 고장(Sensor Fault)은 여러 형태로 나타날 수 있으며 갑작스럽게 또는 점진적으로 발생할 수 있다. 완전한 통신 손실은 비교적 쉽게 식별할 수 있지만 센서가 그럴듯한 측정값을 계속 전송하는 미세한 고장(Subtle Failure)은 탐지하기가 더 어렵다. 바이어스 드리프트(Bias Drift), 스케일 팩터 오류(Scale-Factor Error), 값 고정(Frozen Value), 타임스탬프 오류(Timestamp Error), 잡음 증가, 간헐적 통신, 보정 변화(Calibration Change), 환경적 성능 저하는 명확한 하드웨어 고장 표시 없이도 융합 결과를 손상시킬 수 있다.

고장 검출(Fault Detection)은 센서 수준의 상태와 측정 수준의 일관성을 모두 관찰하는 것에서 시작된다. 미들웨어(Middleware)는 통신 타임아웃, 업데이트 주기, 시퀀스 번호(Sequence Number), 장치 진단(Device Diagnostics), 타임스탬프, 온도, 센서 내부 상태를 모니터링할 수 있다. 추정기(Estimator)는 각 관측값이 예측된 시스템 상태와 통계적으로 일치하는지를 독립적으로 평가할 수 있다. 이러한 두 종류의 정보를 결합하면 센서가 자체적으로 보고하는 상태 정보에만 의존하는 것보다 강력한 고장 검출이 가능하다.

혁신값 또는 잔차 분석(Innovation or Residual Analysis)은 칼만 필터 기반 융합(Kalman-Filter-Based Fusion)에서 특히 유용하다. 측정된 관측값과 예측값 사이의 차이는 해당 측정값이 현재 상태 추정값과 얼마나 일치하는지를 나타낸다. 잔차가 통계적으로 예상되는 한계를 반복적으로 초과하면 센서 성능 저하, 잘못된 공분산(Covariance), 보정 오류, 시간 문제 또는 실제 고장을 의미할 수 있다. 정규화 혁신 검사(Normalized Innovation Test)를 사용하면 서로 다른 측정 유형을 비교 가능한 통계적 기준으로 평가할 수 있다.

한 번의 비정상적인 잔차가 발생했다고 해서 이를 즉시 영구적인 센서 고장으로 판단해서는 안 된다. 동적인 운동, 일시적인 가림(Occlusion), 위성항법시스템 다중경로 수신(Multipath GNSS Reception), 비정상적인 지형 또는 불완전한 모델 때문에 일시적으로 큰 차이가 발생할 수 있다. 따라서 실제 고장 로직(Fault Logic)은 지속성(Persistence), 발생 빈도, 크기, 운용 상황, 다른 센서와의 일치성을 함께 고려한다. 이를 통해 불필요한 센서 격리를 방지하면서 실제 고장에 대한 증거가 충분해지면 신속하게 대응할 수 있다.

고장 격리(Fault Isolation)는 비정상 동작이 검출된 이후 어떤 센싱 소스가 문제의 원인인지를 결정한다. 중복 또는 상호보완적인 센서 사이의 교차 비교(Cross-Comparison)를 통해 잘못된 측정값과 실제 차량 운동을 구별할 수 있다. 예를 들어 위성항법시스템과 융합 궤적(Fused Trajectory) 사이의 불일치는 관성측정장치(IMU), 휠 오도메트리, 라이다 위치추정(LiDAR Localization), 지도 제약(Map Constraint)과 비교할 수 있다. 여러 독립적인 정보원이 서로 일치한다면 불일치하는 센서를 성능 저하 상태로 판단할 신뢰도가 높아진다.

센서가 신뢰할 수 없는 것으로 판단되면 융합 시스템은 즉시 완전히 제거하기보다 해당 센서의 영향력을 감소시킬 수 있다. 적응형 공분산 조정(Adaptive Covariance Adjustment)은 의심스러운 데이터에 할당되는 측정 불확실성을 증가시켜 확률적 추정기(Probabilistic Estimator)가 해당 관측값을 덜 신뢰하도록 한다. 이는 센서 품질이 점진적으로 저하되는 경우 점진적 성능 저하(Graceful Degradation)를 제공한다. 고장이 심각하거나 지속되면 해당 측정값을 최종적으로 거부하거나 센서를 완전히 제외할 수 있다.

측정 게이팅(Measurement Gating)은 또 다른 보호 메커니즘을 제공한다. 관측값을 추정기에 적용하기 전에 혁신값을 미리 정의된 통계적 허용 영역(Statistical Acceptance Region)과 비교할 수 있다. 이 영역을 벗어난 측정값은 거부하거나 추가 검증을 위해 보류하거나 낮은 신뢰도로 처리한다. 게이팅은 일시적인 위성항법시스템 위치 점프, 잘못된 객체 연관(Object Association), 잘못된 비전 위치추정(Visual Localization) 결과, 기타 이상치(Outlier)가 추정 상태를 갑작스럽게 교란하는 것을 억제하는 데 특히 유용하다.

중복성(Redundancy)은 단순히 동일한 장치를 복제하는 방식이 아니라 센싱 기능(Sensing Function)을 기준으로 설계해야 한다. 위성항법시스템, 관성측정장치, 라이다, 레이더, 카메라, 휠 오도메트리는 서로 다른 물리적 특성을 관측하며 환경 조건에 따라 서로 다른 방식으로 고장난다. 따라서 이종 중복성(Diverse Redundancy)은 공통 원인 고장(Common-Cause Failure)에 대해 더 높은 회복력을 제공한다. 예를 들어 위성 기반 위치추정과 관성 및 환경 상대 위치추정(Environment-Relative Localization)을 결합하면 하나의 센싱 원리가 일시적으로 신뢰할 수 없게 되더라도 내비게이션을 유지할 수 있다.

고장 허용 구조(Fault-Tolerant Architecture)는 사용 가능한 센서 조합에 따라 여러 융합 모드(Fusion Mode)를 유지할 수 있다. 정상 운용에서는 모든 센서를 사용하고, 성능 저하 모드(Degraded Mode)에서는 위성항법시스템, 카메라 위치추정, 라이다 위치추정 또는 휠 오도메트리 없이 동작할 수 있다. 각 모드에서는 어떤 측정값을 계속 사용할 수 있는지, 불확실성이 어떻게 변화하는지, 어느 수준의 운용 기능을 유지할 수 있는지를 정의해야 한다. 모드 전환(Mode Transition)은 문서화되지 않은 알고리즘의 부수 효과가 아니라 제어되고 관찰 가능한 방식으로 이루어져야 한다.

정보원이 손실되면 상태 불확실성(State Uncertainty)은 증가해야 한다. 중요한 센서가 사용할 수 없게 되었는데도 동등한 정보가 다른 곳에서 제공되지 않는다면 융합 시스템은 이전과 동일한 신뢰도를 계속 보고해서는 안 된다. 공분산 증가(Covariance Growth) 또는 다른 명시적인 불확실성 표현을 통해 이러한 성능 저하를 경로 계획(Planning), 제어(Control), 안전 기능에 전달할 수 있다. 상위 시스템은 이에 따라 속도를 낮추고, 안전 여유(Safety Margin)를 증가시키며, 기동을 제한하거나 추정 품질이 충분하지 않을 경우 제어된 정지(Controlled Stop)를 요청할 수 있다.

고장 복구(Fault Recovery)는 고장 검출만큼 중요하다. 일시적으로 성능이 저하된 센서가 이후 정상 상태로 복귀할 수 있지만 즉시 전체 영향력을 복원하면 불연속성(Discontinuity)이 발생하거나 해결되지 않은 고장이 다시 유입될 수 있다. 복구 로직(Recovery Logic)은 재통합(Reintegration) 이전에 일정 기간의 안정적인 측정, 성공적인 일관성 검사, 유효한 동기화, 정상적인 진단 상태를 요구할 수 있다. 이후 센서 가중치를 점진적으로 증가시켜 정상 융합으로 제어된 전환을 수행할 수 있다.

시간 관련 고장(Time Fault)은 정확한 측정값이라도 잘못된 물리적 시간과 연결되면 위험해질 수 있으므로 특별한 주의가 필요하다. 클록 드리프트(Clock Drift), 지연된 패킷(Delayed Packet), 타임스탬프 손상, 동기화 손실, 과도한 처리 지연은 공간적으로 정확한 데이터조차 현재 로봇 상태와 불일치하게 만들 수 있다. 따라서 고장 허용 융합은 측정값 자체뿐 아니라 타임스탬프 유효성, 메시지 경과 시간(Message Age), 동기화 품질(Synchronization Quality), 지연시간(Latency)을 함께 모니터링해야 한다.

좌표 프레임(Coordinate Frame)과 보정 관련 고장도 유사하게 체계적인 불일치를 발생시킬 수 있다. 라이다 장착 위치의 변화, 잘못된 카메라 외부 파라미터 보정(Camera Extrinsic Calibration), 관성측정장치 정렬 오류(IMU Alignment Error), 잘못된 변환 설정(Transform Configuration)은 각각의 센서가 내부적으로 정상이어도 측정값 사이에 지속적인 불일치를 발생시킬 수 있다. 적절한 복구 전략과 유지보수 조치가 서로 다르므로 융합 진단은 장치 고장과 변환 또는 보정 고장을 구별해야 한다.

고장 허용 융합 미들웨어(Fault-Tolerant Fusion Middleware)는 측정 데이터와 함께 센서 상태 정보(Sensor-Health Information)를 전달해야 한다. 상태는 정상(Normal), 성능 저하(Degraded), 사용 불가(Unavailable), 복구 중(Recovering), 격리(Isolated) 등을 나타낼 수 있으며 신뢰도, 고장 코드(Fault Code), 동기화 상태, 데이터 경과 시간을 포함할 수 있다. 이를 통해 위치추정, 인지(Perception), 경로 계획, 제어, 안전 모니터링(Safety Monitoring), 기록(Recording), 플릿 진단(Fleet Diagnostics)이 현재 센싱 시스템의 능력에 대한 일관된 정보를 공유할 수 있다.

칼만 필터(Kalman Filter)는 혁신값 모니터링, 적응형 공분산, 측정 게이팅, 동적 센서 포함(Dynamic Sensor Inclusion)을 통해 고장 허용성을 지원할 수 있다. 파티클 필터(Particle Filter)는 일관되지 않은 관측값의 우도(Likelihood)를 낮추거나 센싱 조건이 악화될 때 측정 모델을 변경할 수 있다. 더욱 발전된 시스템에서는 서로 다른 센서 부분집합(Sensor Subset)을 사용하는 여러 추정기를 병렬로 실행하고 출력을 비교하여 계산 자원이 허용하는 범위에서 고장 검출과 격리를 위한 추가 증거를 확보할 수 있다.

점진적 성능 저하(Graceful Degradation)는 핵심적인 설계 목표이다. 하나의 센서가 손실될 때마다 시스템이 완전 자율 운용에서 즉시 전체 고장 상태로 전환되어서는 안 된다. 대신 사용 가능한 기능은 남아 있는 센싱 능력과 추정 신뢰도에 따라 단계적으로 감소해야 한다. 예를 들어 실외 자율이동로봇(Outdoor AMR)이 실시간 이동측위 보정(RTK Correction)을 상실하더라도 관성 정보, 오도메트리, 라이다 정보를 사용하여 속도를 낮추고 불확실성을 증가시킨 상태에서 일정 시간 운용을 지속할 수 있다.

안전 관련 의사결정(Safety-Related Decision)은 추정기가 계속해서 출력값을 생성하려는 동작과 분리되어야 한다. 융합 알고리즘은 불확실성이 안전한 자율 운용에 사용하기에는 지나치게 커진 상황에서도 수학적으로 상태 추정값을 생성할 수 있다. 따라서 안전 감독(Safety Supervision)은 추정 무결성(Estimation Integrity)을 평가하고 결과 상태가 현재 임무에 적합한지를 판단해야 한다. 필요한 신뢰도를 유지할 수 없다면 시스템은 정의된 최소 위험 동작(Minimal-Risk Behavior)으로 전환해야 한다.

진단 및 기록(Diagnostics and Recording)은 고장 허용 동작을 검증하기 위해 필수적이다. 센서 측정값, 잔차, 공분산 값, 상태 정보, 거부된 관측값, 모드 전환, 동기화 상태, 고장 판단을 일관된 타임스탬프와 함께 기록해야 한다. 이러한 기록을 통해 엔지니어는 현장 사고(Field Incident)를 재구성하고 격리 판단이 적절했는지를 검증하며 임계값(Threshold)을 개선하고, 기록 데이터 재생(Recorded-Data Replay) 또는 시뮬레이션을 통해 고장을 재현할 수 있다.

시험(Testing)에서는 정상 운용만 검증하는 것이 아니라 의도적으로 센서 고장을 주입해야 한다. 유용한 시험 시나리오에는 센서 연결 해제, 측정값 고정, 바이어스 주입(Bias Injection), 잡음 증가, 패킷 지연, 타임스탬프 오프셋(Timestamp Offset), 보정 오류, 위성항법시스템 성능 저하, 휠 슬립, 간헐적 통신 등이 포함된다. 시뮬레이션, 소프트웨어 인 더 루프(Software-in-the-Loop), 하드웨어 인 더 루프(Hardware-in-the-Loop), 통제된 현장 시험을 통해 고장이 허용할 수 없는 상태 추정 동작 없이 검출, 격리, 허용, 복구되는지를 검증할 수 있다.

피지컬 AI 시스템(Physical AI System)에서 고장 허용 융합은 불확실한 물리적 센싱과 자율 지능(Autonomous Intelligence) 사이에 무결성 계층(Integrity Layer)을 제공한다. 경로 계획, 제어, 월드 모델링(World Modeling), 안전 기능은 단순한 상태 추정값뿐 아니라 해당 추정값을 얼마나 신뢰할 수 있는지와 현재 어떤 센서가 이를 지원하고 있는지에 대한 정보도 함께 받아야 한다. 이를 통해 자율 동작은 인지 품질이 항상 일정하다고 가정하는 대신 실제 센싱 능력에 맞추어 적응할 수 있다.

따라서 고장 허용 융합(Fault-Tolerant Fusion)은 추정, 상태 평가(Health Assessment), 일관성 검사(Consistency Checking), 격리, 적응(Adaptation), 복구가 지속적으로 이루어지는 과정으로 이해해야 한다. 그 효과는 센서 다양성(Sensor Diversity), 정확한 불확실성 모델, 동기화된 데이터, 미들웨어 진단, 강건한 추정(Robust Estimation), 명시적인 성능 저하 운용 모드에 달려 있다. 이러한 메커니즘을 결합하면 자율이동로봇(AMR), 자율주행차(Autonomous Vehicle), 무인항공기(UAV), 기타 피지컬 AI 플랫폼이 실제 환경에서 센싱 성능이 불완전해지는 상황에서도 제어된 동작을 유지할 수 있다.

##  

## 10.05. Fusion Latency Budget

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

Fusion latency budget defines the maximum allowable time between a physical event being sensed and the corresponding fused state becoming available to downstream functions. In autonomous robots, this delay directly affects localization, perception, planning, control, and safety. A fusion algorithm may produce mathematically accurate estimates, but if those estimates describe an outdated physical state, the overall system can still make incorrect or unsafe decisions.

Latency should therefore be analyzed as an end-to-end property rather than as the execution time of the fusion algorithm alone. The complete path begins when a sensor captures a physical phenomenon and includes sensing, internal processing, communication, middleware transport, synchronization, buffering, fusion computation, and output delivery. Every stage consumes part of the total latency budget and can introduce both fixed delay and timing variation.

Sensor acquisition latency begins at the physical measurement process. A camera requires exposure and frame readout, a rotating LiDAR requires time to complete part or all of a scan, radar performs waveform transmission and signal processing, and an IMU samples inertial quantities at defined intervals. The timestamp should represent the actual measurement time as accurately as possible because data arrival time alone cannot describe when the observed physical event occurred.

Internal sensor processing adds another component to the latency budget. Cameras may perform image signal processing, radar sensors execute detection and signal-processing pipelines, GNSS receivers calculate navigation solutions, and LiDAR units assemble point-cloud packets. These operations can create delays before measurements leave the device. Sensor specifications and system measurements should therefore distinguish acquisition time from internal processing and communication latency.

Communication latency occurs while measurements travel from sensors to computing nodes through Ethernet, CAN, CAN FD, USB, serial links, or specialized camera interfaces. Transmission time depends on payload size, link bandwidth, bus utilization, packet scheduling, protocol overhead, and network topology. High-bandwidth sensors such as cameras and 3D LiDARs require particular attention because congestion can increase both average delay and latency variation.

Middleware introduces latency through message reception, serialization, copying, queues, scheduling, and inter-process or inter-computer transport. In ROS 2 and DDS-based systems, Quality of Service settings influence how messages are buffered and delivered. Excessive queue depth may preserve old data at the cost of freshness, while poorly selected reliability policies can create retransmission or blocking behavior that increases the age of information reaching the fusion process.

Synchronization latency arises because fusion algorithms often require measurements from different sensors to correspond to compatible timestamps. A fast IMU stream may need to wait for slower GNSS, camera, radar, or LiDAR observations, while approximate synchronization mechanisms may hold messages until matching data becomes available. Waiting can improve temporal alignment but increases latency, creating a fundamental tradeoff between synchronization accuracy and information freshness.

Buffering is necessary when sensor streams are asynchronous or arrive with variable jitter. The fusion middleware may retain recent observations so that measurements can be associated, interpolated, or processed after delayed information arrives. However, buffering must be bounded. Large buffers can hide communication irregularities while silently increasing system latency, so buffer depth should be selected from measured sensor rates, jitter distributions, processing times, and application deadlines.

Fusion computation itself consumes part of the available budget. Kalman filtering can often be executed efficiently, but large state vectors, nonlinear EKF operations, particle filters, scan matching, visual localization, and multi-object fusion can require substantially more computation. Processing time should be measured under realistic CPU and GPU loads because an algorithm that satisfies timing requirements in isolation may violate them when perception and AI workloads execute concurrently.

Scheduling latency becomes important when several software components compete for computing resources. A fusion process may be ready to execute but remain delayed because another thread occupies the CPU, GPU, memory subsystem, or accelerator. Operating-system scheduling, thread priorities, CPU affinity, interrupt handling, and resource contention therefore contribute to the practical latency budget. Average execution time alone cannot represent these worst-case scheduling effects.

The age of information is often more meaningful than simple communication delay. A fused state delivered at the current time may contain measurements captured tens or hundreds of milliseconds earlier. The system should therefore track the relationship between measurement timestamp, processing time, and output timestamp. State propagation can compensate for some delay by projecting an estimate toward the current time, but propagation uncertainty increases as the underlying observations become older.

Different downstream functions require different latency limits. Mapping and long-term world-model updates may tolerate relatively large delays, while motion control and collision avoidance generally require much fresher information. Safety-related functions can require particularly predictable timing because delayed obstacle or localization information reduces the time available for intervention. A single universal latency requirement is therefore inappropriate for all fusion outputs.

Vehicle speed converts latency directly into spatial error. If a robot travels at velocity v and its state estimate is delayed by time Δt, the platform moves approximately vΔt before the information is consumed, even before other estimation errors are considered. This relationship becomes increasingly important for outdoor AMRs, autonomous vehicles, and UAVs because higher operating speed reduces the acceptable end-to-end latency for perception and control.

Angular motion creates similar problems for orientation-sensitive sensors. During rapid turning, even a small timestamp difference between camera, LiDAR, radar, and IMU measurements can produce substantial geometric misalignment. Motion compensation and timestamp-aware transformation can reduce these effects, but they depend on accurate synchronization and high-rate state estimates. Latency budgeting must therefore consider both translational and rotational dynamics.

A practical budget allocates portions of the total allowable delay to individual pipeline stages. Sensor acquisition, device processing, network transport, middleware handling, synchronization, fusion computation, and downstream delivery can each receive a target limit. These allocations provide engineering boundaries for component selection and optimization. If one stage exceeds its allocation, engineers can identify where architectural changes are required instead of treating latency as an unexplained system-level problem.

Latency and jitter should be evaluated separately. Latency represents the delay experienced by information, while jitter describes variation in that delay over time. A system with a stable 30 ms delay can sometimes be easier to compensate than one varying unpredictably between 10 and 80 ms. Fusion pipelines therefore need distributions, percentiles, maximum observed values, and deadline-miss statistics rather than only average latency measurements.

Timestamp quality is fundamental to accurate latency measurement. Hardware timestamps generated close to the physical acquisition event provide stronger timing information than software timestamps assigned after data reaches a driver. Precision Time Protocol, GNSS time references, hardware triggers, and synchronized clocks can establish a common time base across distributed sensors and computers. Without reliable clock synchronization, measured latency values themselves can become misleading.

Deadline monitoring allows the system to detect when sensor or fusion information becomes too old for its intended use. Middleware can compare the current time with acquisition timestamps and reject stale measurements before they enter the estimator. Fusion outputs can similarly include their reference time, processing delay, and validity status. Higher-level functions can then decide whether an estimate remains suitable for normal operation or requires degraded behavior.

Fault-tolerant fusion and latency management are closely connected because excessive delay is effectively a sensor-quality fault. A measurement can be numerically correct yet operationally invalid if it arrives too late. Sensor-health logic should therefore consider timeout, message age, synchronization error, and latency deviation together with conventional measurement consistency. Persistent timing degradation can trigger reduced weighting, sensor isolation, or a transition to another fusion mode.

Optimization should begin with measurement rather than assumption. Engineers should instrument each major pipeline boundary and record acquisition timestamps, arrival times, queue entry and exit times, fusion start and completion times, and output delivery times. This produces an observable latency chain that can reveal network congestion, queue buildup, scheduling interference, expensive algorithms, or synchronization waits that would otherwise remain hidden.

Recording latency statistics during realistic operation is particularly important because laboratory conditions may underestimate system load. Multiple cameras, LiDAR processing, neural networks, mapping, visualization, logging, and communication can execute simultaneously on an AMR compute platform. Stress testing should therefore examine normal load, peak load, sensor bursts, degraded networking, and processor contention to determine whether timing requirements remain satisfied under realistic conditions.

Latency reduction can involve faster interfaces, higher network bandwidth, reduced message copies, zero-copy transport, smaller queues, optimized synchronization, parallel computation, hardware acceleration, and better task scheduling. However, reducing delay should not compromise data integrity or synchronization. The objective is not simply the lowest possible latency but predictable delivery of sufficiently synchronized and trustworthy information within the deadline required by each function.

For Physical AI systems, fusion latency budget connects sensing architecture directly to autonomous behavior. The world representation used by planning and control must describe the physical environment closely enough to the present moment that decisions remain valid. As robots become faster and perception pipelines become more complex, timing becomes an architectural dimension equal in importance to sensor accuracy, computational performance, and probabilistic estimation quality.

Fusion latency budget should therefore be treated as a measurable engineering contract across sensors, networks, middleware, computing, and fusion algorithms. End-to-end delay, jitter, timestamp accuracy, information age, and deadline compliance must be designed and validated together. This disciplined approach allows AMRs, autonomous vehicles, UAVs, and other Physical AI platforms to deliver fused state information with the freshness and predictability required for reliable autonomous operation.

융합 지연시간 예산(Fusion Latency Budget)은 물리적 사건이 센서에 의해 감지된 시점부터 해당 사건을 반영한 융합 상태(Fused State)가 하위 기능(Downstream Function)에 제공될 때까지 허용되는 최대 시간을 정의한다. 자율 로봇(Autonomous Robot)에서 이러한 지연은 위치추정(Localization), 인지(Perception), 경로 계획(Planning), 제어(Control), 안전(Safety)에 직접적인 영향을 미친다. 융합 알고리즘이 수학적으로 정확한 추정값을 생성하더라도 해당 추정값이 이미 과거의 물리적 상태를 나타낸다면 전체 시스템은 여전히 잘못되거나 안전하지 않은 의사결정을 내릴 수 있다.

따라서 지연시간(Latency)은 융합 알고리즘 자체의 실행 시간만이 아니라 종단간 특성(End-to-End Property)으로 분석해야 한다. 전체 경로는 센서가 물리적 현상을 포착하는 순간부터 시작되며 센싱(Sensing), 내부 처리(Internal Processing), 통신(Communication), 미들웨어 전송(Middleware Transport), 동기화(Synchronization), 버퍼링(Buffering), 융합 연산(Fusion Computation), 출력 전달(Output Delivery)을 포함한다. 모든 단계는 전체 지연시간 예산의 일부를 소비하며 고정 지연(Fixed Delay)과 시간 변동(Timing Variation)을 모두 발생시킬 수 있다.

센서 획득 지연(Sensor Acquisition Latency)은 물리적 측정 과정에서 시작된다. 카메라는 노출(Exposure)과 프레임 판독(Frame Readout)이 필요하고, 회전형 라이다(Rotating LiDAR)는 스캔의 일부 또는 전체를 완료하는 데 시간이 필요하며, 레이더(Radar)는 파형 송신과 신호 처리를 수행하고, 관성측정장치(IMU)는 정해진 주기로 관성량을 샘플링한다. 타임스탬프(Timestamp)는 실제 측정 시간을 가능한 정확하게 나타내야 하며 데이터 도착 시간만으로는 관측된 물리적 사건이 언제 발생했는지를 정확히 설명할 수 없다.

센서 내부 처리(Internal Sensor Processing)는 지연시간 예산에 또 다른 요소를 추가한다. 카메라는 이미지 신호 처리(Image Signal Processing)를 수행할 수 있고, 레이더 센서는 검출 및 신호 처리 파이프라인(Detection and Signal-Processing Pipeline)을 실행하며, 위성항법시스템(GNSS) 수신기는 내비게이션 해(Navigation Solution)를 계산하고, 라이다는 포인트 클라우드 패킷(Point-Cloud Packet)을 구성한다. 이러한 연산은 측정값이 장치를 벗어나기 전에 지연을 발생시킬 수 있으므로 센서 사양과 시스템 측정에서는 획득 시간과 내부 처리 및 통신 지연을 구분해야 한다.

통신 지연(Communication Latency)은 측정값이 이더넷(Ethernet), CAN, CAN FD, USB, 직렬 링크(Serial Link), 특수 카메라 인터페이스를 통해 센서에서 컴퓨팅 노드(Computing Node)로 이동하는 동안 발생한다. 전송 시간은 페이로드 크기(Payload Size), 링크 대역폭(Link Bandwidth), 버스 사용률(Bus Utilization), 패킷 스케줄링(Packet Scheduling), 프로토콜 오버헤드(Protocol Overhead), 네트워크 토폴로지(Network Topology)에 따라 달라진다. 카메라와 3차원 라이다(3D LiDAR) 같은 고대역폭 센서는 혼잡으로 인해 평균 지연과 지연시간 변동이 모두 증가할 수 있으므로 특히 주의해야 한다.

미들웨어(Middleware)는 메시지 수신, 직렬화(Serialization), 데이터 복사, 큐(Queue), 스케줄링(Scheduling), 프로세스 간 또는 컴퓨터 간 전송 과정에서 지연을 추가한다. ROS 2 및 데이터 분산 서비스 기반 시스템(DDS-Based System)에서는 서비스 품질(Quality of Service, QoS) 설정이 메시지의 버퍼링과 전달 방식에 영향을 준다. 지나치게 깊은 큐는 데이터의 최신성(Freshness)을 희생하면서 오래된 데이터를 유지할 수 있으며, 부적절한 신뢰성 정책(Reliability Policy)은 재전송 또는 블로킹(Blocking)을 발생시켜 융합 프로세스에 도달하는 정보의 시간적 노후도(Age of Information)를 증가시킬 수 있다.

동기화 지연(Synchronization Latency)은 융합 알고리즘이 서로 다른 센서의 측정값을 호환 가능한 타임스탬프에 맞추어야 하기 때문에 발생한다. 빠른 관성측정장치 스트림은 더 느린 위성항법시스템, 카메라, 레이더 또는 라이다 관측값을 기다려야 할 수 있으며, 근사 동기화 메커니즘(Approximate Synchronization Mechanism)은 대응하는 데이터가 도착할 때까지 메시지를 보관할 수 있다. 이러한 대기는 시간 정렬(Temporal Alignment)을 향상시킬 수 있지만 지연시간을 증가시키므로 동기화 정확도와 정보 최신성 사이에 근본적인 절충 관계(Tradeoff)가 존재한다.

센서 스트림이 비동기식(Asynchronous)이거나 가변적인 지터(Jitter)를 가지고 도착하는 경우 버퍼링이 필요하다. 융합 미들웨어는 측정값을 연관시키거나 보간(Interpolation)하고 지연된 정보가 도착한 이후 처리할 수 있도록 최근 관측값을 유지할 수 있다. 그러나 버퍼링에는 명확한 한계가 필요하다. 지나치게 큰 버퍼는 통신 이상을 숨기면서 시스템 지연을 조용히 증가시킬 수 있으므로 버퍼 깊이(Buffer Depth)는 실제 측정된 센서 주기, 지터 분포(Jitter Distribution), 처리 시간, 응용 프로그램의 데드라인(Deadline)을 기준으로 결정해야 한다.

융합 연산(Fusion Computation) 자체도 사용 가능한 지연시간 예산의 일부를 소비한다. 칼만 필터링(Kalman Filtering)은 일반적으로 효율적으로 실행할 수 있지만 큰 상태 벡터(State Vector), 비선형 확장 칼만 필터 연산(Nonlinear EKF Operation), 파티클 필터(Particle Filter), 스캔 매칭(Scan Matching), 비전 위치추정(Visual Localization), 다중 객체 융합(Multi-Object Fusion)은 상당히 많은 연산량을 요구할 수 있다. 단독 실행 시 시간 요구사항을 만족하는 알고리즘도 인지 및 인공지능 워크로드가 동시에 실행되면 요구사항을 위반할 수 있으므로 실제 CPU 및 GPU 부하 조건에서 처리 시간을 측정해야 한다.

여러 소프트웨어 구성요소가 컴퓨팅 자원을 경쟁적으로 사용하는 경우 스케줄링 지연(Scheduling Latency)이 중요해진다. 융합 프로세스가 실행 준비 상태여도 다른 스레드가 CPU, GPU, 메모리 서브시스템(Memory Subsystem), 가속기(Accelerator)를 점유하고 있으면 실행이 지연될 수 있다. 따라서 운영체제 스케줄링, 스레드 우선순위(Thread Priority), CPU 친화도(CPU Affinity), 인터럽트 처리(Interrupt Handling), 자원 경합(Resource Contention)이 실제 지연시간 예산에 영향을 준다. 평균 실행 시간만으로는 이러한 최악 조건의 스케줄링 영향을 표현할 수 없다.

정보의 시간적 노후도(Age of Information)는 단순한 통신 지연보다 더 의미 있는 지표가 될 수 있다. 현재 시점에 전달된 융합 상태라도 그 내부에는 수십 또는 수백 밀리초 전에 측정된 데이터가 포함될 수 있다. 따라서 시스템은 측정 타임스탬프, 처리 시간, 출력 타임스탬프 사이의 관계를 추적해야 한다. 상태 전파(State Propagation)를 이용하면 추정값을 현재 시점으로 예측하여 일부 지연을 보상할 수 있지만 기반 관측값이 오래될수록 전파 불확실성(Propagation Uncertainty)은 증가한다.

하위 기능마다 요구되는 지연시간 한계는 서로 다르다. 매핑(Mapping)과 장기적인 월드 모델 갱신(World-Model Update)은 비교적 큰 지연을 허용할 수 있지만 모션 제어(Motion Control)와 충돌 회피(Collision Avoidance)는 일반적으로 훨씬 최신의 정보를 필요로 한다. 특히 안전 관련 기능은 장애물 또는 위치 정보가 지연될수록 개입 가능한 시간이 감소하므로 예측 가능한 타이밍을 요구할 수 있다. 따라서 모든 융합 출력에 하나의 보편적인 지연시간 요구사항을 적용하는 것은 적절하지 않다.

차량 속도(Vehicle Speed)는 지연시간을 직접적인 공간 오차(Spatial Error)로 변환한다. 로봇이 속도 v로 이동하고 상태 추정값이 Δt만큼 지연된다면 다른 추정 오차를 고려하지 않더라도 플랫폼은 정보가 사용되기 전에 대략 vΔt만큼 이동한다. 이러한 관계는 운용 속도가 증가할수록 인지 및 제어를 위한 허용 가능한 종단간 지연시간이 감소하기 때문에 실외 자율이동로봇(Outdoor AMR), 자율주행차(Autonomous Vehicle), 무인항공기(UAV)에서 더욱 중요해진다.

회전 운동(Angular Motion)은 자세에 민감한 센서에서 유사한 문제를 발생시킨다. 빠르게 회전하는 동안 카메라, 라이다, 레이더, 관성측정장치 측정값 사이에 작은 타임스탬프 차이만 있어도 상당한 기하학적 정렬 오류(Geometric Misalignment)가 발생할 수 있다. 모션 보상(Motion Compensation)과 타임스탬프 인지형 변환(Timestamp-Aware Transformation)을 통해 이러한 영향을 줄일 수 있지만 정확한 동기화와 고주파 상태 추정값이 필요하다. 따라서 지연시간 예산은 병진 운동(Translational Dynamics)과 회전 운동(Rotational Dynamics)을 모두 고려해야 한다.

실용적인 지연시간 예산은 전체 허용 지연을 개별 파이프라인 단계에 할당한다. 센서 획득, 장치 처리, 네트워크 전송, 미들웨어 처리, 동기화, 융합 연산, 하위 시스템 전달에 각각 목표 한계(Target Limit)를 설정할 수 있다. 이러한 할당은 구성요소 선택과 최적화를 위한 엔지니어링 경계(Engineering Boundary)를 제공한다. 특정 단계가 할당량을 초과하면 지연을 설명되지 않는 시스템 수준 문제로 취급하는 대신 어느 부분에서 구조적 변경이 필요한지를 식별할 수 있다.

지연시간과 지터는 별도로 평가해야 한다. 지연시간은 정보가 경험하는 시간적 지연을 의미하며 지터는 해당 지연이 시간에 따라 변화하는 정도를 나타낸다. 안정적인 30밀리초(ms)의 지연을 갖는 시스템은 10\~80밀리초 사이에서 예측할 수 없이 변동하는 시스템보다 보상하기 쉬울 수 있다. 따라서 융합 파이프라인은 평균 지연시간만이 아니라 분포(Distribution), 백분위수(Percentile), 최대 관측값(Maximum Observed Value), 데드라인 미준수 통계(Deadline-Miss Statistics)를 평가해야 한다.

정확한 지연시간 측정을 위해서는 타임스탬프 품질(Timestamp Quality)이 필수적이다. 물리적 획득 시점에 가까운 위치에서 생성되는 하드웨어 타임스탬프(Hardware Timestamp)는 데이터가 드라이버에 도달한 후 할당되는 소프트웨어 타임스탬프(Software Timestamp)보다 더 정확한 시간 정보를 제공한다. 정밀 시간 프로토콜(Precision Time Protocol, PTP), 위성항법시스템 시간 기준(GNSS Time Reference), 하드웨어 트리거(Hardware Trigger), 동기화된 클록(Synchronized Clock)을 통해 분산된 센서와 컴퓨터 사이에 공통 시간 기준을 구축할 수 있다. 신뢰할 수 있는 클록 동기화가 없다면 측정된 지연시간 값 자체가 잘못될 수 있다.

데드라인 모니터링(Deadline Monitoring)을 통해 센서 또는 융합 정보가 의도된 용도로 사용하기에 지나치게 오래되었는지를 검출할 수 있다. 미들웨어는 현재 시간과 획득 타임스탬프를 비교하여 오래된 측정값(Stale Measurement)이 추정기에 입력되기 전에 거부할 수 있다. 융합 출력도 기준 시간(Reference Time), 처리 지연, 유효성 상태(Validity Status)를 포함할 수 있다. 상위 기능은 이를 이용하여 추정값이 정상 운용에 계속 적합한지 또는 성능 저하 동작(Degraded Behavior)이 필요한지를 판단할 수 있다.

고장 허용 융합(Fault-Tolerant Fusion)과 지연시간 관리는 밀접하게 연결되어 있으며 과도한 지연은 실질적으로 센서 품질 고장(Sensor-Quality Fault)으로 간주할 수 있다. 측정값이 수치적으로 정확하더라도 너무 늦게 도착한다면 운용 측면에서는 유효하지 않을 수 있다. 따라서 센서 상태 로직(Sensor-Health Logic)은 일반적인 측정 일관성과 함께 타임아웃, 메시지 경과 시간(Message Age), 동기화 오류, 지연시간 편차를 고려해야 한다. 지속적인 타이밍 성능 저하는 가중치 감소, 센서 격리(Sensor Isolation), 다른 융합 모드(Fusion Mode)로의 전환을 유발할 수 있다.

최적화(Optimization)는 가정이 아니라 측정에서 시작해야 한다. 엔지니어는 주요 파이프라인 경계마다 계측(Instrumentation)을 적용하고 획득 타임스탬프, 도착 시간, 큐 진입 및 이탈 시간, 융합 시작 및 완료 시간, 출력 전달 시간을 기록해야 한다. 이를 통해 관찰 가능한 지연시간 체인(Observable Latency Chain)을 구축할 수 있으며 네트워크 혼잡, 큐 누적(Queue Buildup), 스케줄링 간섭, 높은 연산 비용의 알고리즘, 동기화 대기 등 기존에는 숨겨져 있던 문제를 식별할 수 있다.

실제 운용 중 지연시간 통계를 기록하는 것은 실험실 조건이 시스템 부하를 과소평가할 수 있기 때문에 특히 중요하다. 여러 카메라, 라이다 처리, 신경망(Neural Network), 매핑, 시각화, 로깅(Logging), 통신 기능이 자율이동로봇 컴퓨팅 플랫폼에서 동시에 실행될 수 있다. 따라서 스트레스 시험(Stress Testing)은 정상 부하, 최대 부하(Peak Load), 센서 버스트(Sensor Burst), 네트워크 성능 저하, 프로세서 자원 경합 조건을 포함하여 실제 환경에서도 타이밍 요구사항이 유지되는지를 평가해야 한다.

지연시간 감소에는 더 빠른 인터페이스, 높은 네트워크 대역폭, 메시지 복사 감소, 무복사 전송(Zero-Copy Transport), 더 작은 큐, 최적화된 동기화, 병렬 연산(Parallel Computation), 하드웨어 가속(Hardware Acceleration), 개선된 작업 스케줄링(Task Scheduling) 등이 활용될 수 있다. 그러나 지연시간 감소를 위해 데이터 무결성(Data Integrity)이나 동기화를 희생해서는 안 된다. 목표는 단순히 가장 낮은 지연시간을 달성하는 것이 아니라 각 기능에서 요구하는 데드라인 내에 충분히 동기화되고 신뢰할 수 있는 정보를 예측 가능하게 전달하는 것이다.

피지컬 AI 시스템(Physical AI System)에서 융합 지연시간 예산은 센싱 구조(Sensing Architecture)를 자율 동작(Autonomous Behavior)과 직접 연결한다. 경로 계획과 제어가 사용하는 월드 표현(World Representation)은 의사결정이 유효할 정도로 현재의 물리적 환경을 충분히 가깝게 반영해야 한다. 로봇의 속도가 증가하고 인지 파이프라인이 복잡해질수록 타이밍(Timing)은 센서 정확도, 컴퓨팅 성능, 확률적 추정 품질(Probabilistic Estimation Quality)과 동등하게 중요한 구조적 설계 요소가 된다.

따라서 융합 지연시간 예산(Fusion Latency Budget)은 센서, 네트워크, 미들웨어, 컴퓨팅, 융합 알고리즘 전체에 적용되는 측정 가능한 엔지니어링 계약(Measurable Engineering Contract)으로 다루어야 한다. 종단간 지연(End-to-End Delay), 지터, 타임스탬프 정확도, 정보의 시간적 노후도, 데드라인 준수(Deadline Compliance)를 함께 설계하고 검증해야 한다. 이러한 체계적인 접근을 통해 자율이동로봇(AMR), 자율주행차, 무인항공기, 기타 피지컬 AI 플랫폼은 신뢰성 높은 자율 운용에 필요한 최신성(Freshness)과 예측 가능성(Predictability)을 갖춘 융합 상태 정보를 제공할 수 있다.
