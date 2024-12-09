"""Robot builder module for MuJoCo models.

This module provides functionality for building and configuring robot models in MuJoCo.
It handles model loading, PD control setup, and various robot properties.
"""
import numpy as np
import mujoco
from brax.io import mjcf
from typing import List, Optional, Tuple, Dict, Any, Union
import os
from dataclasses import dataclass, field

from . import _utils


@dataclass
class ActuatorMotor:
    """Base class for robot actuators.
    
    Attributes:
        range: Valid range for actuator commands [min, max]
        dyn: Dynamic parameters for the actuator
        gain: Gain parameters for the actuator
        bias: Bias parameters for the actuator
    """
    range: List[float] = field(default_factory=lambda: [-100, 100])
    dyn: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0]))
    gain: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0]))
    bias: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0]))


@dataclass
class ActuatorPosition(ActuatorMotor):
    """Position-controlled actuator implementation.
    
    Attributes:
        kp: Position gain
        kd: Derivative gain
    """
    kp: float = 1.0
    kd: float = 0.0
    
    def __post_init__(self):
        self.gain[0] = self.kp
        self.bias[1] = -self.kp
        self.bias[2] = -self.kd


@dataclass
class ActuatorVelocity(ActuatorMotor):
    """Velocity-controlled actuator implementation.
    
    Attributes:
        kv: Velocity gain
    """
    kv: float = 1.0
    
    def __post_init__(self):
        self.gain[0] = self.kv
        self.bias[2] = -self.kv


@dataclass
class ActuatedJoint:
    """Data class containing information about a single actuated joint.
    
    Attributes:
        name: Joint name in the MuJoCo model
        joint_id: Unique joint ID in MuJoCo model
        qpos_idx: Index in the position state vector
        qvel_idx: Index in the velocity state vector
        lower_limit: Lower position limit of the joint
        upper_limit: Upper position limit of the joint
    """
    name: str
    joint_id: int  
    qpos_idx: int
    qvel_idx: int
    lower_limit: float
    upper_limit: float


@dataclass
class ActuatedJoints:
    """Data class containing information about all actuated joints."""
    joints: List[ActuatedJoint]
    qvel_indices: np.ndarray
    qpos_indices: np.ndarray
    joint_ids: np.ndarray
    names: List[str]
    lower_limits: np.ndarray
    upper_limits: np.ndarray


class RobotBuilder:
    """Class to handle robot initialization and configuration."""
    
    def __init__(
        self,
        model_path: str,
        dt: float,
    ) -> None:
        """Initialize the robot from a MuJoCo model.
        
        Args:
            model_path: Path to the MuJoCo XML model file
            dt: Simulation timestep in seconds
            
        Raises:
            ValueError: If dt is not positive or if model_path is empty
            FileNotFoundError: If model_path does not exist
        """
        if not model_path:
            raise ValueError("Model path cannot be empty")
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        if dt <= 0:
            raise ValueError("Timestep must be positive")
            
        self._dt = None
        
        # Initialize gain arrays as None
        self._kp = None
        self._kd = None
        
        # Initialize joint limits as None
        self._joint_limits = None
        
        # Load robot spec
        self.spec = mujoco.MjSpec.from_file(model_path)
            
        # Compile model
        self.mj_model = self.spec.compile()
        
        # Set timestep
        self.dt = dt
        
        # Create data after model compilation
        self.data = mujoco.MjData(self.mj_model)

    @property
    def dt(self) -> float:
        """Get simulation timestep in seconds."""
        if self._dt is None:
            raise ValueError("Simulation timestep (dt) has not been set")
        return self._dt

    @dt.setter
    def dt(self, value: float) -> None:
        """Set simulation timestep and update the model.
        
        Args:
            value: Timestep value in seconds
            
        Raises:
            ValueError: If value is None or not positive
        """
        if value is None:
            raise ValueError("Cannot set dt to None")
        if value <= 0:
            raise ValueError("Timestep must be positive")
        self._dt = value
        if hasattr(self, 'mj_model'):
            self.mj_model.opt.timestep = value

    @property
    def kp(self) -> np.ndarray:
        """Get proportional gains for PD controller."""
        if self._kp is None:
            raise ValueError("Proportional gains (kp) have not been set")
        return self._kp

    @kp.setter
    def kp(self, value: np.ndarray) -> None:
        """Set proportional gains and update the model.
        
        Args:
            value: Array of proportional gain values
            
        Raises:
            ValueError: If value is None
        """
        if value is None:
            raise ValueError("Cannot set kp to None")
        self._kp = value
        if hasattr(self, 'mj_model'):
            self._setup_pd(self._kp, self._kd)

    @property
    def kd(self) -> np.ndarray:
        """Get derivative gains for PD controller."""
        if self._kd is None:
            raise ValueError("Derivative gains (kd) have not been set")
        return self._kd

    @kd.setter
    def kd(self, value: np.ndarray) -> None:
        """Set derivative gains and update the model.
        
        Args:
            value: Array of derivative gain values
            
        Raises:
            ValueError: If value is None
        """
        if value is None:
            raise ValueError("Cannot set kd to None")
        self._kd = value
        if hasattr(self, 'mj_model'):
            self._setup_pd(self._kp, self._kd)

    @property
    def joint_limits(self) -> np.ndarray:
        """Get joint position limits as (n_joints, 2) array of [lower, upper] bounds."""
        if self._joint_limits is None:
            self._joint_limits = self.mj_model.jnt_range
        return self._joint_limits



    def _setup_pd(self, kp: np.ndarray, kd: np.ndarray) -> None:
        """Configure PD controller gains for MuJoCo actuators.
        
        Args:
            kp: Array of proportional gains
            kd: Array of derivative gains
        """
        if kp is None or kd is None:
            return
            
        # Set actuator gains (kp) and biases (-kp, -kd)
        self.mj_model.actuator_gainprm[:, 0] = kp
        self.mj_model.actuator_biasprm[:, 1] = -kp
        self.mj_model.actuator_biasprm[:, 2] = -kd
        
        # Set actuator types to position servo
        self.mj_model.actuator_gaintype[:] = 0  # Position servo
        self.mj_model.actuator_biastype[:] = 1  # Affine transformation

    def set_solver_settings(
        self,
        solver_type: int = 2,  # Newton solver
        iterations: int = 1,
        ls_iterations: int = 5,
        use_pyramidal_cone: bool = True,
        impratio: float = 100.0,
        disable_euler_damp: bool = True
    ) -> None:
        """Configure solver parameters for the model.
        
        Args:
            solver_type: Solver algorithm (0=PGS, 1=CG, 2=Newton)
            iterations: Number of main solver iterations
            ls_iterations: Number of line search iterations
            use_pyramidal_cone: Whether to use pyramidal cone approximation
            impratio: Impermeability ratio
            disable_euler_damp: Whether to disable euler damping
            
        Raises:
            ValueError: If parameters are invalid
        """
        if solver_type not in [0, 1, 2]:
            raise ValueError("solver_type must be 0 (PGS), 1 (CG), or 2 (Newton)")
        if iterations < 1:
            raise ValueError("iterations must be positive")
        if ls_iterations < 1:
            raise ValueError("ls_iterations must be positive")
            
        self.mj_model.opt.solver = solver_type
        self.mj_model.opt.iterations = iterations
        self.mj_model.opt.ls_iterations = ls_iterations
        
        # Set cone type if requested
        if use_pyramidal_cone:
            self.mj_model.opt.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
            self.mj_model.opt.impratio = impratio
        
        # Disable euler damping if requested
        if disable_euler_damp:
            self.mj_model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_EULERDAMP

    def _get_body_id(self, name: str) -> int:
        """Get body ID by name.
        
        Args:
            name: Name of the body
            
        Returns:
            Body ID in the model
            
        Raises:
            ValueError: If body is not found
        """
        body_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id == -1:
            raise ValueError(f"Body not found: {name}")
        return body_id

    def _get_sensor_id(self, name: str) -> int:
        """Get sensor ID by name.
        
        Args:
            name: Name of the sensor
            
        Returns:
            Sensor ID in the model
            
        Raises:
            ValueError: If sensor is not found
        """
        sensor_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id == -1:
            raise ValueError(f"Sensor not found: {name}")
        return sensor_id

    def _get_joint_id(self, name: str) -> int:
        """Get joint ID by name.
        
        Args:
            name: Name of the joint
            
        Returns:
            Joint ID in the model
            
        Raises:
            ValueError: If joint is not found
        """
        joint_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id == -1:
            raise ValueError(f"Joint not found: {name}")
        return joint_id

    def _get_actuator_id(self, name: str) -> int:
        """Get actuator ID by name.
        
        Args:
            name: Name of the actuator
            
        Returns:
            Actuator ID in the model
            
        Raises:
            ValueError: If actuator is not found
        """
        actuator_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id == -1:
            raise ValueError(f"Actuator not found: {name}")
        return actuator_id

    def remove_collisions(self, body_names: Optional[List[str]] = None) -> None:
        """Remove contacts for specified bodies or all bodies if none specified.
        
        Args:
            body_names: List of body names to disable collisions for.
                       If None, disables all collisions.
        """
        if body_names is None:
            self.mj_model.geom_contype[:] = 0
            self.mj_model.geom_conaffinity[:] = 0
            return
            
        body_ids = [self._get_body_id(name) for name in body_names]
        
        for i in range(self.mj_model.ngeom):
            geom_bodyid = self.mj_model.geom_bodyid[i]
            if geom_bodyid in body_ids:
                self.mj_model.geom_contype[i] = 0
                self.mj_model.geom_conaffinity[i] = 0

    def add_collisions(self, body_names: List[str]) -> None:
        """Enable contacts for specified bodies.
        
        Args:
            body_names: List of body names to enable collisions for
            
        Raises:
            ValueError: If body_names is empty or None
        """
        if not body_names:
            raise ValueError("body_names cannot be empty or None")
            
        body_ids = [self._get_body_id(name) for name in body_names]
        
        for i in range(self.mj_model.ngeom):
            geom_bodyid = self.mj_model.geom_bodyid[i]
            if geom_bodyid in body_ids:
                self.mj_model.geom_contype[i] = 1
                self.mj_model.geom_conaffinity[i] = 1

    @property
    def brax_sys(self):
        """Get the Brax system representation of the model."""
        return mjcf.load_model(self.mj_model)

    @property
    def bodies(self) -> List[str]:
        """Get list of all body names in the model."""
        bodies = []
        for i in range(self.mj_model.nbody):
            name = self.mj_model.body(i).name
            if name:  # Only include bodies that have names
                bodies.append(name)
        return bodies

    @property
    def actuated_joints(self) -> ActuatedJoints:
        """Get information about all actuated joints including position limits."""
        joints = []
        qvel_indices = []
        qpos_indices = []
        names = []
        lower_limits = []
        upper_limits = []
        
        # Get unique joint IDs that are actuated
        joint_ids = np.unique(self.mj_model.actuator_trnid[:, 0])
        
        for joint_id in joint_ids:
            # Get joint name
            joint_name = self.mj_model.joint(joint_id).name
            
            # Find qpos and qvel indices for this joint
            qpos_idx = self.mj_model.jnt_qposadr[joint_id]
            qvel_idx = self.mj_model.jnt_dofadr[joint_id]
            
            # Get joint limits
            lower = self.mj_model.jnt_range[joint_id][0]
            upper = self.mj_model.jnt_range[joint_id][1]
            
            joints.append(ActuatedJoint(
                name=joint_name,
                joint_id=joint_id,
                qpos_idx=qpos_idx,
                qvel_idx=qvel_idx,
                lower_limit=lower,
                upper_limit=upper
            ))

            qvel_indices.append(qvel_idx)
            qpos_indices.append(qpos_idx)
            names.append(joint_name)
            lower_limits.append(lower)
            upper_limits.append(upper)

        return ActuatedJoints(
            joints=joints,
            qvel_indices=np.array(qvel_indices),
            qpos_indices=np.array(qpos_indices),
            joint_ids=joint_ids,
            names=names,
            lower_limits=np.array(lower_limits),
            upper_limits=np.array(upper_limits)
        )

    def set_pyramidal_cone_options(self) -> None:
        """Set pyramidal cone options for contact handling.
        
        Sets the following options:
        - Pyramidal cone approximation
        - Impratio = 100
        - 1 iteration
        - 5 line search iterations
        - Disable euler damping
        """
        # Set cone type and solver parameters
        self.mj_model.opt.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL  # pyramidal cone
        self.mj_model.opt.impratio = 100.0  # impermeability ratio
        self.mj_model.opt.iterations = 1  # solver iterations
        self.mj_model.opt.ls_iterations = 5  # line search iterations
        
        # Disable euler damping
        self.mj_model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_EULERDAMP


    def add_sensor(
        self,
        name: str,
        sensor_type: mujoco.mjtSensor,
        body_name: str,
        site_name: Optional[str] = None,
        site_pos: Optional[List[float]] = None,
        site_quat: Optional[List[float]] = None,
        **kwargs: Any
    ) -> None:
        """Add a sensor to the model.
        
        Args:
            name: Name for the sensor
            sensor_type: MuJoCo sensor type (mjtSensor)
            body_name: Name of body to attach sensor to
            site_name: Optional name for sensor site. If None, will use name + "_site"
            site_pos: Optional position offset for sensor site [x, y, z]
            site_quat: Optional orientation for sensor site [w, x, y, z]
            **kwargs: Additional sensor-specific parameters
            
        Raises:
            ValueError: If body doesn't exist or invalid parameters
        """
        # Verify body exists
        try:
            body = self.spec.find_body(body_name)
        except ValueError as e:
            raise ValueError(f"Cannot add sensor to non-existent body: {body_name}") from e

        # Set default site position and orientation if not provided
        if site_pos is None:
            site_pos = [0, 0, 0]
        if site_quat is None:
            site_quat = [1.0, 0.0, 0.0, 0.0]

        if len(site_pos) != 3:
            raise ValueError("site_pos must be [x, y, z]")
        if len(site_quat) != 4:
            raise ValueError("site_quat must be [w, x, y, z]")

        # Create or get site name
        if site_name is None:
            site_name = f"{name}_site"

        # Add site to body if it doesn't exist yet
        site_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id == -1:  # Site doesn't exist
            site = body.add_site(pos=site_pos, quat=site_quat, name=site_name)
        self.mj_model = self.spec.compile()

        # Add sensor following MuJoCo spec syntax
        sensor = self.spec.add_sensor()
        sensor.name = name
        sensor.type = sensor_type
        sensor.objname = site_name
        sensor.objtype = mujoco.mjtObj.mjOBJ_SITE

        # Add any additional parameters from kwargs
        if 'range' in kwargs:
            sensor.range = kwargs['range']

        # Compile after adding sensor
        self.mj_model = self.spec.compile()
        self.data = mujoco.MjData(self.mj_model)

    def add_imu(
        self,
        body_name: str,
        name_prefix: str = "imu",
        site_pos: Optional[List[float]] = None,
        site_quat: Optional[List[float]] = None
    ) -> None:
        """Add IMU (accelerometer + gyro + quaternion) to a body.
        
        Args:
            body_name: Name of body to attach IMU to
            name_prefix: Prefix for sensor names
            site_pos: Optional position offset for IMU site
            site_quat: Optional orientation for IMU site
        """
        # Create a common site for all IMU sensors
        site_name = f"{name_prefix}_site"
        
        # Add accelerometer
        self.add_sensor(
            name=f"{name_prefix}_accel",
            sensor_type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
            body_name=body_name,
            site_name=site_name,
            site_pos=site_pos,
            site_quat=site_quat
        )
        
        # Add gyro
        self.add_sensor(
            name=f"{name_prefix}_gyro",
            sensor_type=mujoco.mjtSensor.mjSENS_GYRO,
            body_name=body_name,
            site_name=site_name,
            site_pos=site_pos,
            site_quat=site_quat
        )

        # Add quaternion
        self.add_sensor(
            name=f"{name_prefix}_quat",
            sensor_type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
            body_name=body_name,
            site_name=site_name,
            site_pos=site_pos,
            site_quat=site_quat
        )

    @property
    def sensor_names(self) -> List[str]:
        """Get list of all sensor names in the model."""
        return [self.data.sensor(i).name for i in range(self.mj_model.nsensor)]
    
    def get_sensor_indices(self, sensor_name: str) -> Tuple[int, int]:
        """Get sensor start and end indices by name.
        
        Args:
            sensor_name: Name of the sensor
            
        Returns:
            Tuple of (start_index, end_index) where end_index is exclusive
            
        Raises:
            ValueError: If sensor is not found
        """
        # Get sensor ID
        sensor_id = self._get_sensor_id(sensor_name)
        
        # Get sensor dimension
        sensor_dim = len(self.data.sensor(sensor_id).data)
        
        # Calculate start index
        start_idx = 0
        for i in range(sensor_id):
            start_idx += len(self.data.sensor(i).data)
            
        # Calculate end index
        end_idx = start_idx + sensor_dim
            
        return start_idx, end_idx

    def set_actuator_type(self, actuator_type: Union[ActuatorMotor, ActuatorPosition, ActuatorVelocity]) -> None:
        """Configure actuator type and gains for the model using actuator dataclass.
        
        Args:
            actuator_type: Actuator configuration object
            
        Raises:
            ValueError: If invalid actuator type
        """
        if isinstance(actuator_type, ActuatorPosition):
            # Set gains for position control
            self.mj_model.actuator_gainprm[:, 0] = actuator_type.kp
            self.mj_model.actuator_biasprm[:, 1] = -actuator_type.kp
            self.mj_model.actuator_biasprm[:, 2] = -actuator_type.kd
            self.mj_model.actuator_gaintype[:] = 0  # Position servo
            self.mj_model.actuator_biastype[:] = 1  # Affine transformation
            
        elif isinstance(actuator_type, ActuatorVelocity):
            # Set gains for velocity control
            self.mj_model.actuator_gainprm[:, 0] = actuator_type.kv
            self.mj_model.actuator_biasprm[:, 2] = -actuator_type.kv
            self.mj_model.actuator_gaintype[:] = 2  # Velocity servo
            self.mj_model.actuator_biastype[:] = 1  # Affine transformation
            
        elif isinstance(actuator_type, ActuatorMotor):
            # Set for torque control
            self.mj_model.actuator_gainprm[:] = actuator_type.gain
            self.mj_model.actuator_biasprm[:] = actuator_type.bias
            self.mj_model.actuator_gaintype[:] = 3  # User-defined gain
            self.mj_model.actuator_biastype[:] = 0  # No bias
            
        else:
            raise ValueError("actuator_type must be an instance of ActuatorMotor, ActuatorPosition, or ActuatorVelocity")
        
        # Set control ranges
        for i in range(self.mj_model.nu):
            self.mj_model.actuator_ctrlrange[i] = actuator_type.range

    def _init_default_actuators(self) -> None:
        """Initialize default actuator configuration (torque control)."""
        # Default torque ranges for UR5e joints (in Nm)
        default_ranges = {
            'shoulder_pan': [-150, 150],
            'shoulder_lift': [-150, 150],
            'elbow': [-150, 150],
            'wrist_1': [-28, 28],
            'wrist_2': [-28, 28],
            'wrist_3': [-28, 28]
        }
        
        self.actuator_configs: Dict[str, ActuatorMotor] = {}
        for name in self.joint_names:
            self.actuator_configs[name] = ActuatorMotor(range=default_ranges[name])
        self._update_actuators()

    def _update_actuators(self) -> None:
        """Update all actuators in the model based on current configuration."""
        for name, actuator in self.actuator_configs.items():
            self.update_actuator(name, actuator)

    def update_actuator(self, actuator_id: Union[str, int], actuator: ActuatorMotor) -> None:
        """Update specific actuator in the model.
        
        Args:
            actuator_id: Actuator name or ID
            actuator: Actuator configuration object
        """
        model_actuator = self.mj_model.actuator(actuator_id)
        model_actuator.dynprm = np.zeros(len(model_actuator.dynprm))
        model_actuator.gainprm = np.zeros(len(model_actuator.gainprm))
        model_actuator.biasprm = np.zeros(len(model_actuator.biasprm))
        model_actuator.ctrlrange = actuator.range 
        model_actuator.dynprm[:3] = actuator.dyn
        model_actuator.gainprm[:3] = actuator.gain
        model_actuator.biasprm[:3] = actuator.bias

    def configure_actuators(self, config: Dict[str, ActuatorMotor]) -> None:
        """Configure multiple actuators at once.
        
        Args:
            config: Dictionary mapping actuator names to their configurations
        
        Raises:
            ValueError: If an unknown actuator name is provided
        """
        for name, actuator in config.items():
            if name in self.actuator_configs:
                self.actuator_configs[name] = actuator
            else:
                raise ValueError(f"Unknown actuator name: {name}")
        self._update_actuators()
