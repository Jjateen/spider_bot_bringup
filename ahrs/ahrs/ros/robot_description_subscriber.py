import threading
import xml.etree.ElementTree as ET

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from ahrs.graphics.robot_model import RobotModel
from ahrs.graphics.urdf_parser import load_urdf_from_string
from ahrs.utils.logger import setup_logger

logger = setup_logger("ahrs.robot_description")


class RobotDescriptionSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("robot_description_subscriber")
        self._model: RobotModel | None = None
        self._event = threading.Event()
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._subscription = self.create_subscription(
            String, "/robot_description", self._callback, latched_qos
        )
        logger.info("Subscribed to /robot_description")

    def _callback(self, msg: String) -> None:
        try:
            self._model = load_urdf_from_string(msg.data)
            logger.info(
                f"Loaded robot model from /robot_description "
                f"({len(self._model.links)} links, {len(self._model.joints)} joints)"
            )
        except ET.ParseError as e:
            logger.error(f"Failed to parse URDF from /robot_description: {e}")
        except Exception as e:
            logger.error(f"Failed to load robot model: {e}")
        finally:
            self._event.set()

    @property
    def model(self) -> RobotModel | None:
        return self._model

    def wait_for_description(self, timeout: float = 5.0) -> bool:
        return self._event.wait(timeout)
