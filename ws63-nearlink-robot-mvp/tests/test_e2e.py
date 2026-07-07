import threading
import time
import unittest

from simulator.robot_simulator import RobotController, RobotTCPServer
from upper_client.robot_client import RobotClient
from upper_client.robot_protocol import Command, Status


class EndToEndTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = RobotController(watchdog_seconds=0.12)
        self.server = RobotTCPServer(
            ("127.0.0.1", 0), controller=self.controller, quiet=True
        )
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.thread.start()
        self.client = RobotClient("127.0.0.1", self.server.server_address[1])

    def tearDown(self) -> None:
        self.client.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)

    def test_five_commands_and_ack(self) -> None:
        for key in ("F", "L", "R", "B", "S"):
            self.assertEqual(self.client.send(key).status, Status.OK)
        state = self.controller.snapshot()
        self.assertEqual((state.left, state.right), (0, 0))
        self.assertEqual(state.last_command, Command.STOP)

    def test_watchdog_stops_motors(self) -> None:
        self.client.send("F", 50)
        self.assertEqual(
            (self.controller.snapshot().left, self.controller.snapshot().right),
            (50, 50),
        )
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            state = self.controller.snapshot()
            if state.watchdog_stops:
                break
            time.sleep(0.02)
        self.assertEqual((state.left, state.right), (0, 0))
        self.assertEqual(state.watchdog_stops, 1)

    def test_obstacle_blocks_forward(self) -> None:
        self.server.obstacle = True
        result = self.client.send("F")
        self.assertEqual(result.status, Status.OBSTACLE_STOP)
        self.assertEqual(
            (self.controller.snapshot().left, self.controller.snapshot().right),
            (0, 0),
        )


if __name__ == "__main__":
    unittest.main()

