import asyncio
import threading
import sys
import os

# Aggiunge la cartella superiore al path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from textual.app import App, ComposeResult
from textual.widgets import Tabs, Tab, Static, Button, Input, Checkbox

import rclpy
from terminal_node import TerminalNode

class TerminalGUI(App):
    CSS_PATH = "style.css"

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = TerminalNode()
        from rclpy.executors import MultiThreadedExecutor
        self.executor = MultiThreadedExecutor()
        threading.Thread(target=self.executor.spin, daemon=True).start()

        # Callback feedback live
        self.node.arm_feedback_gui = self.update_arm_feedback
        self.node.disarm_feedback_gui = self.update_disarm_feedback
        self.node.takeoff_feedback_gui = self.update_takeoff_feedback
        self.node.landing_feedback_gui = self.update_landing_feedback
        self.node.navigate_feedback_gui = self.update_navigate_feedback

    def compose(self) -> ComposeResult:
        # Tabs
        yield Tabs(
            Tab("Services"),
            Tab("Arm/Disarm"),
            Tab("Flight"),
        )

        # Containers (usiamo id per gestire visibilità)
        yield Static("Services Tab", id="services_container")
        yield Static("Arm/Disarm Tab", id="arm_container")
        yield Static("Flight Tab", id="flight_container")

        # --- Services ---
        yield Button("Enable Service", id="enable_service_button")
        yield Button("Reset Service", id="reset_service_button")
        yield Static("Feedback: ", id="services_feedback")

        # --- Arm/Disarm ---
        yield Input(placeholder="Nome Robot", id="arm_name_input")
        yield Button("Arm", id="arm_button")
        yield Button("Disarm", id="disarm_button")
        yield Static("Feedback: ", id="arm_feedback")

        # --- Flight ---
        yield Input(placeholder="Altitudine Takeoff", id="takeoff_input")
        yield Button("Takeoff", id="takeoff_button")
        yield Button("Landing", id="landing_button")

        yield Input(placeholder="X Navigate", id="navigate_x")
        yield Input(placeholder="Y Navigate", id="navigate_y")
        yield Input(placeholder="Z Navigate", id="navigate_z")
        yield Button("Navigate", id="navigate_button")

        # Throttle e Flight Mode come Input
        yield Input(value=str(self.node.slider_defaults.get('throttle', 0.5)),
                    placeholder="Throttle (0-1)", id="throttle_input")
        yield Checkbox(label="Safety Check", value=self.node.checkbox_defaults.get('safety_check', True),
                       id="safety_checkbox")
        yield Input(value=self.node.select_defaults.get('mode', 'manual'),
                    placeholder="Flight Mode", id="flight_mode_input")
        yield Static("Feedback: ", id="flight_feedback")

    def on_mount(self) -> None:
        # Mostra solo il tab iniziale dopo che i widget sono montati
        self.set_tab_visibility("Services")

    def set_tab_visibility(self, tab_name: str):
        self.query_one("#services_container").display = (tab_name == "Services")
        self.query_one("#arm_container").display = (tab_name == "Arm/Disarm")
        self.query_one("#flight_container").display = (tab_name == "Flight")

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        # --- Services ---
        if event.button.id == "enable_service_button":
            result = await self.node.call_enable_service(True)
            self.query_one("#services_feedback", Static).update(f"Enable Service: {result.success}")

        elif event.button.id == "reset_service_button":
            result = await self.node.call_reset_service()
            self.query_one("#services_feedback", Static).update(f"Reset Service: {result.success}")

        # --- Arm/Disarm ---
        elif event.button.id == "arm_button":
            name = self.query_one("#arm_name_input", Input).value
            await self.node.send_arm_goal(name)

        elif event.button.id == "disarm_button":
            name = self.query_one("#arm_name_input", Input).value
            await self.node.send_disarm_goal(name)

        # --- Flight ---
        elif event.button.id == "takeoff_button":
            altitude = float(self.query_one("#takeoff_input", Input).value or 0)
            await self.node.send_takeoff_goal(altitude)

        elif event.button.id == "landing_button":
            await self.node.send_landing_goal()

        elif event.button.id == "navigate_button":
            x = float(self.query_one("#navigate_x", Input).value or 0)
            y = float(self.query_one("#navigate_y", Input).value or 0)
            z = float(self.query_one("#navigate_z", Input).value or 0)
            await self.node.send_navigate_goal(x, y, z)

    # --- Feedback live GUI methods ---
    async def update_arm_feedback(self, msg):
        self.query_one("#arm_feedback", Static).update(f"Feedback ARM: {msg}")

    async def update_disarm_feedback(self, msg):
        self.query_one("#arm_feedback", Static).update(f"Feedback DISARM: {msg}")

    async def update_takeoff_feedback(self, msg):
        self.query_one("#flight_feedback", Static).update(f"Takeoff Altitudine: {msg}")

    async def update_landing_feedback(self, msg):
        self.query_one("#flight_feedback", Static).update(f"Landing Altitudine: {msg}")

    async def update_navigate_feedback(self, msg):
        self.query_one("#flight_feedback", Static).update(f"Navigate Posizione: {msg}")


if __name__ == "__main__":
    TerminalGUI().run()
